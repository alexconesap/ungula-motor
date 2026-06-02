// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include <memory>

#include "ungula/motor/drivers/tmc2209/tmc2209_config.h"
#include "ungula/motor/i_device_transport.h"
#include "ungula/motor/i_motion_planner.h"
#include "ungula/motor/i_motor_driver.h"
#include "ungula/motor/i_step_signal_generator.h"
#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motor_axis_config.h"

namespace ungula::motor::tmc2209
{

/// One-file TMC2209 motor driver. Implements the full `IMotorDriver`
/// contract by composing:
///
///   - An `IDeviceTransport` (byte-level UART pipe — typically one
///     `Tmc2209UartTransport` shared by two drivers on the same bus,
///     each driver carrying its own slave address in `Tmc2209Config`).
///   - An `IStepSignalGenerator` (the host's pulse generator — usually
///     `GptimerStepSignal`, but the driver doesn't care which one as
///     long as the contract is met).
///   - An `IMotionPlanner` (the host's planner — usually
///     `TrapezoidalPlanner`).
///
/// All chip-specific behaviour (TMC datagram framing, GCONF /
/// CHOPCONF / IHOLD_IRUN / SGTHRS / TCOOLTHRS / COOLCONF register
/// layouts, mA-to-CS conversion, intent-to-chopper-mode mapping,
/// SG_RESULT load-percentage translation) is private to the
/// implementation. The host never sees a register name.
///
/// `MotorIntent` mapping:
///
///   - `Quiet`                  → StealthChop, low TPWMTHRS so the
///                                 chip stays in StealthChop across the
///                                 useful speed range.
///   - `HighTorque`             → SpreadCycle.
///   - `EnergySaving` | `Cool`  → CoolStep on,
///                                 plus hold current clamped lower.
///   - `AdaptiveCurrent`        → CoolStep on (overrides the
///                                 axis-level current ramp when the
///                                 chip supports it natively).
///   - `Precision`              → INTPOL on, microstep depth as
///                                 configured. (The chip already
///                                 interpolates to 256× at any MRES.)
///
/// **Stall detection requires StealthChop on TMC2209.** Per datasheet
/// section 14 / 15.1: "StallGuard4 requires use of StealthChop2." If
/// the axis uses `HighTorque` (SpreadCycle), the chip's load
/// estimator does not run and `SG_RESULT` stays near zero regardless
/// of load - DIAG will never assert. Use `Quiet` (or default) when
/// you want stall detection to work; pair with `AdaptiveCurrent` only
/// after the threshold is tuned (CoolStep changes coil current
/// dynamically and is a noise source while tuning).
class Tmc2209Driver final : public IMotorDriver {
    public:
        /// Self-owns constructor (the one most hosts want). The driver
        /// internally allocates an `RmtStepSignal` and a
        /// `TrapezoidalPlanner` and drives them as part of its own
        /// lifecycle. STEP / DIR / EN / DIAG pins come from `cfg`.
        /// `begin()` will call `RmtStepSignal::begin(stepPin, dirPin, ...)`
        /// using the pin / timing fields in `cfg`.
        ///
        /// Use this on every standard ESP32 board where you can
        /// allocate an RMT TX channel and the planner doesn't need to
        /// be shared with anything else.
        Tmc2209Driver(Tmc2209Config cfg, IDeviceTransport &transport);

        /// Pluggable constructor (for tests, host code that needs to
        /// share a planner across many axes, or hosts whose RMT
        /// channels are already in use by another peripheral and want
        /// to inject `GptimerStepSignal` instead). The host owns
        /// `stepSignal` + `planner`; the driver only holds references.
        /// The driver does NOT call `stepSignal.begin()` in this mode
        /// (the host already did).
        Tmc2209Driver(Tmc2209Config cfg, IDeviceTransport &transport,
                      IStepSignalGenerator &stepSignal, IMotionPlanner &planner);

        Tmc2209Driver(const Tmc2209Driver &) = delete;
        Tmc2209Driver &operator=(const Tmc2209Driver &) = delete;

        // ---- IMotorDriver -----------------------------------------------
        Status begin() override;
        Status enable() override;
        Status disable() override;
        Status clearFault() override;

        Status armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps, uint32_t accelSps2,
                       uint32_t decelSps2) override;
        Status armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2) override;
        Status stop(StopMode mode) override;

        DriverMotionStatus motionStatus() const override;
        Position commandedPositionSteps() const override;
        uint32_t commandedSpsNow() const override;
        Status resetPosition(Position newSteps) override;

        DriverIdentity identity() override;
        IntentSupport applyIntent(MotorIntent intent) override;
        void fillDriverDiagnostics(MotorDiagnostics &out) const override;

        /// Accessor for the active step signal generator (owned or
        /// pluggable). Hosts wiring a `LimitSystem` need this pointer
        /// to install the ISR-context emergency-stop path.
        IStepSignalGenerator *stepSignalForLimits()
        {
                return &stepSignal_;
        }

        /// TMC2209 stall-debug snapshot.
        ///
        /// On the TMC2209 several stall-related registers (SGTHRS,
        /// TCOOLTHRS, COOLCONF, TPWMTHRS) are WRITE-ONLY: reading
        /// them returns 0 regardless of what was written. The driver
        /// caches the values it wrote so the snapshot can report what
        /// the chip is actually using. The read-only / read-write
        /// registers (SG_RESULT, GCONF, CHOPCONF, IOIN, TSTEP,
        /// DRV_STATUS) come straight from the chip.
        ///
        /// Blocks ~5 ms (5 read datagrams at 115200 baud). Use it to
        /// verify configuration and watch SG_RESULT live while you
        /// load the motor by hand.
        struct StallSnapshot {
                // ---- Stall threshold (write-only on the chip; cached lib-side) ----
                /// Raw byte the lib wrote into SGTHRS (datasheet 5.5.5).
                /// 0..255. Higher = more sensitive.
                uint8_t  sgthrsWritten;
                /// The SG_RESULT value at which the chip pulls DIAG
                /// HIGH. The chip's rule (datasheet 14.1) is
                /// `DIAG asserted when SG_RESULT < SGTHRS * 2`, so this
                /// equals `sgthrsWritten * 2`. Pre-computed so the host
                /// doesn't repeat the multiplication at every dump
                /// site. Range: 0..510.
                uint16_t diagTripsBelowSgResult;

                // ---- TCOOLTHRS (write-only; cached lib-side) ----------------------
                /// Raw 32-bit value the lib wrote into TCOOLTHRS.
                uint32_t tcoolthrsWritten;
                /// The effective 20-bit TSTEP threshold the chip
                /// actually uses (`tcoolthrsWritten & 0xFFFFF`). When
                /// the live TSTEP falls below this value, the chip
                /// arms StallGuard4 (datasheet 14.2).
                uint32_t tcoolthrsClkCycles;

                // ---- COOLCONF (write-only; cached lib-side) -----------------------
                uint32_t coolconfWritten;

                // ---- Live SG_RESULT (R) -------------------------------------------
                /// Raw 32-bit read of register 0x41.
                uint32_t sgResultRaw;
                /// SG_RESULT decoded to its valid 10-bit range
                /// (`sgResultRaw & 0x3FF`). 0 = motor stalled (max
                /// load reported), 1023 = free-running. The chip
                /// asserts DIAG when this drops below
                /// `diagTripsBelowSgResult`.
                uint16_t sgResult;

                // ---- Other readable diagnostics -----------------------------------
                uint32_t gconf;       // 0x00 (R/W): bit 2 = en_SpreadCycle (1 disables StallGuard4)
                uint32_t chopconf;    // 0x6C (R/W): chopper + microstep config
                uint32_t ioin;        // 0x06 (R): input states; bit 4 = DIAG level
                uint32_t tstep;       // 0x12 (R): time between steps in clk; small = fast
                uint32_t drvStatus;   // 0x6F (R): error + driver-status flags
                uint32_t ifcnt;       // 0x02 (R): cumulative count of CRC-OK writes the chip accepted

                // ---- Decoded flags ------------------------------------------------
                bool diagLevelHigh;      // decoded IOIN bit 4 (TMC2209 datasheet table 5.5)
                bool stealthChopActive;  // decoded from GCONF bit 2 (StallGuard4 needs this true)
                bool ok;                 // true only if every chip read succeeded
        };
        Result<StallSnapshot> readStallSnapshot();

        // ---- Chip-side helpers exposed for tests ------------------------
        /// Convert mA RMS to the chip's 5-bit `CS` field given the
        /// sense resistor and vsense flag. Pure math — exposed so
        /// tests + hosts can reason about the current actually
        /// programmed without re-reading the chip.
        static uint8_t milliampsToCs(uint16_t rmsMilliamps, float senseResistorOhms,
                                     bool useHighSensitivity);
        /// Inverse of `milliampsToCs`. Useful for diagnostics.
        static uint16_t csToMilliamps(uint8_t cs, float senseResistorOhms, bool useHighSensitivity);

    private:
        // Register layer (private — TMC framing lives here).
        Status writeRegister(uint8_t reg, uint32_t value);
        Result<uint32_t> readRegister(uint8_t reg);

        // Programming helpers.
        Status writeGconf(bool spreadCycle);
        Status writeChopconf();
        Status writeIholdIrun();
        Status writeStallConfig();
        Status writeCoolStepConfig(bool enabled);
        Status writePwmconf();
        Status clearGstat();
        Status readIdentityFromIoin();
        Status setEnablePinLevel(bool enabled);

        // Intent-derived state.
        bool intentSpreadCycle() const;
        bool intentCoolStepOn() const;

        // Wiring. The two `owned*` smart pointers are non-null only
        // in self-owns mode; in pluggable mode they're null and the
        // references below point to host-owned objects.
        Tmc2209Config cfg_;
        IDeviceTransport &transport_;
        std::unique_ptr<IStepSignalGenerator> ownedStepSignal_;
        std::unique_ptr<IMotionPlanner> ownedPlanner_;
        IStepSignalGenerator &stepSignal_;
        IMotionPlanner &planner_;

        // Cached identity (filled at begin()).
        DriverIdentity identity_{ "Trinamic", "TMC2209", 0, 0, 0 };

        // Cached write-only register values. TMC2209 returns 0 when
        // these are read back, so the lib keeps the last-written
        // value alongside the chip and exposes it via readStallSnapshot.
        uint8_t cachedSgthrs_ = 0;
        uint32_t cachedTcoolthrs_ = 0;
        uint32_t cachedCoolconf_ = 0;

        // Cached intent (last applied).
        MotorIntent activeIntent_ = MotorIntent::Default;

        // Bookkeeping for diagnostics.
        mutable PlannedMove lastPlannedMove_{};
        bool begun_ = false;
};

} // namespace ungula::motor::tmc2209
