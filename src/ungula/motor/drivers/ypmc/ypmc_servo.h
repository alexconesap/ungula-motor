// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/axis_config.h"
#include "ungula/motor/events/axis_event.h"
#include "ungula/motor/result.h"

namespace ungula::motor::ypmc
{

/// RATTMOTOR YPMC-series AC servo, paired with the S2SVD15 drive.
///
/// The drive is a conventional STEP/DIR (CMD+DIR) industrial servo
/// controller — it does its own position/velocity/torque loops and the
/// host supplies a pulse train. UngulaMotor reuses the regular
/// `Axis::createStepDirServo` path; this module just contributes:
///
///   1. Documented timing / polarity defaults for the S2SVD15 so a host
///      doesn't have to re-derive them from the drive manual.
///   2. `applyDriveDefaults()` — populates a `StepDirServoAxisConfig`
///      with those defaults.
///   3. `BrakeController` — coordinates a host-driven brake relay with
///      the axis lifecycle (the YPMC motor option carries a 24 V
///      holding brake; in many wiring schemes the host sequences it
///      rather than the drive).
///
/// What this module is NOT:
///
///   - It does NOT replace the pulse engine, the actuator, or the
///     axis facade. The drive owns its position loop; we own the
///     pulse train.
///   - It does NOT implement the optional Modbus (RS-232 / RS-485)
///     side-channel. The kit ships without that cable and the bus
///     protocol isn't required for STEP/DIR operation. A Modbus
///     diagnostics layer can be added later under the same
///     `ungula::motor::ypmc` namespace.
///
/// ## Wiring overview (S2SVD15, CN1 connector)
///
/// Mandatory pins (host → drive, single-ended):
///
///   - PUL+ / PUL−        : STEP pulse input. The drive accepts CMD+DIR
///                          (default), CW/CCW, or quadrature pulse
///                          modes — `applyDriveDefaults()` targets
///                          CMD+DIR.
///   - DIR+ / DIR−        : Direction input.
///   - SRV-ON (SVON)      : Servo enable, active-HIGH. Must be asserted
///                          ≥ 50 ms before the drive accepts pulses.
///
/// Optional / recommended (drive → host, open-collector):
///
///   - ALM−               : Alarm output, active-LOW. Wire to a
///                          `SensorRole::CrashLimit` for sub-µs ISR
///                          halt on a drive fault.
///   - COIN / INP         : In-position output, active-HIGH. Inform-
///                          ational; useful for "settled" detection
///                          after a moveTo.
///
/// Brake (when the motor is the "with brake" SKU):
///
///   - BRK control        : 24 V coil through a relay. The brake is
///                          ENGAGED when the coil is de-energised
///                          (mechanical default). Release the brake
///                          BEFORE commanding motion; engage it AFTER
///                          motion stops. See `BrakeController` below.

/// Drive timing requirements per the S2SVD15 datasheet, in CMD+DIR
/// mode. These are conservative; the drive accepts faster but margin
/// matters for noisy industrial wiring.
struct DriveTiming {
        /// Direction must be stable this long before the first STEP
        /// rising edge. S2SVD15 spec is 5 µs minimum; 10 µs is a safer
        /// default for long cable runs.
        uint32_t dirSetupUs = 10;
        /// Minimum STEP high pulse width. Spec: 1 µs; we ask for 3 to
        /// stay comfortably above noise.
        uint32_t minPulseHighUs = 3;
        /// Minimum STEP low pulse width.
        uint32_t minPulseLowUs = 3;
        /// Maximum STEP frequency the drive accepts. Spec: 500 kpps
        /// in CMD+DIR mode. The pulse engine clamps to this; hosts
        /// that need more throughput must reduce the drive's
        /// electronic gear ratio rather than over-clocking the pulse
        /// train.
        uint32_t maxStepRateSps = 500'000;
};

/// Default S2SVD15 timing — conservative, suitable for typical wiring.
inline constexpr DriveTiming kDefaultDriveTiming = {};

/// Polarity defaults for the S2SVD15. None of these are user-tunable
/// at the drive without rewiring + re-parameterising. Documented here
/// so the host doesn't have to guess.
struct DrivePolarity {
        /// SRV-ON / SVON input is active-HIGH on the S2SVD15.
        bool srvOnActiveHigh = true;
        /// ALM− output is open-collector, asserted LOW. Wire as
        /// `SensorPolarity::NormallyClosed` so a missing wire is
        /// also treated as a fault.
        bool alarmActiveLow = true;
        /// COIN / INP output is active-HIGH (LOW = not yet settled).
        bool inPositionActiveHigh = true;
        /// Which DIR level the pulse engine writes for `Direction::Forward`.
        /// `true` (default) → Forward = HIGH, Backward = LOW.
        /// `false` swaps the mapping at the wire level — set this when
        /// the motor's physical Forward doesn't match the lib's logical
        /// Forward (most common cause: motor mounted reversed, or the
        /// drive's electronic gear sign is flipped). Re-wiring DIR+/DIR−
        /// or changing the drive's Pn parameter has the same effect but
        /// this knob is a firmware 'fix' for a 'hardware' problem.
        /// A soft guy solves issues made by a hard guy ;-)
        bool dirActiveHigh = true;
};

inline constexpr DrivePolarity kDefaultDrivePolarity = {};

/// Populate the timing / polarity / SRV-ON pin polarity fields of a
/// `StepDirServoAxisConfig` with the S2SVD15 defaults. The caller is
/// still responsible for filling pins, axis ID, units, and ramp
/// limits.
///
/// ALM and COIN pins, if set on `cfg`, are wired through as a
/// `CrashLimit` sensor (ALM only — COIN stays out of the SensorBank;
/// hosts that want INP poll `feedback().inPosition`).
///
/// Returns `InvalidConfig` if the caller has already filled the
/// sensor slots beyond capacity.
Status applyDriveDefaults(StepDirServoAxisConfig &cfg,
                          const DriveTiming &timing = kDefaultDriveTiming,
                          const DrivePolarity &polarity = kDefaultDrivePolarity);

/// 24 V holding-brake controller for YPMC motors that carry the brake
/// option. The drive itself can manage the brake when the appropriate
/// CN1 pins are wired (BRK+/BRK−); when the host wires its own relay
/// (e.g. through an ESP32 GPIO + opto-MOSFET) this class sequences
/// the brake against the axis lifecycle.
///
/// ## Sequencing contract
///
/// Brake-release MUST precede the first STEP pulse, with enough delay
/// for the mechanical brake to fully release (typical 60–150 ms).
/// Brake-engagement happens AFTER motion has stopped and the motor
/// has settled (typical 10–50 ms). Wrong ordering causes the motor
/// to fight an engaged brake → drive overload trip on the first
/// move, or coast-on-disable when the brake hasn't caught yet.
///
/// This class does NOT auto-release on `MotionStarted` — by the time
/// that event fires, the engine has already armed and may have
/// started pulsing. Hosts call `release()` explicitly between
/// `enable()` and the first motion command, observe the
/// `releaseSettleMs` delay, then command motion.
///
/// On `MotionCompleted` / `MotionStopped` / `EmergencyStopped` /
/// `FaultRaised` the listener auto-engages the brake (after the
/// `engageSettleMs` delay) — that's the safe default. Hosts that want
/// the motor to stay free between moves call `setAutoEngage(false)`.
class BrakeController final : public IAxisEventListener {
    public:
        struct Config {
                /// GPIO pin driving the brake-release coil (typically
                /// through an opto-coupler + power relay). `GPIO_NONE`
                /// disables this controller — `begin()` will reject.
                uint8_t brakeReleasePin = 0xFF;
                /// Active level for the brake-release coil. `true`
                /// (default) means HIGH energises the relay and
                /// releases the brake — matches a typical opto-MOSFET
                /// driven from a 3.3 V GPIO.
                bool brakeReleaseActiveHigh = true;
                /// How long to block in `release()` after asserting
                /// the coil, before returning. The brake's mechanical
                /// release takes 60–150 ms depending on motor frame;
                /// 120 ms is a safe default. Set lower if your motor's
                /// datasheet is faster and you measured it.
                uint32_t releaseSettleMs = 120;
                /// How long to wait between motion-stopped and
                /// de-energising the coil. The motor's rotor needs to
                /// fully settle before the brake catches it.
                uint32_t engageSettleMs = 30;
                /// Whether the listener auto-engages on
                /// `MotionCompleted` / `MotionStopped`. Most
                /// vertical-axis hosts want this; horizontal axes
                /// that prefer to stay free between moves disable it.
                bool autoEngageOnMotionEnd = true;
        };

        explicit BrakeController(const Config &cfg);

        BrakeController(const BrakeController &) = delete;
        BrakeController &operator=(const BrakeController &) = delete;

        /// Configure the GPIO and seed the brake as ENGAGED (coil
        /// de-energised). Idempotent. Returns `InvalidConfig` for an
        /// unset `brakeReleasePin`.
        Status begin();

        /// Energise the brake-release coil and block for
        /// `releaseSettleMs`. Returns `NotInitialized` before
        /// `begin()`.
        Status release();

        /// De-energise the coil and block for `engageSettleMs`.
        /// Returns `NotInitialized` before `begin()`.
        Status engage();

        bool isReleased() const
        {
                return released_;
        }

        void setAutoEngage(bool on)
        {
                cfg_.autoEngageOnMotionEnd = on;
        }

        // ---- IAxisEventListener ----------------------------------

        void onAxisEvent(const AxisEvent &ev) override;

    private:
        Config cfg_;
        bool begun_ = false;
        bool released_ = false;

        void writeCoil(bool energised);
};

} // namespace ungula::motor::ypmc
