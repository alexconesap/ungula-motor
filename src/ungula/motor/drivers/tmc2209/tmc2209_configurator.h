// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/drivers/driver_identity.h"
#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/result.h"

namespace ungula::motor::tmc2209
{

/// Chopper mode selection. The TMC2209 supports two:
///   - **StealthChop** (default): voltage-PWM, near-silent, optimal
///     for ≤ ~200 mm/s. The chip auto-tunes during the first move.
///     This mode is required for using the stall detection DIAG pin.
///   - **SpreadCycle**: classic current-controlled chopper, audible
///     but better at high velocity. Some hosts switch at runtime via
///     TPWMTHRS; this driver just sets the boot mode.
enum class ChopperMode : uint8_t {
        StealthChop = 0,
        SpreadCycle = 1,
};

/// Microstep resolution. The chip exposes 1/8/16/32/64/128/256;
/// `Full` is documented but discouraged (rough motion). MRES field
/// in CHOPCONF is the inverse log: 0 -> 256, 8 -> 1.
enum class Microsteps : uint8_t {
        Full = 0, // = 1 step
        Half = 1,
        Quarter = 2,
        Eighth = 3,
        Sixteenth = 4,
        ThirtySecond = 5,
        SixtyFourth = 6,
        OneTwentyEighth = 7,
        TwoFiftySix = 8,
};

/// Initialises a TMC2209 to a known-good configuration. Touches only
/// the registers needed for motion timing to start working — chopper,
/// microsteps, currents, stealthChop selection, and a one-shot
/// `GSTAT` clear so reset/under-voltage flags don't latch from boot.
///
/// ## Scope
///
/// The configurator is **stateful but read-mostly**: it holds a cached
/// copy of the last value written to each "shadow" register
/// (CHOPCONF, GCONF) so that `setMicrosteps()` after `begin()` doesn't
/// stomp on the chopper config. No diagnostics, no register dumps —
/// those live on `Tmc2209Diagnostics`.
///
/// ## Failure semantics
///
/// Configuration methods return `Status`; identity reads return
/// `Result<DriverIdentity>`. Failures from the underlying transport
/// propagate as `ErrorCode::TransportError`; invalid parameter values
/// surface as `InvalidConfig`. The configurator makes NO attempt to
/// recover or retry — that's the host's policy decision.
class Tmc2209Configurator : public IDriverIdentityProvider {
    public:
        struct Config {
                /// Run-coil current in mA RMS. The configurator converts this
                /// to the chip's 5-bit `IRUN` field using `senseResistorOhms`
                /// and the vsense scaling (see `useHighSensitivity` below).
                /// Out-of-range or zero is rejected with `InvalidConfig`.
                uint16_t runCurrentMa = 800;
                /// Hold-coil current in mA RMS. Drops kicks in
                /// `iHoldDelay * ~21 ms` after motion ends. Common practice:
                /// 30..50% of `runCurrentMa`.
                uint16_t holdCurrentMa = 400;
                /// Delay (in IHOLDDELAY units of 2^18 clocks ≈ 21 ms each)
                /// before the chip ramps from run → hold after motion stops.
                /// 0..15. Default 1 gives ~21 ms; higher values reduce idle
                /// noise at the cost of warmer coils after a quick stop.
                uint8_t iHoldDelay = 1;

                /// Sense resistor on the carrier board, in ohms. Typical
                /// values: 0.11 Ω (BTT TMC2209 boards), 0.075 Ω (Watterott
                /// SilentStepStick), 0.05 Ω (some industrial designs).
                /// **Wrong value here means wrong actual current.** Check
                /// the silk screen on your board.
                float senseResistorOhms = 0.11f;

                /// Vsense bit. The TMC2209 has two current-scaling ranges:
                ///   false → Vfs = 0.325 V (low-sensitivity, the default).
                ///           Best for currents above ~300 mA RMS.
                ///   true  → Vfs = 0.180 V (high-sensitivity).
                ///           Use for low-current setups (≤ ~300 mA RMS) so
                ///           you don't get stuck at IRUN=1 with poor
                ///           resolution. Wires through CHOPCONF[17].
                bool useHighSensitivity = false;

                Microsteps microsteps = Microsteps::Sixteenth;
                ChopperMode mode = ChopperMode::StealthChop;
                /// TOFF in CHOPCONF[3:0]. 0 disables the driver; 3..5 is the
                /// common range. The datasheet's default is 3.
                uint8_t toff = 3;
                /// Enable INTPOL (microstep interpolation to 256). Smooths
                /// step-input motion at coarse MRES; the chip handles it,
                /// the host just turns it on.
                bool interpolate = true;
        };

        /// `uart` must outlive the configurator. Slave addressing is
        /// configured at the `ITmcUart` level so multiple configurators
        /// can share one UART transport — each addressed to a different
        /// chip on the same bus.
        explicit Tmc2209Configurator(ITmcUart &uart);

        Tmc2209Configurator(const Tmc2209Configurator &) = delete;
        Tmc2209Configurator &operator=(const Tmc2209Configurator &) = delete;

        /// Apply the full configuration. Writes (in order):
        ///   - `GCONF`     (chopper mode + PDN_DISABLE so UART can drive
        ///                  the chip even when the STEP pin is idle).
        ///   - `CHOPCONF`  (microsteps + TOFF + INTPOL).
        ///   - `IHOLD_IRUN` (run / hold currents).
        ///   - `GSTAT` 0x07 — clears RESET / DRV_ERR / UV_CP flags
        ///                    accumulated since boot.
        ///
        /// Does NOT read any register. Diagnostics belong on
        /// `Tmc2209Diagnostics`.
        Status begin(const Config &cfg);

        /// Change run/hold current at runtime, in mA RMS. Uses the
        /// `senseResistorOhms` + `useHighSensitivity` values from the
        /// last `begin()`. Safe to call during motion (one UART write,
        /// ~700 µs at 115200 baud). Out-of-range values return
        /// `InvalidConfig`.
        Status setCurrents(uint16_t runCurrentMa, uint16_t holdCurrentMa,
                           uint8_t iHoldDelay = 1);

        /// Static helper for users who want to do the math in their
        /// own code (e.g. for a UI showing both mA and CS register
        /// values). Computes the 5-bit CS field for the given target
        /// RMS current. Returns a clamped value in 0..31 — caller can
        /// check for clipping by re-reading via `csToMilliamps()`.
        static uint8_t milliampsToCs(uint16_t rmsMilliamps,
                                     float    senseResistorOhms,
                                     bool     useHighSensitivity = false);

        /// Inverse of `milliampsToCs()`. Useful for diagnostics: read
        /// `shadowIholdIrun()` then split + report the actual currents
        /// the chip is producing.
        static uint16_t csToMilliamps(uint8_t cs,
                                      float   senseResistorOhms,
                                      bool    useHighSensitivity = false);

        /// Change microstep resolution. Updates the cached CHOPCONF and
        /// writes it back so other CHOPCONF fields are preserved.
        Status setMicrosteps(Microsteps msteps);

        /// Switch chopper mode at runtime. Updates the cached GCONF;
        /// re-writes it. Some applications switch StealthChop ↔
        /// SpreadCycle at the TPWMTHRS velocity threshold rather than
        /// here — that path is host-managed, not driver-managed.
        Status setChopperMode(ChopperMode mode);

        /// Write 0x07 to GSTAT to clear RESET / DRV_ERR / UV_CP. Real
        /// W1C; reading GSTAT does NOT clear it (the old driver had this
        /// bug — keeping this method's body honest is the regression
        /// test in `test_tmc2209_configurator`).
        Status clearGstat();

        // ---- IDriverIdentityProvider ------------------------------------

        /// Reads `IOIN` and extracts the chip's version byte (bits 31:24
        /// per the TMC2209 datasheet — production silicon reports 0x21).
        /// Returns vendor="Trinamic", model="TMC2209", firmwareMajor =
        /// version byte, firmwareMinor=0, rawId = full IOIN word.
        ///
        /// Identity is intrinsic to the driver class: every TMC2209 host
        /// can answer "what chip is this?" via this method directly,
        /// without going through the axis. The axis's universal
        /// `readDriverIdentity()` delegates here when the kit (or a
        /// compose-by-hand host) wires this configurator as the
        /// `IDriverIdentityProvider` on the actuator config.
        Result<DriverIdentity> readDriverIdentity() override;

        // ---- Cached state inspection -------------------------------------
        //
        // Exposed so the diagnostics layer and the host can know what
        // CHOPCONF / GCONF look like without going back to the wire. These
        // are NOT a substitute for reading the chip — they reflect the
        // last value WE wrote, which is what the chip should have but is
        // not guaranteed to be (a power glitch could reset it).

        uint32_t shadowGconf() const
        {
                return shadowGconf_;
        }
        uint32_t shadowChopconf() const
        {
                return shadowChopconf_;
        }
        uint32_t shadowIholdIrun() const
        {
                return shadowIholdIrun_;
        }

    private:
        ITmcUart &uart_;
        uint32_t shadowGconf_ = 0;
        uint32_t shadowChopconf_ = 0;
        uint32_t shadowIholdIrun_ = 0;
        // Cached from begin() so runtime `setCurrents(mA, mA)` can
        // repeat the mA→CS conversion without re-passing the sense
        // resistor / vsense flag.
        float senseResistorOhms_ = 0.0f;
        bool useHighSensitivity_ = false;
        bool begun_ = false;
};

} // namespace ungula::motor::tmc2209
