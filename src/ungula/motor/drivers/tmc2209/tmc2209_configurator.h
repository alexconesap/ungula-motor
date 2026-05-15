// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

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
/// Every method returns `Status`. Failures from the underlying
/// transport propagate as `ErrorCode::TransportError`; invalid
/// parameter values surface as `InvalidConfig`. The configurator
/// makes NO attempt to recover or retry — that's the host's policy
/// decision.
class Tmc2209Configurator {
    public:
        struct Config {
                /// Run current, 0..31 → 1/32 ... 32/32 of full Vref scale. The
                /// physical current depends on Vref and the sense resistor; the
                /// host is responsible for the math.
                uint8_t runCurrent = 16;
                /// Hold current, 0..31. Common practice is half of `runCurrent`
                /// so the motor stays warm but quiet at standstill.
                uint8_t holdCurrent = 8;
                /// Delay (in IHOLDDELAY units of 2^18 clocks ≈ 21 ms each)
                /// before the chip ramps from run → hold after motion stops.
                /// 0..15. Default 1 gives ~21 ms; higher values reduce idle
                /// noise at the cost of warmer coils after a quick stop.
                uint8_t iHoldDelay = 1;
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

        /// Change run/hold current at runtime. Safe to call during motion
        /// (one UART write, ~700 µs at 115200 baud). Out-of-range values
        /// return `InvalidConfig`.
        Status setCurrents(uint8_t runCurrent, uint8_t holdCurrent, uint8_t iHoldDelay = 1);

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
        bool begun_ = false;
};

} // namespace ungula::motor::tmc2209
