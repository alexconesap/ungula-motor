// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/result.h"

namespace ungula::motor::tmc2209
{

/// TMC2209 CoolStep configurator. CoolStep watches `SG_RESULT` (the
/// motor-load measurement that StallGuard also reads) and dynamically
/// adjusts coil current: low load → reduce current, save heat; rising
/// load → increase current, prevent stalls.
///
/// Replaces the host-side velocity-based current curves that legacy
/// motion libraries used to implement. CoolStep is strictly better
/// because it keys off ACTUAL mechanical load, not commanded velocity
/// — so light load at high speed still saves heat, and heavy load at
/// low speed still gets full current.
///
/// ## Independent of `Tmc2209StallGuard`
///
/// CoolStep and StallGuard share two pieces of chip state:
///   - `SG_RESULT` (the load measurement). Both read it.
///   - `TCOOLTHRS` (TSTEP threshold for "feature active when motor is
///     moving above this velocity"). Either feature can write it.
///
/// You can use CoolStep alone, StallGuard alone, or both. When using
/// both, set `tCoolThrs` on one of them and leave the other at 0 — the
/// non-writer just reads the value that's already there.
///
/// ## Failure semantics
///
/// `begin()` requires `senseResistorOhms` only if `minCurrentMa` /
/// `currentFloor` is to be enforced — current adjustment happens in
/// the chip, not in this driver. Returns `InvalidConfig` for
/// out-of-range knobs; `TransportError` from UART failures.
class Tmc2209CoolStep {
public:
    /// How aggressively CoolStep ramps current up or down on each
    /// load measurement. `Step1` is gentle (one IRUN step per
    /// adjustment); `Step8` is the fastest. Wires to SEUP / SEDN
    /// (2-bit fields in COOLCONF, exponent of 2).
    enum class StepWidth : uint8_t {
        Step1 = 0,
        Step2 = 1,
        Step4 = 2,
        Step8 = 3,
    };

    /// Minimum current floor for CoolStep when load is light. Wires
    /// to SEIMIN (1-bit in COOLCONF):
    ///   Half    → CoolStep floor = IRUN / 2
    ///   Quarter → CoolStep floor = IRUN / 4 (was the old default)
    enum class CurrentFloor : uint8_t {
        Half    = 0,
        Quarter = 1,
    };

    struct Config {
        /// SG_RESULT threshold below which CoolStep INCREASES current.
        /// 1..15 enables CoolStep; 0 DISABLES the entire feature.
        /// Lower values = trigger sooner (chip is more eager to bump
        /// current). The chip multiplies SEMIN by 32 internally for
        /// the comparison.
        uint8_t loadThresholdToIncrease = 4;

        /// SG_RESULT margin above the increase threshold below which
        /// CoolStep DECREASES current. 0..15. Hysteresis: comparison
        /// is against (SEMIN + SEMAX + 1) * 32 — bigger SEMAX = wider
        /// dead band so the current doesn't oscillate.
        uint8_t loadThresholdToDecreaseMargin = 2;

        /// Current-increment step width when load rises.
        StepWidth currentRampUp = StepWidth::Step1;

        /// Current-decrement step width when load drops. Slower than
        /// `currentRampUp` is the safe default — drop current
        /// gradually so a brief load spike doesn't get under-fed.
        StepWidth currentRampDown = StepWidth::Step2;

        /// Floor that CoolStep will not go below at light load.
        CurrentFloor currentFloor = CurrentFloor::Half;

        /// TSTEP threshold for CoolStep + StallGuard activation
        /// (same register as `Tmc2209StallGuard::Config::tCoolThrs`).
        /// 0 = "don't touch this register here" — relevant when
        /// you're also using StallGuard and want it to own this
        /// value. `0xFFFFF` keeps the feature armed at any non-zero
        /// velocity.
        uint32_t tCoolThrs = 0xFFFFF;
    };

    explicit Tmc2209CoolStep(ITmcUart& uart);

    Tmc2209CoolStep(const Tmc2209CoolStep&)            = delete;
    Tmc2209CoolStep& operator=(const Tmc2209CoolStep&) = delete;

    /// Apply the CoolStep configuration. Writes COOLCONF (and
    /// optionally TCOOLTHRS — see Config). Idempotent.
    Status begin(const Config& cfg);

    /// Disable CoolStep at runtime. Writes COOLCONF.SEMIN = 0, which
    /// is the documented "disable" pattern per datasheet. Other
    /// fields are left in their last-written state so re-enabling
    /// via `begin()` doesn't have to repeat them.
    Status disable();

    // ---- Cached state ------------------------------------------------

    uint32_t shadowCoolconf() const { return shadowCoolconf_; }
    uint32_t shadowTCoolThrs() const { return shadowTCoolThrs_; }

private:
    ITmcUart& uart_;
    uint32_t  shadowCoolconf_  = 0;
    uint32_t  shadowTCoolThrs_ = 0;
};

}  // namespace ungula::motor::tmc2209
