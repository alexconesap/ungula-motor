// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motor_step_timing.h"
#include "ungula/motor/motor_units.h" // GPIO_NONE

namespace ungula::motor::stepdir
{

/// Configuration for any plain STEP/DIR drive (DM542, generic stepper
/// breakouts, anything that takes STEP / DIR / EN pins and has no
/// register-level configuration channel). The driver owns the EN pin
/// only; STEP + DIR are owned by the `IStepSignalGenerator` the host
/// hands the driver.
struct GenericStepDirConfig {
        /// Driver-enable pin. `GPIO_NONE` for drives that hard-wire EN
        /// to ground.
        uint8_t enablePin = GPIO_NONE;
        /// EN polarity. Most stepper drivers (DM542, TB6600, ...) are
        /// active-LOW. Flip per drive datasheet.
        bool enableActiveLow = true;

        // ---- STEP / DIR pins ---------------------------------------------
        // Used ONLY by the self-owns constructor (the one-arg form,
        // `GenericStepDirDriver(cfg)`). The pluggable constructor
        // ignores these because the host's step signal generator was
        // already configured externally with the same info.
        uint8_t stepPin = GPIO_NONE;
        uint8_t dirPin = GPIO_NONE;
        /// True = DIR HIGH means physical forward. Flip if your drive
        /// or wiring inverts the convention.
        bool dirActiveHigh = true;
        /// Settle time between DIR write and the first STEP edge. Most
        /// drives spec 1-5 us; default sits in `timing::kDefaultDirSetupUs`.
        uint32_t dirSetupUs = timing::kDefaultDirSetupUs;
        /// Minimum STEP pulse HIGH width. Default 1 us matches every
        /// drive in the lib's target list; defined once in
        /// `motor_step_timing.h`.
        uint32_t minPulseHighUs = timing::kDefaultMinPulseHighUs;
        /// Minimum STEP pulse LOW width. Same default and constraints.
        uint32_t minPulseLowUs = timing::kDefaultMinPulseLowUs;

        /// Step-signal generator tick rate, in Hz. Used ONLY by the
        /// self-owns constructor — it forwards this to the owned
        /// `RmtStepSignal::Config::resolutionHz`. Pluggable mode
        /// ignores this (the host already configured its generator).
        ///
        /// Higher resolution = finer rung quantization. At 1 MHz a
        /// requested 167 kSPS quantizes to ~125 / ~167 / ~250 kSPS
        /// rungs (huge gaps). 10 MHz makes those gaps ten times
        /// smaller. Cost is per-symbol RMT memory pressure (each
        /// symbol's duration field is 15-bit, so very long pulses
        /// split). For Wendy-class motion (≤ 500 kSPS) 10 MHz is
        /// comfortable and produces honest speed reporting.
        uint32_t resolutionHz = 1'000'000u;
};

} // namespace ungula::motor::stepdir
