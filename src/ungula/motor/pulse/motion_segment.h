// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// One slice of a precomputed step pulse train.
///
/// The planner expresses a move as a sequence of constant-rate segments
/// (acceleration phase, cruise phase, deceleration phase, and inserted
/// stop ramps for soft-stop). The pulse engine consumes one segment at a
/// time, replaying `stepCount` STEP edges at the half-period given by
/// `halfPeriodTicks`.
///
/// "Half period" means: the alarm fires every `halfPeriodTicks` ticks; the
/// STEP pin is toggled on each alarm. One STEP pulse = two alarms (rising
/// edge, falling edge), so a step count of N consumes 2N alarms.
///
/// Using fixed half-periods per segment instead of a per-step velocity
/// curve means:
///   - the ISR does zero floating point math,
///   - re-arming is a single alarm reschedule with the same value,
///   - the planner controls how finely the curve is approximated.
///
/// For a smooth trapezoidal profile the planner inserts several segments
/// across the accel ramp; for a coarse profile, two or three is enough.
struct MotionSegment {
        /// Number of STEP edges to emit at this rate (one STEP = high-low).
        /// `0` is a valid terminator/sentinel meaning "end of move".
        uint32_t stepCount = 0;
        /// Half-period in timer ticks. With the project default 1 MHz tick,
        /// a 1 µs tick → halfPeriodTicks=500 means 1 ms per step, i.e. 1 kHz
        /// step rate.
        uint32_t halfPeriodTicks = 0;
};

} // namespace ungula::motor
