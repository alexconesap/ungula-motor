// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// One slice of a precomputed step pulse train.
///
/// The planner expresses a move as a sequence of constant-rate segments
/// (acceleration phase, cruise phase, deceleration phase, and inserted
/// stop ramps for soft-stop). The pulse engine consumes one segment at
/// a time, replaying `stepCount` STEP edges at the half-period given by
/// `halfPeriodTicks`.
///
/// "Half period" means: the alarm fires every `halfPeriodTicks` ticks;
/// the STEP pin is toggled on each alarm. One STEP pulse = two alarms
/// (rising edge, falling edge), so a step count of N consumes 2N alarms.
///
/// Using fixed half-periods per segment instead of a per-step velocity
/// curve means:
///   - the ISR does zero floating point math,
///   - re-arming is a single alarm reschedule with the same value,
///   - the planner controls how finely the curve is approximated.
struct MotionSegment {
        uint32_t stepCount = 0; // 0 = terminator / unused slot
        uint32_t halfPeriodTicks = 0; // generator-resolution ticks
};

/// Compile-time cap on segments per planned move. 32 keeps enough
/// resolution for smooth accel/decel ramps.
constexpr uint8_t MAX_PLANNED_SEGMENTS = 32;

/// Output of any `IMotionPlanner::plan*()`. Self-contained — the step
/// signal generator can execute this without consulting the planner
/// again.
///
/// `totalSteps` MUST equal `sum(segments[i].stepCount)`. The planner
/// is responsible for folding any integer-division rounding error into
/// one specific segment so the host's commanded distance is honoured
/// exactly.
struct PlannedMove {
        Direction direction = Direction::Forward;
        uint32_t totalSteps = 0;
        uint8_t segmentCount = 0;
        // Resolved knobs the planner used (for diagnostics + the
        // generator's "current SPS" computation).
        uint32_t cruiseSps = 0;
        MotionSegment segments[MAX_PLANNED_SEGMENTS]{};
};

} // namespace ungula::motor
