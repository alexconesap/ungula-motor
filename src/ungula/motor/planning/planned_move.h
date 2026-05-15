// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/motor/axis_types.h"
#include "ungula/motor/pulse/motion_segment.h"

namespace ungula::motor
{

/// Compile-time cap on the number of segments per planned move. A
/// trapezoidal ramp with reasonable resolution fits in ~16; we keep a
/// margin for jog/stop ramps that splice into an in-flight move.
inline constexpr uint8_t MAX_PLANNED_SEGMENTS = 32;

/// Output of `MotionPlanner::plan*()`. Self-contained — the pulse engine
/// can execute this without re-consulting the planner.
///
/// `totalSteps` is the sum of `segments[i].stepCount` across the move.
/// The pulse engine maintains a running step counter and compares it
/// against the per-segment subtotal to decide when to advance.
struct PlannedMove {
        Direction direction = Direction::Forward;
        uint32_t totalSteps = 0;
        uint8_t segmentCount = 0;
        MotionSegment segments[MAX_PLANNED_SEGMENTS]{};
};

} // namespace ungula::motor
