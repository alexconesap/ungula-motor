// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_motion_planner.h"
#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// Default motion planner. Produces trapezoidal or triangular S-step
/// profiles using integer math only (one isqrt). Stateless — the same
/// planner instance can plan many moves; nothing carries between calls.
///
/// Algorithms:
///   - **Trapezoidal** when the move is long enough to reach the max
///     velocity given the configured accel / decel ramps.
///   - **Triangular** when the move is short; the peak velocity is
///     reduced so accel + decel exactly cover the requested distance.
///   - **Single-segment constant velocity** for moves so short
///     (< 4 steps) that no ramp fits. Half-period derived from the
///     `minPulseHighUs` / `minPulseLowUs` floor.
///
/// Exact step-count guarantee: `sum(move.segments[i].stepCount)` equals
/// the requested distance for every successful plan. Integer-division
/// rounding is folded into the cruise segment for trapezoidal profiles
/// and into the last decel sub-segment for triangular / stop ramps.
class TrapezoidalPlanner final : public IMotionPlanner {
    public:
        static constexpr uint8_t MAX_RAMP_SUB_SEGMENTS = 14;

        PlannedMove planMove(Position fromSteps, Position toSteps, const PlannerLimits &limits,
                             uint32_t timerResolutionHz, uint32_t minTimerTicks) override;

        PlannedMove planJog(Direction dir, uint32_t safetyCapSteps, const PlannerLimits &limits,
                            uint32_t timerResolutionHz, uint32_t minTimerTicks) override;

        /// Decel-only ramp from `currentSps` to 0. Used by the axis
        /// when `stop(Decelerate)` swaps the in-flight move for a
        /// controlled rampdown.
        PlannedMove planStop(Direction dir, uint32_t currentSps, const PlannerLimits &limits,
                             uint32_t timerResolutionHz, uint32_t minTimerTicks) const override;

        /// Returns the actual step rate the planner will emit when the
        /// host asks for `requestedSps` at a generator running at
        /// `resolutionHz`. Rung quantization is unavoidable — the
        /// planner ceiling-rounds the half-period so actual is
        /// `≤ requestedSps`, never higher. Hosts use this for honest
        /// "commanded speed" reporting instead of the un-quantized
        /// request.
        static uint32_t actualSpsFor(uint32_t requestedSps, uint32_t resolutionHz);
};

} // namespace ungula::motor
