// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/motor/axis_types.h"
#include "ungula/motor/planning/planned_move.h"

namespace ungula::motor
{

/// Builds precomputed `PlannedMove`s from a target + a `TrajectoryLimits`
/// description. Stateless — the same planner instance can plan many
/// moves; nothing carries between calls.
///
/// Algorithms:
///   - **Trapezoidal** when the move is long enough to reach the maximum
///     velocity given the configured accel/decel.
///   - **Triangular** when the move is short; peak velocity is reduced
///     so accel + decel exactly cover the requested distance.
///   - **Single-segment constant velocity** for moves so short
///     (< 4 steps) that a ramp doesn't fit. Half-period is computed
///     from `minPulseHighUs` + `minPulseLowUs` instead.
///
/// All math is integer; no `float` / `double` enters the segment values.
/// (One `sqrt` for the triangular peak — wrapped behind an
/// integer-Newton-method helper to keep the engine code platform-clean.)
///
/// ## Exact step-count guarantee
///
/// The planner GUARANTEES `sum(move.segments[i].stepCount) == |deltaSteps|`
/// for every successful plan. Integer-division rounding error is folded
/// into:
///   - the cruise segment for trapezoidal profiles, and
///   - the LAST decel sub-segment for triangular and stop ramps.
///
/// The user's commanded distance is non-negotiable; if the planner
/// cannot honour it (e.g. the correction would make a decel segment
/// non-positive), the affected segment is removed and the planner
/// re-balances. The only case where `totalSteps` may differ from the
/// request is when the request itself was zero or invalid (empty move
/// returned).
///
/// ## Timer minTicks awareness
///
/// `minHalfPeriodTicks` is the floor enforced on every generated
/// segment's `halfPeriodTicks`. It must match the HAL timer's
/// `HwTimerConfig::minTicks` of the timer that will run the move; the
/// planner caps the peak velocity if the requested rate would generate
/// a half-period below this floor. Pass `0` to disable the check
/// (useful only when the engine validates on `loadMove`).
///
/// All `plan*` calls are pure: they take inputs and return a result; no
/// shared state. Safe to call from any thread, though callers typically
/// invoke them from task context only.
class MotionPlanner {
    public:
        /// Maximum sub-segments per ramp side. With 32 total and 1 reserved
        /// for cruise, this leaves 14 each for accel and decel and a few
        /// spares for soft-stop ramps spliced into in-flight moves.
        static constexpr uint8_t MAX_RAMP_SEGMENTS = 14;

        /// Plan a relative move. `deltaSteps` may be negative; direction is
        /// derived from its sign and stored in the returned `PlannedMove`.
        /// `resolutionHz` is the tick rate of the hardware timer the move
        /// will run on (the planner needs it to convert SPS → half-period
        /// in ticks). `minHalfPeriodTicks` is the timer's minimum schedulable
        /// alarm interval; the planner clamps every generated segment's
        /// half-period to be ≥ this value.
        ///
        /// Returns an empty move (`totalSteps == 0`, `segmentCount == 0`)
        /// when `deltaSteps == 0` or when the trajectory limits are
        /// pathological (zero max velocity / zero accel / zero decel).
        PlannedMove planBy(Distance deltaSteps, const TrajectoryLimits &limits,
                           uint32_t resolutionHz, uint32_t minHalfPeriodTicks = 0) const;

        /// Plan an absolute move from `currentPos` to `targetPos`. Thin
        /// wrapper over `planBy(targetPos - currentPos, ...)`.
        PlannedMove planTo(Position currentPos, Position targetPos, const TrajectoryLimits &limits,
                           uint32_t resolutionHz, uint32_t minHalfPeriodTicks = 0) const;

        /// Plan a long-running move in one direction for `maxSteps`. Used
        /// for `Axis::jog()` — the caller is expected to stop the engine
        /// (via `stop()` or `emergencyStop`) before the full move completes.
        ///
        /// The shape is full-trapezoidal: accel to `maxVelocitySps`, cruise
        /// for the bulk of the distance, decel to 0 at the end. If the
        /// caller never stops it, the move terminates naturally — useful
        /// for tests and as a safety upper bound.
        PlannedMove planJog(Direction direction, uint32_t maxSteps, const TrajectoryLimits &limits,
                            uint32_t resolutionHz, uint32_t minHalfPeriodTicks = 0) const;

        /// Plan a deceleration-only ramp from `currentSps` to 0. Used by
        /// `Axis::stop(StopMode::Decelerate)`: the host loads this move
        /// into the engine (replacing the in-flight move) so motion ramps
        /// down cleanly instead of stopping abruptly.
        ///
        /// The returned move has `totalSteps` = analytical decel distance
        /// from `currentSps` to 0 (folded exactly into the emitted segments,
        /// same step-count guarantee as `planBy`). Direction matches the
        /// supplied argument since the caller knows which way the axis is
        /// moving.
        PlannedMove planStop(Direction direction, Velocity currentSps,
                             const TrajectoryLimits &limits, uint32_t resolutionHz,
                             uint32_t minHalfPeriodTicks = 0) const;
};

} // namespace ungula::motor
