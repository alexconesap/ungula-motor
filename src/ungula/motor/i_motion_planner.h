// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motor_step_timing.h"
#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// Motion-shape constraints handed to the planner. All values are in the
/// internal step domain — the axis resolves user-facing units to SPS /
/// SPS² before calling the planner. Pulse-width defaults come from the
/// shared `timing::kDefault*` constants so every config struct in the
/// lib draws its defaults from one place.
struct PlannerLimits {
        uint32_t maxVelocitySps = 0;
        uint32_t accelSpsPerSec = 0;
        uint32_t decelSpsPerSec = 0;
        uint32_t hardStepRateCeilingSps = 0;
        uint32_t minPulseHighUs = timing::kDefaultMinPulseHighUs;
        uint32_t minPulseLowUs  = timing::kDefaultMinPulseLowUs;
};

/// Contract for any motion planner. Stage 1 ships one default impl
/// (`TrapezoidalPlanner`). The interface exists so a future S-curve
/// planner can drop in without touching `MotorAxis`.
class IMotionPlanner {
    public:
        virtual ~IMotionPlanner() = default;

        /// Plan an absolute move from `fromSteps` to `toSteps`. If the
        /// signed distance is below the cruise-reachable threshold the
        /// planner emits a two-segment (triangle) profile.
        virtual PlannedMove planMove(Position fromSteps, Position toSteps,
                                     const PlannerLimits &limits, uint32_t timerResolutionHz,
                                     uint32_t minTimerTicks) = 0;

        /// Plan an indefinite jog. `safetyCapSteps` is a planner-level
        /// upper bound (the axis passes a deliberately large number so
        /// the planner doesn't second-guess the host's intent).
        virtual PlannedMove planJog(Direction dir, uint32_t safetyCapSteps,
                                    const PlannerLimits &limits, uint32_t timerResolutionHz,
                                    uint32_t minTimerTicks) = 0;
};

} // namespace ungula::motor
