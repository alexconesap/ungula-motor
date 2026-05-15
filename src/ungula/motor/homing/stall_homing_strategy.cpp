// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/homing/stall_homing_strategy.h"

namespace ungula::motor
{

Status StallHomingStrategy::begin(IHomingAxis& axis)
{
    // Fresh cycle — start with FastApproach toward the mechanical stop.
    phase_ = HomingPhase::FastApproach;
    return axis.commandJog(cfg_.approachDirection, cfg_.fastFeedSps);
}

HomingProgress StallHomingStrategy::step(IHomingAxis& axis)
{
    switch (phase_) {
    case HomingPhase::FastApproach: {
        if (!axis.isStallActive()) {
            // Motion completed without a stall — the jog's safety
            // bound expired, meaning we never hit the stop. Wiring
            // or sensitivity issue; fail hard rather than silently
            // re-jog.
            return HomingProgress::Failed;
        }
        phase_ = HomingPhase::Backoff;
        const auto s = axis.commandMove(
            (cfg_.approachDirection == Direction::Forward ? -1 : 1) * cfg_.backoffSteps,
            cfg_.slowFeedSps);
        if (!s.ok()) return HomingProgress::Failed;
        return HomingProgress::InProgress;
    }

    case HomingPhase::Backoff: {
        // Backoff is a fixed-distance move; it shouldn't stall. If
        // it did, the backoff distance is too small or the mechanism
        // is binding — fail.
        if (axis.isStallActive()) {
            return HomingProgress::Failed;
        }
        phase_ = HomingPhase::SlowApproach;
        const auto s = axis.commandJog(cfg_.approachDirection, cfg_.slowFeedSps);
        if (!s.ok()) return HomingProgress::Failed;
        return HomingProgress::InProgress;
    }

    case HomingPhase::SlowApproach: {
        if (!axis.isStallActive()) {
            return HomingProgress::Failed;
        }
        phase_ = HomingPhase::SetHomePosition;
        const auto s = axis.resetPosition(cfg_.homePositionSteps);
        if (!s.ok()) return HomingProgress::Failed;
        phase_ = HomingPhase::Complete;
        return HomingProgress::Succeeded;
    }

    default:
        return HomingProgress::Failed;
    }
}

void StallHomingStrategy::finish(IHomingAxis& axis, bool succeeded)
{
    if (!succeeded) {
        (void)axis.stopMove();
    }
}

}  // namespace ungula::motor
