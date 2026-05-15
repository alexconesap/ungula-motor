// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/homing/limit_switch_homing_strategy.h"

namespace ungula::motor
{

Status LimitSwitchHomingStrategy::begin(IHomingAxis &axis)
{
        if (axis.isHomeActive()) {
                // Already on the switch — skip FastApproach. Backoff first so
                // SlowApproach has a clean off-switch state to start from.
                phase_ = HomingPhase::Backoff;
                return axis.commandMove((cfg_.approachDirection == Direction::Forward ? -1 : 1) *
                                            cfg_.backoffSteps,
                                        cfg_.slowFeedSps);
        }
        phase_ = HomingPhase::FastApproach;
        return axis.commandJog(cfg_.approachDirection, cfg_.fastFeedSps);
}

HomingProgress LimitSwitchHomingStrategy::step(IHomingAxis &axis)
{
        switch (phase_) {
        case HomingPhase::FastApproach: {
                if (!axis.isHomeActive()) {
                        // Motion completed without finding the switch. Either the
                        // jog's safety bound was hit or the host stopped us
                        // externally; either way this isn't a successful home.
                        return HomingProgress::Failed;
                }
                phase_ = HomingPhase::Backoff;
                const auto s = axis.commandMove(
                    (cfg_.approachDirection == Direction::Forward ? -1 : 1) * cfg_.backoffSteps,
                    cfg_.slowFeedSps);
                if (!s.ok())
                        return HomingProgress::Failed;
                return HomingProgress::InProgress;
        }

        case HomingPhase::Backoff: {
                if (axis.isHomeActive()) {
                        // Backoff didn't clear the switch. Either backoffSteps is
                        // too small for the mechanics, or the switch is stuck.
                        return HomingProgress::Failed;
                }
                phase_ = HomingPhase::SlowApproach;
                const auto s = axis.commandJog(cfg_.approachDirection, cfg_.slowFeedSps);
                if (!s.ok())
                        return HomingProgress::Failed;
                return HomingProgress::InProgress;
        }

        case HomingPhase::SlowApproach: {
                if (!axis.isHomeActive()) {
                        // Slow jog completed without the switch firing — see
                        // FastApproach above. Likely a wiring issue.
                        return HomingProgress::Failed;
                }
                phase_ = HomingPhase::SetHomePosition;
                const auto s = axis.resetPosition(cfg_.homePositionSteps);
                if (!s.ok())
                        return HomingProgress::Failed;
                phase_ = HomingPhase::Complete;
                return HomingProgress::Succeeded;
        }

        default:
                // Any other phase reaching step() (Idle/Complete/Failed) is a
                // controller bug — the controller shouldn't tick us there.
                return HomingProgress::Failed;
        }
}

void LimitSwitchHomingStrategy::finish(IHomingAxis &axis, bool succeeded)
{
        if (!succeeded) {
                (void)axis.stopMove();
        }
        // The controller already updates its phase; we don't reset our
        // own phase_ here so a follow-up `currentPhase()` query reflects
        // what we actually reached. The next homing cycle re-enters
        // `begin()` which resets phase_ before commanding motion.
}

} // namespace ungula::motor
