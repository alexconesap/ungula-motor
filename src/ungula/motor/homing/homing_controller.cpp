// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/homing/homing_controller.h"

namespace ungula::motor
{

void HomingController::setStrategy(IHomingStrategy *strategy, uint32_t timeoutMs)
{
        strategy_ = strategy;
        timeoutMs_ = timeoutMs;
}

Status HomingController::begin(IHomingAxis &axis, int64_t nowMs)
{
        if (!strategy_)
                return Status::Err(ErrorCode::Unsupported);
        if (isActive())
                return Status::Err(ErrorCode::MotionInProgress);

        phase_ = HomingPhase::FastApproach;
        failureReason_ = StopReason::None;
        startedAtMs_ = nowMs;

        const auto s = strategy_->begin(axis);
        if (!s.ok()) {
                // Strategy refused to start. Map the rejection to a Failed
                // phase but keep the underlying ErrorCode visible to the
                // caller via the returned Status.
                phase_ = HomingPhase::Failed;
                failureReason_ = StopReason::HomingFailed;
                return s;
        }
        // Sync the controller phase with whatever the strategy moved to
        // inside its begin(). Most strategies stay in FastApproach here
        // but the contract allows them to skip directly to a sub-phase.
        phase_ = strategy_->currentPhase();
        return Status::Ok();
}

void HomingController::tick(IHomingAxis &axis, int64_t nowMs)
{
        if (!isActive())
                return;
        if (!strategy_)
                return;

        // Timeout check first — homing strategies that get stuck don't
        // belong in production. Zero means "no timeout"; only check when
        // timeoutMs_ is non-zero.
        if (timeoutMs_ > 0) {
                const int64_t elapsed = nowMs - startedAtMs_;
                if (elapsed >= static_cast<int64_t>(timeoutMs_)) {
                        abort(axis, StopReason::HomingFailed);
                        return;
                }
        }

        // The strategy is invited to step ONLY when the axis is idle.
        // Mid-motion ticks would just spin in InProgress and would risk
        // racing with motion-complete events. The strategy itself uses
        // `IHomingAxis::isMotionIdle()` internally for the same reason.
        if (!axis.isMotionIdle())
                return;

        advance(axis);
}

void HomingController::advance(IHomingAxis &axis)
{
        const auto progress = strategy_->step(axis);

        // Always reflect the strategy's reported phase. The strategy is
        // the authority — the controller just orchestrates timeouts and
        // termination.
        phase_ = strategy_->currentPhase();

        switch (progress) {
        case HomingProgress::InProgress:
                return;
        case HomingProgress::Succeeded:
                strategy_->finish(axis, true);
                phase_ = HomingPhase::Complete;
                return;
        case HomingProgress::Failed:
                strategy_->finish(axis, false);
                phase_ = HomingPhase::Failed;
                failureReason_ = StopReason::HomingFailed;
                return;
        }
}

void HomingController::abort(IHomingAxis &axis, StopReason reason)
{
        if (!isActive())
                return;

        (void)axis.stopMove();
        if (strategy_)
                strategy_->finish(axis, false);

        phase_ = HomingPhase::Failed;
        failureReason_ = (reason == StopReason::None) ? StopReason::HomingFailed : reason;
}

} // namespace ungula::motor
