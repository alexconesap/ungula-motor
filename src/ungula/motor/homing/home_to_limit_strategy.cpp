// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/homing/home_to_limit_strategy.h"

#include "ungula/motor/motor_axis.h"

namespace ungula::motor
{

HomeToLimitStrategy::HomeToLimitStrategy(ILimitSystem &limits)
        : limits_(limits)
{
}

Status HomeToLimitStrategy::start(MotorAxis &axis)
{
        active_ = true;
        succeeded_ = false;
        failureReason_ = StopReason::None;
        phase_ = Phase::Seeking;

        // Already on the home sensor — declare success immediately.
        if (limits_.isActive(LimitKind::HomeSensor)) {
                // Position will be reset to 0 once the axis transitions
                // through the strategy's completion path in tick(). Skip
                // straight to Done.
                phase_ = Phase::Done;
                succeeded_ = true;
                return Status::Ok();
        }

        // Otherwise begin a backward jog toward home.
        return axis.moveBackward();
}

void HomeToLimitStrategy::tick(MotorAxis &axis, int64_t /*nowMs*/)
{
        if (!active_) {
                return;
        }

        switch (phase_) {
        case Phase::Idle:
                active_ = false;
                return;

        case Phase::Seeking: {
                // Check home sensor first — if it activated, halt and
                // declare success.
                if (limits_.isActive(LimitKind::HomeSensor)) {
                        (void)axis.stop();
                        phase_ = Phase::Done;
                        succeeded_ = true;
                        return;
                }
                // The jog might also have terminated for another reason
                // (TravelLimit on the home side, emergency, fault). The
                // axis stays in `Homing` while the strategy runs (so
                // listeners see the homing phase, not Jogging), which
                // means we can't gate failure detection on `state ==
                // Idle`. Use the explicit motion-in-flight signal
                // instead.
                const MotorState st = axis.state();
                if (st == MotorState::Faulted || st == MotorState::EmergencyStopped) {
                        phase_ = Phase::Done;
                        succeeded_ = false;
                        failureReason_ = axis.lastStopReason();
                        return;
                }
                if (!axis.isMotionInFlight() &&
                    !limits_.isActive(LimitKind::HomeSensor)) {
                        // The jog finished (target reached, travel
                        // limit, etc.) without finding home.
                        phase_ = Phase::Done;
                        succeeded_ = false;
                        failureReason_ = axis.lastStopReason();
                        return;
                }
                return;
        }

        case Phase::Done:
                active_ = false;
                return;
        }
}

} // namespace ungula::motor
