// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "limit_switch_homing_strategy.h"

#include "i_homeable_motor.h"

namespace motor {

    namespace {

        int32_t signForDirection(Direction dir) {
            return (dir == Direction::FORWARD) ? 1 : -1;
        }

        Direction oppositeOf(Direction dir) {
            return (dir == Direction::FORWARD) ? Direction::BACKWARD : Direction::FORWARD;
        }

    }  // namespace

    LimitSwitchHomingStrategy::LimitSwitchHomingStrategy(const Config& cfg) : cfg_(cfg) {}

    void LimitSwitchHomingStrategy::begin(IHomeableMotor& motor) {
        phase_ = Phase::FastApproach;
        succeeded_ = false;

        // Any residual stall/fault from a prior attempt would block the first
        // move. Limit state itself is cleared by the FSM when we start moving.
        motor.clearStall();
        motor.clearFault();

        startApproach(motor, cfg_.fastSpeedSps, cfg_.fastAccelMs);
    }

    bool LimitSwitchHomingStrategy::tick(IHomeableMotor& motor) {
        const MotorFsmState fsmState = motor.state();

        switch (phase_) {
            case Phase::FastApproach:
                if (fsmState == MotorFsmState::LimitReached) {
                    // LimitReached → Idle via emergencyStop so the next move is accepted.
                    motor.emergencyStop();
                    if (cfg_.finalApproach) {
                        startBackoff(motor);
                        phase_ = Phase::Backoff;
                    } else {
                        phase_ = Phase::Done;
                        succeeded_ = true;
                        return true;
                    }
                } else if (fsmState == MotorFsmState::Fault) {
                    phase_ = Phase::Done;
                    return true;
                }
                return false;

            case Phase::Backoff:
                if (fsmState == MotorFsmState::TargetReached) {
                    motor.emergencyStop();
                    startApproach(motor, cfg_.slowSpeedSps, cfg_.slowAccelMs);
                    phase_ = Phase::SlowApproach;
                } else if (fsmState == MotorFsmState::LimitReached) {
                    // Rare: backoff didn't clear the switch (too few steps). Bail out
                    // — caller should widen backoffSteps.
                    phase_ = Phase::Done;
                    return true;
                } else if (fsmState == MotorFsmState::Fault) {
                    phase_ = Phase::Done;
                    return true;
                }
                return false;

            case Phase::SlowApproach:
                if (fsmState == MotorFsmState::LimitReached) {
                    motor.emergencyStop();
                    phase_ = Phase::Done;
                    succeeded_ = true;
                    return true;
                }
                if (fsmState == MotorFsmState::Fault) {
                    phase_ = Phase::Done;
                    return true;
                }
                return false;

            case Phase::Done:
                return true;
        }
        return true;
    }

    void LimitSwitchHomingStrategy::finish(IHomeableMotor& motor, bool succeeded) {
        if (motor.isMoving()) {
            motor.emergencyStop();
        }
        motor.clearStall();
        motor.clearFault();

        if (succeeded) {
            motor.resetPosition();
        }
    }

    void LimitSwitchHomingStrategy::startApproach(IHomeableMotor& motor, int32_t speedSps,
                                                  uint32_t accelMs) {
        motor.setProfileSpeed(MotionProfile::HOMING, speedSps);
        motor.setProfileAccel(MotionProfile::HOMING, accelMs);
        motor.setProfileDecel(MotionProfile::HOMING, accelMs);
        motor.setActiveProfile(MotionProfile::HOMING);

        if (cfg_.homingDirection == Direction::FORWARD) {
            motor.moveForward();
        } else {
            motor.moveBackward();
        }
    }

    void LimitSwitchHomingStrategy::startBackoff(IHomeableMotor& motor) {
        const Direction away = oppositeOf(cfg_.homingDirection);
        const int32_t deltaSteps = signForDirection(away) * cfg_.backoffSteps;

        motor.setProfileSpeed(MotionProfile::HOMING, cfg_.slowSpeedSps);
        motor.setProfileAccel(MotionProfile::HOMING, cfg_.slowAccelMs);
        motor.setProfileDecel(MotionProfile::HOMING, cfg_.slowAccelMs);
        motor.setActiveProfile(MotionProfile::HOMING);
        motor.moveBy(static_cast<float>(deltaSteps), DistanceUnit::STEPS);
    }

}  // namespace motor
