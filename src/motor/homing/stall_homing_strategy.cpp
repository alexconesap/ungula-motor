// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "stall_homing_strategy.h"

#include "i_homeable_motor.h"

namespace motor {

    namespace {

        // Sign used for a moveBy delta. Forward is positive, backward negative —
        // matches the convention in LocalMotor::moveBy.
        int32_t signForDirection(Direction dir) {
            return (dir == Direction::FORWARD) ? 1 : -1;
        }

        Direction oppositeOf(Direction dir) {
            return (dir == Direction::FORWARD) ? Direction::BACKWARD : Direction::FORWARD;
        }

    }  // namespace

    StallHomingStrategy::StallHomingStrategy(const Config& cfg) : cfg_(cfg) {}

    void StallHomingStrategy::begin(IHomeableMotor& motor) {
        phase_ = Phase::FastApproach;
        succeeded_ = false;

        // Stall detection is what ends each approach phase. If the caller forgot
        // to turn it on we'd run the motor into the stop forever.
        motor.setAutoStopOnStall(true);

        // Fresh start — any leftover Stall/Fault from a prior run would block
        // the first move.
        motor.clearStall();
        motor.clearFault();

        startApproach(motor, cfg_.fastSpeedSps, cfg_.fastAccelMs);
    }

    bool StallHomingStrategy::tick(IHomeableMotor& motor) {
        const MotorFsmState fsmState = motor.state();

        switch (phase_) {
            case Phase::FastApproach:
                if (fsmState == MotorFsmState::Stall) {
                    // clearStall() brings FSM back to Idle so the next move is accepted.
                    motor.clearStall();
                    if (cfg_.finalApproach) {
                        startBackoff(motor);
                        phase_ = Phase::Backoff;
                    } else {
                        // Single-stall mode: first touch is good enough.
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
                // LocalMotor sits in TargetReached once the relative move lands. FSM
                // will not accept a new command until we push it back to Idle —
                // emergencyStop is the cheap way since the motor is already parked.
                if (fsmState == MotorFsmState::TargetReached) {
                    motor.emergencyStop();
                    startApproach(motor, cfg_.slowSpeedSps, cfg_.slowAccelMs);
                    phase_ = Phase::SlowApproach;
                } else if (fsmState == MotorFsmState::Fault) {
                    phase_ = Phase::Done;
                    return true;
                }
                return false;

            case Phase::SlowApproach:
                if (fsmState == MotorFsmState::Stall) {
                    motor.clearStall();
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

    void StallHomingStrategy::finish(IHomeableMotor& motor, bool succeeded) {
        // Make sure no residual motion carries on after we hand control back.
        if (motor.isMoving()) {
            motor.emergencyStop();
        }
        // Ensure the FSM is clean for the caller. clearStall/clearFault are
        // no-ops if the motor isn't in those states.
        motor.clearStall();
        motor.clearFault();

        if (succeeded) {
            motor.resetPosition();
        }
    }

    void StallHomingStrategy::startApproach(IHomeableMotor& motor, int32_t speedSps,
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

    void StallHomingStrategy::startBackoff(IHomeableMotor& motor) {
        const Direction away = oppositeOf(cfg_.homingDirection);
        const int32_t deltaSteps = signForDirection(away) * cfg_.backoffSteps;

        motor.setProfileSpeed(MotionProfile::HOMING, cfg_.slowSpeedSps);
        motor.setProfileAccel(MotionProfile::HOMING, cfg_.slowAccelMs);
        motor.setProfileDecel(MotionProfile::HOMING, cfg_.slowAccelMs);
        motor.setActiveProfile(MotionProfile::HOMING);
        motor.moveBy(static_cast<float>(deltaSteps), DistanceUnit::STEPS);
    }

}  // namespace motor
