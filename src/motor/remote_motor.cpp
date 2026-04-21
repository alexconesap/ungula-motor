// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "remote_motor.h"

namespace motor {

    RemoteMotor::RemoteMotor(IMotorCommandSink& commandSink, uint8_t motorId)
        : commandSink_(commandSink), motorId_(motorId) {}

    void RemoteMotor::enable() {
        commandSink_.send(motorId_, MotorCommandType::ENABLE);
    }

    void RemoteMotor::disable() {
        commandSink_.send(motorId_, MotorCommandType::DISABLE);
    }

    void RemoteMotor::moveForward() {
        commandSink_.send(motorId_, MotorCommandType::MOVE_FORWARD);
    }

    void RemoteMotor::moveBackward() {
        commandSink_.send(motorId_, MotorCommandType::MOVE_BACKWARD);
    }

    void RemoteMotor::moveTo(float target, DistanceUnit unit) {
        MotorMoveParams params{target, unit};
        commandSink_.sendMove(motorId_, MotorCommandType::MOVE_TO, params);
    }

    void RemoteMotor::moveBy(float delta, DistanceUnit unit) {
        MotorMoveParams params{delta, unit};
        commandSink_.sendMove(motorId_, MotorCommandType::MOVE_BY, params);
    }

    void RemoteMotor::executeProfile(const MotionProfileSpec& profile) {
        commandSink_.sendProfile(motorId_, profile);
    }

    void RemoteMotor::stop() {
        commandSink_.send(motorId_, MotorCommandType::STOP);
    }

    void RemoteMotor::emergencyStop() {
        commandSink_.send(motorId_, MotorCommandType::EMERGENCY_STOP);
    }

    MotorFsmState RemoteMotor::state() const {
        return cachedState_;
    }

    int32_t RemoteMotor::positionSteps() const {
        return cachedPosition_;
    }

    bool RemoteMotor::isMoving() const {
        switch (cachedState_) {
            case MotorFsmState::WaitingStart:
            case MotorFsmState::Starting:
            case MotorFsmState::RunningForward:
            case MotorFsmState::RunningBackward:
            case MotorFsmState::Decelerating:
                return true;
            default:
                return false;
        }
    }

    void RemoteMotor::updateState(MotorFsmState newState, int32_t position) {
        cachedState_ = newState;
        cachedPosition_ = position;
    }

}  // namespace motor
