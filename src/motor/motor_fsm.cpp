// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "motor_fsm.h"
#include <time/time_control.h>

namespace motor {

    void MotorFsm::setPublisher(MotorEventPublisher<MAX_MOTOR_EVENT_LISTENERS>* publisher) {
        publisher_ = publisher;
    }

    void MotorFsm::setPositionSource(const int32_t* position) {
        positionSource_ = position;
    }

    // ---- Transition helpers ----

    bool MotorFsm::isMoving() const {
        switch (state_) {
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

    bool MotorFsm::transition(MotorFsmState next, MotorEventType eventType) {
        MotorFsmState prev = state_;
        state_ = next;

        if (publisher_ != nullptr) {
            MotorEvent event = {};
            event.type = eventType;
            event.previousState = prev;
            event.newState = next;
            event.positionSteps = (positionSource_ != nullptr) ? *positionSource_ : 0;
            event.timestampMs = ungula::TimeControl::syncNow();
            publisher_->publish(event);
        }
        return true;
    }

    // ---- Enable / Disable ----

    bool MotorFsm::requestEnable() {
        if (state_ != MotorFsmState::Disabled) {
            return false;
        }
        return transition(MotorFsmState::Idle, MotorEventType::StateChanged);
    }

    bool MotorFsm::requestDisable() {
        // Can disable from any state
        return transition(MotorFsmState::Disabled, MotorEventType::Stopped);
    }

    // ---- Motion start ----

    bool MotorFsm::requestMoveForward() {
        if (state_ != MotorFsmState::Idle) {
            return false;
        }
        return transition(MotorFsmState::Starting, MotorEventType::Started);
    }

    bool MotorFsm::requestMoveBackward() {
        if (state_ != MotorFsmState::Idle) {
            return false;
        }
        return transition(MotorFsmState::Starting, MotorEventType::Started);
    }

    bool MotorFsm::requestWaitStart() {
        if (state_ != MotorFsmState::Idle) {
            return false;
        }
        return transition(MotorFsmState::WaitingStart, MotorEventType::StateChanged);
    }

    bool MotorFsm::requestStarting() {
        if (state_ != MotorFsmState::WaitingStart && state_ != MotorFsmState::Idle) {
            return false;
        }
        return transition(MotorFsmState::Starting, MotorEventType::Started);
    }

    bool MotorFsm::requestRunning(Direction dir) {
        if (state_ != MotorFsmState::Starting) {
            return false;
        }
        MotorFsmState target = (dir == Direction::FORWARD) ? MotorFsmState::RunningForward
                                                           : MotorFsmState::RunningBackward;
        return transition(target, MotorEventType::StateChanged);
    }

    // ---- Motion stop ----

    bool MotorFsm::requestDecelerate() {
        if (!isMoving()) {
            return false;
        }
        return transition(MotorFsmState::Decelerating, MotorEventType::StateChanged);
    }

    bool MotorFsm::requestStop() {
        // Active motion (incl. Decelerating, which `isMoving()` covers).
        if (isMoving()) {
            return transition(MotorFsmState::Idle, MotorEventType::Stopped);
        }
        // Auto-clear of the soft terminal states the service timer leaves
        // briefly visible (`TargetReached`, `LimitReached`). `Stall` and
        // `Fault` are NOT cleared here — they require explicit
        // `clearStall()` / `clearFault()` so the host actively
        // acknowledges the condition.
        if (state_ == MotorFsmState::TargetReached ||
            state_ == MotorFsmState::LimitReached) {
            return transition(MotorFsmState::Idle, MotorEventType::StateChanged);
        }
        return false;
    }

    bool MotorFsm::requestEmergencyStop() {
        if (state_ == MotorFsmState::Disabled) {
            return false;
        }
        return transition(MotorFsmState::Idle, MotorEventType::Stopped);
    }

    // ---- Target / Limit / Faults ----

    bool MotorFsm::requestTargetReached() {
        if (!isMoving()) {
            return false;
        }
        return transition(MotorFsmState::TargetReached, MotorEventType::TargetReached);
    }

    bool MotorFsm::requestLimitHit() {
        if (!isMoving()) {
            return false;
        }
        return transition(MotorFsmState::LimitReached, MotorEventType::LimitSwitchHit);
    }

    bool MotorFsm::requestStallDetected() {
        if (!isMoving()) {
            return false;
        }
        return transition(MotorFsmState::Stall, MotorEventType::StallDetected);
    }

    bool MotorFsm::requestFault() {
        if (state_ == MotorFsmState::Disabled) {
            return false;
        }
        return transition(MotorFsmState::Fault, MotorEventType::FaultRaised);
    }

    // ---- Acknowledgements ----

    bool MotorFsm::clearStall() {
        if (state_ != MotorFsmState::Stall) {
            return false;
        }
        return transition(MotorFsmState::Idle, MotorEventType::StateChanged);
    }

    bool MotorFsm::clearFault() {
        if (state_ != MotorFsmState::Fault) {
            return false;
        }
        return transition(MotorFsmState::Idle, MotorEventType::StateChanged);
    }

}  // namespace motor
