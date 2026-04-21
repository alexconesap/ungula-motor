// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "motor_event.h"
#include "motor_event_publisher.h"
#include "motor_state.h"
#include "motor_types.h"

/// @brief Motor state machine — manages transitions and publishes events.
///
/// The FSM does not own hardware. It receives transition requests and
/// returns whether they were accepted. On each transition it publishes
/// a typed MotorEvent through the attached publisher.

namespace motor {

    /// @brief Max event listeners per motor.
    constexpr uint8_t MAX_MOTOR_EVENT_LISTENERS = 4;

    class MotorFsm {
        public:
            MotorFsmState state() const {
                return state_;
            }

            /// @brief Attach event publisher for state transition notifications.
            void setPublisher(MotorEventPublisher<MAX_MOTOR_EVENT_LISTENERS>* publisher);

            /// @brief Set position source for event stamping.
            void setPositionSource(const int32_t* position);

            // ---- Transition requests ----
            // Each returns true if the transition was accepted.

            bool requestEnable();
            bool requestDisable();
            bool requestMoveForward();
            bool requestMoveBackward();
            bool requestWaitStart();
            bool requestStarting();
            bool requestRunning(Direction dir);
            bool requestDecelerate();
            bool requestStop();
            bool requestEmergencyStop();
            bool requestTargetReached();
            bool requestLimitHit();
            bool requestStallDetected();
            bool requestFault();

            /// @brief Acknowledge stall — transitions Stall to Idle.
            bool clearStall();

            /// @brief Acknowledge fault — transitions Fault to Idle.
            bool clearFault();

            /// @brief True if state is any active-motion state.
            bool isMoving() const;

        private:
            MotorFsmState state_ = MotorFsmState::Disabled;
            MotorEventPublisher<MAX_MOTOR_EVENT_LISTENERS>* publisher_ = nullptr;
            const int32_t* positionSource_ = nullptr;

            bool transition(MotorFsmState next, MotorEventType eventType);
    };

}  // namespace motor
