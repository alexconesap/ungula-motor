// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "motion_profile.h"
#include "motor_state.h"
#include "motor_types.h"

/// @brief Primary motor abstraction — used by coordinators and application logic.
///
/// Hides all hardware details. Safe to call redundantly. Represents a
/// real-world actuator that may be local or remote.

namespace motor {

    class IMotor {
        public:
            virtual ~IMotor() = default;

            /// @brief Enable the motor driver. Transitions from Disabled to Idle.
            virtual void enable() = 0;

            /// @brief Disable the motor driver. Stops motion, enters Disabled.
            virtual void disable() = 0;

            /// @brief Start continuous forward motion using active profile speed.
            virtual void moveForward() = 0;

            /// @brief Start continuous backward motion using active profile speed.
            virtual void moveBackward() = 0;

            /// @brief Move to absolute position. Unit defaults to STEPS.
            virtual void moveTo(float target, DistanceUnit unit = DistanceUnit::STEPS) = 0;

            /// @brief Move by a relative distance. Unit defaults to STEPS.
            virtual void moveBy(float delta, DistanceUnit unit = DistanceUnit::STEPS) = 0;

            /// @brief Execute a complete motion profile autonomously.
            virtual void executeProfile(const MotionProfileSpec& profile) = 0;

            /// @brief Decelerate to stop gracefully.
            virtual void stop() = 0;

            /// @brief Immediate hard stop — no deceleration ramp.
            virtual void emergencyStop() = 0;

            /// @brief Current FSM state.
            virtual MotorFsmState state() const = 0;

            /// @brief Current position in steps.
            virtual int32_t positionSteps() const = 0;

            /// @brief True if motor is in any motion state.
            virtual bool isMoving() const = 0;
    };

}  // namespace motor
