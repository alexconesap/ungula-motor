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

            /// @brief True if motor is in any motion state (Starting,
            /// RunningForward, RunningBackward, Decelerating, WaitingStart).
            /// Stable — callers can poll this safely from the main loop
            /// without worrying about terminal FSM states that the service
            /// timer auto-clears on every tick.
            virtual bool isMoving() const = 0;

            // ---- Black-box status queries ----
            //
            // Stable flags that survive the transient FSM terminal states.
            // Callers should not need to reason about TargetReached /
            // LimitReached / Stall / Fault directly — these getters
            // summarise the persistent "what happened" view.

            /// @brief True if the motor last came to rest because a limit
            /// switch fired. Remains true across the FSM's auto-clear back
            /// to Idle. Cleared automatically when motion resumes *and*
            /// every registered limit is no longer asserted — i.e., the
            /// axis has physically backed off the switch.
            virtual bool wasLimitHit() const = 0;

            /// @brief True while an internal homing sequence is still
            /// advancing. Goes false as soon as the sequence terminates for
            /// any reason (success, failure, user stop, emergency stop,
            /// watchdog expiry).
            virtual bool isHoming() const = 0;

            /// @brief True once a homing sequence has completed
            /// successfully. Also seeded at begin() via the configured
            /// homing strategy (limit-switch strategies can look at the
            /// real limit pin and conclude "we're already at home"; stall
            /// strategies have no such signal and start false). Cleared by
            /// any subsequent motion command that is not itself the
            /// internal homing.
            virtual bool isHomed() const = 0;
    };

}  // namespace motor
