// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "../i_motor.h"
#include "../motor_types.h"

namespace motor {

    /// @brief IMotor extended with the hooks a homing strategy needs.
    ///
    /// Homing needs to drive the motor's profile table, acknowledge Stall /
    /// Fault, and zero the position counter — none of which belong in the
    /// minimal IMotor interface (a remote motor proxy, for instance, has no
    /// reason to implement them).
    ///
    /// LocalMotor implements this interface. Tests substitute a mock that
    /// records the calls, so homing strategies can be verified without real
    /// hardware.
    class IHomeableMotor : public IMotor {
        public:
            ~IHomeableMotor() override = default;

            /// @brief Enable the stall-auto-stop policy. A stall strategy relies
            /// on the motor transitioning to Stall on its own.
            virtual void setAutoStopOnStall(bool enabled) = 0;

            /// @brief Configure a motion profile's target speed (raw SPS).
            virtual void setProfileSpeed(MotionProfile profile, int32_t speedSps) = 0;

            /// @brief Configure a motion profile's acceleration ramp.
            virtual void setProfileAccel(MotionProfile profile, uint32_t accelMs) = 0;

            /// @brief Configure a motion profile's deceleration ramp.
            virtual void setProfileDecel(MotionProfile profile, uint32_t decelMs) = 0;

            /// @brief Select which profile continuous motion commands use.
            virtual void setActiveProfile(MotionProfile profile) = 0;

            /// @brief Acknowledge a Stall state. Returns the FSM to Idle.
            virtual void clearStall() = 0;

            /// @brief Acknowledge a Fault state. Returns the FSM to Idle.
            virtual void clearFault() = 0;

            /// @brief Zero the step counter (homing reference point).
            virtual void resetPosition() = 0;

            // Note: the per-direction "is the limit currently asserted"
            // query is now part of IMotor as `isLimitActive(Direction)` —
            // strategies call that. No separate hook needed here.
    };

}  // namespace motor
