// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

/// @brief Motor FSM state model — 12 states covering the full lifecycle.

namespace motor {

    enum class MotorFsmState : uint8_t {
        Disabled = 0,     /// Driver disabled, no motion possible.
        Idle,             /// Enabled, stopped, ready for commands.
        WaitingStart,     /// Profile loaded, waiting for startTime.
        Starting,         /// Ramping up from standstill.
        RunningForward,   /// At speed, moving forward.
        RunningBackward,  /// At speed, moving backward.
        Decelerating,     /// Ramping down to stop.
        Stopped,          /// Deceleration complete, transient before Idle.
        TargetReached,    /// Positional move completed.
        LimitReached,     /// Limit switch triggered, motion halted.
        Stall,            /// Stall detected, motor stopped.
        Fault             /// Hardware fault, motor stopped.
    };

    /// @brief Number of FSM states (for array sizing).
    constexpr uint8_t MOTOR_FSM_STATE_COUNT = 12;

    /// @brief Human-readable state name for logging.
    inline const char* fsmStateName(MotorFsmState state) {
        switch (state) {
            case MotorFsmState::Disabled:
                return "Disabled";
            case MotorFsmState::Idle:
                return "Idle";
            case MotorFsmState::WaitingStart:
                return "WaitingStart";
            case MotorFsmState::Starting:
                return "Starting";
            case MotorFsmState::RunningForward:
                return "RunningFwd";
            case MotorFsmState::RunningBackward:
                return "RunningBwd";
            case MotorFsmState::Decelerating:
                return "Decelerating";
            case MotorFsmState::Stopped:
                return "Stopped";
            case MotorFsmState::TargetReached:
                return "TargetReached";
            case MotorFsmState::LimitReached:
                return "LimitReached";
            case MotorFsmState::Stall:
                return "Stall";
            case MotorFsmState::Fault:
                return "Fault";
        }
        return "Unknown";
    }

}  // namespace motor
