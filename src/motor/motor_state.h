// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

/// @brief Motor FSM state model — 11 states covering the full lifecycle.
///
/// `Decelerating` collapses straight to `Idle` once the ramp reaches zero;
/// there is no separate "Stopped" stage (older versions had one but no
/// transition ever produced it). Terminal states `TargetReached`,
/// `LimitReached` are auto-cleared back to `Idle` by the service timer
/// after listeners observe them; `Stall` and `Fault` require an explicit
/// `clearStall()` / `clearFault()` acknowledgement.

namespace motor {

    enum class MotorFsmState : uint8_t {
        Disabled = 0,     /// Driver disabled, no motion possible.
        Idle,             /// Enabled, stopped, ready for commands.
        WaitingStart,     /// Profile loaded, waiting for startTime.
        Starting,         /// Ramping up from standstill.
        RunningForward,   /// At speed, moving forward.
        RunningBackward,  /// At speed, moving backward.
        Decelerating,     /// Ramping down to stop.
        TargetReached,    /// Positional move completed (auto-cleared).
        LimitReached,     /// Limit switch triggered, motion halted (auto-cleared).
        Stall,            /// Stall detected — needs `clearStall()`.
        Fault             /// Hardware fault — needs `clearFault()`.
    };

    /// @brief Number of FSM states (for array sizing).
    constexpr uint8_t MOTOR_FSM_STATE_COUNT = 11;

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
