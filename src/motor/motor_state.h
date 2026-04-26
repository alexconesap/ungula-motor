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

    /// @brief Stable "why did the last motion end?" answer.
    ///
    /// Latched by `LocalMotor` on every motion-ending FSM transition and
    /// kept until the next motion command starts. Outlives the soft
    /// terminal states (`TargetReached`, `LimitReached`) that the service
    /// timer auto-clears within one tick — so the host can call
    /// `isIdle() && lastStopReason() == StopReason::TargetReached` long
    /// after the FSM itself has moved on to `Idle`.
    enum class StopReason : uint8_t {
        None = 0,         /// No motion has run yet, or a new motion is in progress.
        TargetReached,    /// Positional move completed cleanly.
        UserStop,         /// `stop()` was called and the ramp finished.
        LimitHit,         /// A limit switch fired during motion.
        Stall,            /// Driver reported a stall during motion.
        Fault,            /// Hardware fault was raised.
        EmergencyStop,    /// `emergencyStop()` was called.
        Disabled          /// `disable()` was called.
    };

    /// @brief Human-readable stop-reason name (for logs / UI).
    inline const char* stopReasonName(StopReason reason) {
        switch (reason) {
            case StopReason::None:           return "None";
            case StopReason::TargetReached:  return "TargetReached";
            case StopReason::UserStop:       return "UserStop";
            case StopReason::LimitHit:       return "LimitHit";
            case StopReason::Stall:          return "Stall";
            case StopReason::Fault:          return "Fault";
            case StopReason::EmergencyStop:  return "EmergencyStop";
            case StopReason::Disabled:       return "Disabled";
        }
        return "Unknown";
    }

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
