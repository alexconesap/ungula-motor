// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "motor_state.h"

/// @brief Typed motor event model — POD struct, no heap, transport-ready.

namespace motor {

    /// @brief Motor event types.
    enum class MotorEventType : uint8_t {
        Started = 0,
        Stopped,
        StateChanged,
        TargetReached,
        LimitSwitchHit,
        StallDetected,
        FaultRaised
    };

    /// @brief Motor event payload — fixed-size, suitable for local and future remote use.
    struct MotorEvent {
            MotorEventType type;
            MotorFsmState previousState;
            MotorFsmState newState;
            int32_t positionSteps;
            int64_t timestampMs;   /// Stamped with TimeControl::syncNow() at creation
                                   /// time. Signed 64-bit to match the rest of the
                                   /// time API (TimeControl::tick_ms_t) — no narrowing.
    };

}  // namespace motor
