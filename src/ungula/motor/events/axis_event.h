// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/motor/axis_types.h"
#include "ungula/motor/axis_state.h"

namespace ungula::motor
{

enum class AxisEventType : uint8_t {
        StateChanged = 0,
        MotionStarted,
        MotionCompleted,
        MotionStopped,
        HomingStarted,
        HomingCompleted,
        HomingFailed,
        LimitActivated,
        FaultRaised,
        FaultCleared,
        EmergencyStopped,
};

inline const char *axisEventTypeToString(AxisEventType t)
{
        switch (t) {
        case AxisEventType::StateChanged:
                return "StateChanged";
        case AxisEventType::MotionStarted:
                return "MotionStarted";
        case AxisEventType::MotionCompleted:
                return "MotionCompleted";
        case AxisEventType::MotionStopped:
                return "MotionStopped";
        case AxisEventType::HomingStarted:
                return "HomingStarted";
        case AxisEventType::HomingCompleted:
                return "HomingCompleted";
        case AxisEventType::HomingFailed:
                return "HomingFailed";
        case AxisEventType::LimitActivated:
                return "LimitActivated";
        case AxisEventType::FaultRaised:
                return "FaultRaised";
        case AxisEventType::FaultCleared:
                return "FaultCleared";
        case AxisEventType::EmergencyStopped:
                return "EmergencyStopped";
        }
        return "Unknown";
}

/// Compact event payload. Filled in by the Axis at the moment of the
/// state change, enqueued, then drained from task context — never
/// delivered to listeners from ISR.
struct AxisEvent {
        uint32_t sequence = 0;
        int64_t timestampMs = 0;
        AxisId axisId = AxisId(0xFF);
        AxisState state = AxisState::Uninitialized;
        Position commandedPosition = 0;
        Position actualPosition = 0;
        bool hasActualPosition = false;
        AxisEventType type = AxisEventType::StateChanged;
        StopReason stopReason = StopReason::None;
        FaultCode faultCode = FaultCode::None;
};

class IAxisEventListener {
    public:
        virtual ~IAxisEventListener() = default;
        /// Called from task context (event-queue drain), never from ISR.
        virtual void onAxisEvent(const AxisEvent &ev) = 0;
};

} // namespace ungula::motor
