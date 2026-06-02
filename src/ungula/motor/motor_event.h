// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// Event categories emitted by `MotorAxis`. Listeners filter by `type`
/// on the dispatched `MotorEvent`. Multiple events may fire from one
/// service tick (e.g. `LimitActivated` + `MotionStopped` + `StateChanged`
/// when a TravelLimit halts a jog).
enum class MotorEventType : uint8_t {
        StateChanged,
        MotionStarted,
        MotionStopped,
        MotionCompleted,
        LimitActivated,
        HomingCompleted,
        HomingFailed,
        FaultRaised,
        FaultCleared,
        EmergencyStopped,
};

inline const char *motorEventTypeToString(MotorEventType t)
{
        switch (t) {
        case MotorEventType::StateChanged:
                return "StateChanged";
        case MotorEventType::MotionStarted:
                return "MotionStarted";
        case MotorEventType::MotionStopped:
                return "MotionStopped";
        case MotorEventType::MotionCompleted:
                return "MotionCompleted";
        case MotorEventType::LimitActivated:
                return "LimitActivated";
        case MotorEventType::HomingCompleted:
                return "HomingCompleted";
        case MotorEventType::HomingFailed:
                return "HomingFailed";
        case MotorEventType::FaultRaised:
                return "FaultRaised";
        case MotorEventType::FaultCleared:
                return "FaultCleared";
        case MotorEventType::EmergencyStopped:
                return "EmergencyStopped";
        }
        return "Unknown";
}

/// One snapshot of motor state at the instant an event fires. Flat
/// struct, value-copy semantics — listeners must not retain pointers to
/// it past `onMotorEvent` return.
struct MotorEvent {
        MotorAxisId axisId;
        MotorEventType type = MotorEventType::StateChanged;
        MotorState state = MotorState::Uninitialized;
        Position commandedPosition = 0;
        StopReason stopReason = StopReason::None;
        FaultCode faultCode = FaultCode::None;
};

/// Single-slot subscriber. Hosts that need fan-out implement an internal
/// router. Keeping one slot keeps the axis allocation-free and
/// predictable.
class IMotorEventListener {
    public:
        virtual ~IMotorEventListener() = default;
        virtual void onMotorEvent(const MotorEvent &ev) = 0;
};

} // namespace ungula::motor
