// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// MotorAxis lifecycle + motion FSM. Ten states cover every motion
/// scenario across the supported hardware. The axis publishes its
/// current state via `state()`; transitions emit `StateChanged` events
/// to any subscribed `IMotorEventListener`.
enum class MotorState : uint8_t {
        Uninitialized = 0, // post-construction, before begin()
        Disabled, // begin() OK, coil de-energised
        Idle, // enable() OK, ready for motion
        Moving, // finite move (moveTo / moveBy) in flight
        Jogging, // indefinite jog (moveForward / moveBackward)
        Homing, // running configured homing strategy
        Stopping, // controlled decel after stop()
        Faulted, // driver / stall fault latched
        EmergencyStopped, // immediate-halt fault latched
};

inline const char *motorStateToString(MotorState s)
{
        switch (s) {
        case MotorState::Uninitialized:
                return "Uninitialized";
        case MotorState::Disabled:
                return "Disabled";
        case MotorState::Idle:
                return "Idle";
        case MotorState::Moving:
                return "Moving";
        case MotorState::Jogging:
                return "Jogging";
        case MotorState::Homing:
                return "Homing";
        case MotorState::Stopping:
                return "Stopping";
        case MotorState::Faulted:
                return "Faulted";
        case MotorState::EmergencyStopped:
                return "EmergencyStopped";
        }
        return "Unknown";
}

/// Why a motion stopped. Reported by the driver / limit system, surfaced
/// via `lastStopReason()` and on `MotionStopped` / `MotionCompleted`
/// events.
enum class StopReason : uint8_t {
        None = 0,
        TargetReached, // planner ran to completion
        UserStop, // host called stop()
        EmergencyStop, // host called emergencyStop() or EmergencyLimit fired
        TravelLimit, // a TravelLimit limit switch fired
        LimitSwitch, // home / generic limit switch fired (during homing)
        StallDetected, // stall sensor / driver-side stall
        DriverFault, // chip-reported fault
        Timeout, // command supervisor timed out
};

inline const char *stopReasonToString(StopReason r)
{
        switch (r) {
        case StopReason::None:
                return "None";
        case StopReason::TargetReached:
                return "TargetReached";
        case StopReason::UserStop:
                return "UserStop";
        case StopReason::EmergencyStop:
                return "EmergencyStop";
        case StopReason::TravelLimit:
                return "TravelLimit";
        case StopReason::LimitSwitch:
                return "LimitSwitch";
        case StopReason::StallDetected:
                return "StallDetected";
        case StopReason::DriverFault:
                return "DriverFault";
        case StopReason::Timeout:
                return "Timeout";
        }
        return "Unknown";
}

/// Fault classes carried on FaultRaised events. None when no fault is
/// active; the others identify the underlying cause for the host's
/// supervisor logic.
enum class FaultCode : uint8_t {
        None = 0,
        Stall,
        DriverShortCircuit,
        DriverOverTemperature,
        DriverUnderVoltage,
        DriverOpenLoad,
        PulseEngineFault,
        TransportError,
        Other,
};

inline const char *faultToString(FaultCode f)
{
        switch (f) {
        case FaultCode::None:
                return "None";
        case FaultCode::Stall:
                return "Stall";
        case FaultCode::DriverShortCircuit:
                return "DriverShortCircuit";
        case FaultCode::DriverOverTemperature:
                return "DriverOverTemperature";
        case FaultCode::DriverUnderVoltage:
                return "DriverUnderVoltage";
        case FaultCode::DriverOpenLoad:
                return "DriverOpenLoad";
        case FaultCode::PulseEngineFault:
                return "PulseEngineFault";
        case FaultCode::TransportError:
                return "TransportError";
        case FaultCode::Other:
                return "Other";
        }
        return "Unknown";
}

/// How to stop the motor when `stop()` is called. The driver maps these
/// to its hardware-specific halt path.
enum class StopMode : uint8_t {
        Decelerate, // controlled decel using the configured profile
        Immediate, // hard stop — used by emergencyStop() and ISR halts
};

} // namespace ungula::motor
