// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// Top-level axis state, surfaced to the application. Stable — does not
/// auto-clear. The old FSM had soft terminal states (TargetReached,
/// LimitReached) that auto-cleared inside one service tick; that is
/// replaced by `lastStopReason()` on the Axis plus explicit events.
enum class AxisState : uint8_t {
        Uninitialized = 0,
        Disabled,
        Idle,
        Moving,
        Jogging,
        Homing,
        Faulted,
        EmergencyStopped,
};

/// What the Axis does automatically when a motion completes (Moving →
/// Idle, Jogging → Idle). Independent of any custom event listener
/// the host might also register.
///
///   - `HoldCurrent` (default): leave the axis enabled. The TMC2209
///     transitions IRUN → IHOLD on standstill detection (after
///     `iHoldDelay`), so coils stay warm-ish but the motor holds
///     position. Right for direct-drive vertical axes, belts under
///     tension, anything that needs holding torque at rest.
///
///   - `AutoDisable`: drop the EN pin at every motion end. Coils
///     freewheel — the carrier is held only by friction or
///     mechanical resistance (non-back-drivable leadscrew, gear
///     train, brake). Cool, silent, but the axis loses position
///     reference if a force pushes it. The host must call
///     `enable()` again before the next motion command.
enum class IdlePolicy : uint8_t {
        HoldCurrent = 0,
        AutoDisable = 1,
};

inline const char *idlePolicyToString(IdlePolicy p)
{
        switch (p) {
        case IdlePolicy::HoldCurrent:
                return "HoldCurrent";
        case IdlePolicy::AutoDisable:
                return "AutoDisable";
        }
        return "Unknown";
}

inline const char *axisStateToString(AxisState s)
{
        switch (s) {
        case AxisState::Uninitialized:
                return "Uninitialized";
        case AxisState::Disabled:
                return "Disabled";
        case AxisState::Idle:
                return "Idle";
        case AxisState::Moving:
                return "Moving";
        case AxisState::Jogging:
                return "Jogging";
        case AxisState::Homing:
                return "Homing";
        case AxisState::Faulted:
                return "Faulted";
        case AxisState::EmergencyStopped:
                return "EmergencyStopped";
        }
        return "Unknown";
}

/// Why the axis last stopped. Latched on motion-end and cleared when the
/// next motion starts. `None` means the axis has never moved (or has
/// been cleared via `clearFault()` / `emergencyStop()` cleanup).
enum class StopReason : uint8_t {
        None = 0,
        TargetReached,
        UserStop,
        EmergencyStop,
        LimitSwitch,
        TravelLimit,
        StallDetected,
        DriverFault,
        TransportFault,
        FollowingError,
        HomingFailed,
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
        case StopReason::LimitSwitch:
                return "LimitSwitch";
        case StopReason::TravelLimit:
                return "TravelLimit";
        case StopReason::StallDetected:
                return "StallDetected";
        case StopReason::DriverFault:
                return "DriverFault";
        case StopReason::TransportFault:
                return "TransportFault";
        case StopReason::FollowingError:
                return "FollowingError";
        case StopReason::HomingFailed:
                return "HomingFailed";
        }
        return "Unknown";
}

/// Latched fault. `None` means no fault. To leave the faulted state the
/// host must call `clearFault()` — there is no auto-clear.
enum class FaultCode : uint8_t {
        None = 0,
        InvalidConfig,
        DriverFault,
        TransportFault,
        Stall,
        LimitExceeded,
        EmergencyStop,
        FollowingError,
        HomingTimeout,
        PulseEngineFault,
};

inline const char *faultToString(FaultCode f)
{
        switch (f) {
        case FaultCode::None:
                return "None";
        case FaultCode::InvalidConfig:
                return "InvalidConfig";
        case FaultCode::DriverFault:
                return "DriverFault";
        case FaultCode::TransportFault:
                return "TransportFault";
        case FaultCode::Stall:
                return "Stall";
        case FaultCode::LimitExceeded:
                return "LimitExceeded";
        case FaultCode::EmergencyStop:
                return "EmergencyStop";
        case FaultCode::FollowingError:
                return "FollowingError";
        case FaultCode::HomingTimeout:
                return "HomingTimeout";
        case FaultCode::PulseEngineFault:
                return "PulseEngineFault";
        }
        return "Unknown";
}

/// Composite fault status — code + optional driver-specific detail.
struct FaultStatus {
        FaultCode code = FaultCode::None;
        /// Driver-specific status word (e.g. TMC2209 DRV_STATUS register).
        /// 0 when the code is not driver-related.
        uint32_t driverDetail = 0;

        bool active() const
        {
                return code != FaultCode::None;
        }
};

} // namespace ungula::motor
