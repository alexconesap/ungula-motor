// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/actuator/actuator_capabilities.h"
#include "ungula/motor/actuator/i_axis_actuator.h"
#include "ungula/motor/actuator/i_can_servo_protocol.h"

namespace ungula::hal::can
{
class Can;
}

namespace ungula::motor
{

/// CAN servo actuator. Replaces the STEP/DIR wire protocol with a CAN
/// command stream — there are no pulses, no DIR pin, no enable GPIO.
/// Position / velocity commands are delegated to an `ICanServoProtocol`
/// implementation that knows the wire format (CiA-402, vendor-specific,
/// custom).
///
/// ## Scope (skeleton)
///
/// The constructor accepts a `Can&` transport plus an optional
/// `ICanServoProtocol*`. If the protocol pointer is null, ALL motion-
/// related operations return `ErrorCode::Unsupported` — the actuator
/// links and compiles, but can't actually drive a motor without a
/// concrete protocol attached.
///
/// `commandPosition` translates `PlannedMove` (step-space) into an
/// absolute target step count which the protocol then converts to drive
/// native units via its configured scaling. The planner's segment list
/// is IGNORED at this layer; CAN drives manage their own trajectory.
///
/// Not yet implemented:
///   - Any concrete protocol (CiA-402, etc.).
///   - Bus-level error mapping beyond pass-through.
///   - Native homing dispatch via the protocol — `hasNativeHoming`
///     is reported but the `Axis::home()` path does not yet route
///     through it.
class CanServoActuator final : public IAxisActuator {
    public:
        struct Config {
                /// Node ID on the CAN bus. Vendor-specific meaning; for CiA-402
                /// this is the COB-ID base offset.
                uint8_t nodeId = 0;

                /// Whether the drive reports an alarm via the protocol's
                /// `readFaultStatus()` path. Drives feature-gating in
                /// `capabilities()`.
                bool hasAlarmInput = true;

                /// Whether the protocol exposes an in-position bit.
                bool hasInPositionInput = true;

                /// Whether the drive provides an encoder feedback path.
                /// CiA-402 servos always do; some lower-end drives report only
                /// commanded position.
                bool hasActualPosition = true;

                /// Drive can perform its own homing routine (CiA-402 Homing
                /// Mode 6). Axis-level homing strategies can defer to it
                /// rather than running a software search.
                bool hasNativeHoming = true;
        };

        /// `transport` and (when not null) `protocol` must outlive the
        /// actuator. Axis composes both and guarantees lifetime ordering
        /// via `unique_ptr` declaration order in its private state.
        CanServoActuator(ungula::hal::can::Can &transport, ICanServoProtocol *protocol,
                         const Config &cfg);

        CanServoActuator(const CanServoActuator &) = delete;
        CanServoActuator &operator=(const CanServoActuator &) = delete;

        // ---- IAxisActuator ----------------------------------------------

        Status begin() override;
        Status enable() override;
        Status disable() override;

        Status armMotion(const PlannedMove &move) override;
        Status startMotion() override;

        Status stop(StopMode mode) override;
        Status emergencyStop() override;

        AxisFeedback feedback() const override;
        ActuatorCapabilities capabilities() const override;

        FaultStatus faultStatus() const override;
        Status clearFault() override;

    private:
        ungula::hal::can::Can &transport_;
        ICanServoProtocol *protocol_; // borrowed; may be null
        Config cfg_;

        bool begun_ = false;
        bool enabled_ = false;
        bool motionArmed_ = false;

        // Cache of the most recent armed move so `startMotion()` knows what
        // to command. CAN drives don't take a segment list — they take an
        // absolute target + limits, which we extract from the planner
        // output at `armMotion()` time.
        Position armedTarget_ = 0;
        TrajectoryLimits armedLimits_ = {};
        bool armedIsJog_ = false;
        Direction armedJogDir_ = Direction::Forward;
};

} // namespace ungula::motor
