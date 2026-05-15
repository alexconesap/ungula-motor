// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/actuator/can_servo_actuator.h"

namespace ungula::motor
{

CanServoActuator::CanServoActuator(ungula::hal::can::Can &transport, ICanServoProtocol *protocol,
                                   const Config &cfg)
        : transport_(transport)
        , protocol_(protocol)
        , cfg_(cfg)
{
        (void)transport_; // borrowed; protocol implementations use it directly.
}

// =====================================================================
// Lifecycle
// =====================================================================

Status CanServoActuator::begin()
{
        if (begun_)
                return Status::Err(ErrorCode::AlreadyInitialized);

        // Without a protocol attached the actuator can't talk to the
        // drive. Refuse rather than silently "succeed" — the host needs
        // to know its drive isn't reachable.
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        const auto s = protocol_->initializeDrive();
        if (!s.ok())
                return s;

        begun_ = true;
        enabled_ = false;
        return Status::Ok();
}

Status CanServoActuator::enable()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        const auto s = protocol_->enableOperation();
        if (!s.ok())
                return s;

        enabled_ = true;
        return Status::Ok();
}

Status CanServoActuator::disable()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        // Halt any in-flight motion before dropping the power stage, same
        // contract as the STEP/DIR side. `Immediate` because "disable now"
        // means now — caller orchestrates decel ramps at the Axis layer.
        if (motionArmed_) {
                (void)protocol_->stop(StopMode::Immediate);
                motionArmed_ = false;
        }

        const auto s = protocol_->disableOperation();
        if (!s.ok())
                return s;

        enabled_ = false;
        return Status::Ok();
}

// =====================================================================
// Motion control
// =====================================================================

Status CanServoActuator::armMotion(const PlannedMove &move)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!enabled_)
                return Status::Err(ErrorCode::NotEnabled);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        // CAN servos take an absolute target + trajectory limits, not a
        // segment list. PlannedMove as currently shaped is STEP/DIR-centric
        // — the Axis facade will need a CAN-flavoured arming path that
        // bypasses MotionPlanner and calls `protocol_->commandPosition()`
        // directly. Until that path lands, refuse rather than mis-translate.
        (void)move;
        return Status::Err(ErrorCode::Unsupported);
}

Status CanServoActuator::startMotion()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!enabled_)
                return Status::Err(ErrorCode::NotEnabled);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        // Paired with `armMotion()` above — nothing to start because nothing
        // could be armed. The CiA-402 "command on arm" model means start is
        // an empty step on this side.
        if (!motionArmed_)
                return Status::Err(ErrorCode::InvalidState);
        return Status::Ok();
}

Status CanServoActuator::stop(StopMode mode)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        // Forward verbatim. Protocols that don't support Decelerate must
        // return Unsupported themselves — we do NOT downgrade silently.
        const auto s = protocol_->stop(mode);
        if (s.ok())
                motionArmed_ = false;
        return s;
}

Status CanServoActuator::emergencyStop()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);

        const auto s = protocol_->stop(StopMode::Emergency);
        if (s.ok())
                motionArmed_ = false;
        return s;
}

// =====================================================================
// State queries
// =====================================================================

AxisFeedback CanServoActuator::feedback() const
{
        if (!protocol_) {
                // No protocol → no feedback. Return defaults; the host gates on
                // `capabilities().hasActualPosition` etc.
                return AxisFeedback{};
        }
        auto r = protocol_->readFeedback();
        if (!r.ok())
                return AxisFeedback{};
        return r.takeValue();
}

ActuatorCapabilities CanServoActuator::capabilities() const
{
        ActuatorCapabilities c;
        // CAN servos: no GPIO enable, no DIR pin, no pulse engine.
        c.hasEnablePin = false;
        c.hasDirectionPin = false;
        c.hasPulseEngine = false;
        c.hasActualPosition = cfg_.hasActualPosition;
        c.hasInPositionInput = cfg_.hasInPositionInput;
        c.hasAlarmInput = cfg_.hasAlarmInput;
        // Following error is encoder-derived. CAN servos with feedback
        // expose it; without feedback the drive can't compute it either.
        c.hasFollowingError = cfg_.hasActualPosition;
        c.hasNativeHoming = cfg_.hasNativeHoming;
        c.hasStallDetection = false; // would require protocol query
        return c;
}

FaultStatus CanServoActuator::faultStatus() const
{
        if (!protocol_)
                return FaultStatus{};
        auto r = protocol_->readFaultStatus();
        if (!r.ok()) {
                // Bus query itself failed — surface as a transport fault so the
                // caller can distinguish bus problems from drive problems.
                FaultStatus fs;
                fs.code = FaultCode::TransportFault;
                return fs;
        }
        return r.takeValue();
}

Status CanServoActuator::clearFault()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!protocol_)
                return Status::Err(ErrorCode::Unsupported);
        return protocol_->clearFault();
}

} // namespace ungula::motor
