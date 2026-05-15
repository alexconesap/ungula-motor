// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include "ungula/motor/result.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/axis_state.h"

namespace ungula::motor
{

/// Strategy / protocol layer for CAN servo drives. Separates the choice
/// of wire protocol (CiA-402, vendor-specific, JSON-over-CAN, ...) from
/// the transport (lib_hal `ungula::hal::can::Can`).
///
/// `CanServoActuator` borrows an `ICanServoProtocol` reference and a
/// transport reference; protocol implementations talk to the transport
/// through whatever API they need.
class ICanServoProtocol {
    public:
        virtual ~ICanServoProtocol() = default;

        /// Sent once at boot. Negotiate identity, switch the drive into the
        /// state expected by the protocol (CiA-402 "Switched On" + "Operation
        /// Enabled", or vendor-specific equivalent).
        virtual Status initializeDrive() = 0;

        virtual Status enableOperation() = 0;
        virtual Status disableOperation() = 0;
        virtual Status clearFault() = 0;

        /// Command an absolute position with trajectory limits. The protocol
        /// translates `target` from the planner's step-space into the drive's
        /// native units using whatever conversion factor it was configured
        /// with.
        virtual Status commandPosition(Position target, const TrajectoryLimits &limits) = 0;

        /// Command continuous velocity (jog).
        virtual Status commandVelocity(Velocity velocity, const TrajectoryLimits &limits) = 0;

        virtual Status stop(StopMode mode) = 0;

        /// Pull the latest drive state. Returns Unsupported if the protocol
        /// does not implement periodic feedback (rare).
        virtual Result<AxisFeedback> readFeedback() = 0;
        virtual Result<FaultStatus> readFaultStatus() = 0;
};

} // namespace ungula::motor
