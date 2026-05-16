// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include "ungula/motor/result.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/drivers/driver_identity.h"
#include "ungula/motor/planning/planned_move.h"
#include "ungula/motor/actuator/actuator_capabilities.h"

namespace ungula::motor
{

/// Abstract actuator. Sits below the Axis facade and hides the difference
/// between a STEP/DIR drive and a CAN drive. This interface deliberately
/// contains NO stepper-specific concepts (no STEP pin, no microsteps, no
/// run current) — those belong on the concrete `StepDirActuator` and the
/// optional `Tmc2209Configurator`.
class IAxisActuator {
    public:
        virtual ~IAxisActuator() = default;

        /// One-shot bring-up. Configures hardware, attaches ISRs / starts the
        /// pulse engine (if any), reads back drive identity (if any).
        virtual Status begin() = 0;

        /// Power-stage enable / disable. For drives without an enable input,
        /// `enable()` is a no-op that returns Ok.
        virtual Status enable() = 0;
        virtual Status disable() = 0;

        /// Pre-load a move. The Axis facade calls this with the output of the
        /// planner. The actuator is free to translate the move (e.g. a CAN
        /// servo sends a position command and ignores the segment list).
        virtual Status armMotion(const PlannedMove &move) = 0;

        /// Start executing the armed move. Returns InvalidState if armMotion
        /// has not been called since the last completion.
        virtual Status startMotion() = 0;

        /// Stop the in-flight move. `StopMode::Decelerate` requires the
        /// planner / actuator to apply a deceleration ramp.
        virtual Status stop(StopMode mode) = 0;
        virtual Status emergencyStop() = 0;

        /// Live feedback snapshot. Safe from any context.
        virtual AxisFeedback feedback() const = 0;

        /// What this actuator supports. Returned once at construction; does
        /// not change at runtime.
        virtual ActuatorCapabilities capabilities() const = 0;

        /// Latched fault state. Cleared via `clearFault()`.
        virtual FaultStatus faultStatus() const = 0;
        virtual Status clearFault() = 0;

        /// Read driver identity (vendor / model / firmware version).
        /// Default implementation returns `Unsupported` so concrete
        /// actuators only override when they actually have a way to
        /// surface identity (typically by being constructed with an
        /// `IDriverIdentityProvider*`). NOT pure virtual on purpose —
        /// most actuators have no identity source and should not be
        /// forced to write boilerplate.
        ///
        /// May block on UART / CAN traffic. NOT for the motion-timing
        /// path — host calls from setup() or a diagnostics task.
        virtual Result<DriverIdentity> readDriverIdentity()
        {
                return Result<DriverIdentity>::Err(ErrorCode::Unsupported);
        }
};

} // namespace ungula::motor
