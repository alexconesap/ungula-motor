// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include "ungula/motor/axis_types.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Abstract view of an axis that homing strategies are allowed to
/// touch. Deliberately narrow:
///
///   - Strategies cannot call `home()` / `moveTo()` / `subscribe()`.
///   - Strategies cannot touch the Axis FSM directly. They can only
///     request motion through `commandMove*` and observe completion
///     through callbacks delivered by the controller.
///
/// The reason: strategies that compose against the public Axis API
/// can deadlock against the Axis service path (call moveBy → Axis
/// records "moving" → strategy wants to set position before the move
/// completes → Axis rejects "MotionInProgress"). Giving strategies
/// the small subset they actually need avoids the snarl.
class IHomingAxis {
    public:
        virtual ~IHomingAxis() = default;

        /// Request a relative move at the supplied velocity (steps/sec
        /// magnitude). Returns immediately; completion is signalled to the
        /// strategy via `IHomingStrategy::step()` from the controller after
        /// the axis fires a motion-completion event.
        virtual Status commandMove(Distance deltaSteps, Velocity feedSps) = 0;

        /// Jog at constant velocity until `stopMove()` is called or a
        /// sensor halts the axis. Useful for the FastApproach phase where
        /// the total distance to the switch is unknown.
        virtual Status commandJog(Direction direction, Velocity feedSps) = 0;

        /// Immediate stop of the in-flight motion. Returns Ok if the axis
        /// was already idle.
        virtual Status stopMove() = 0;

        /// True when the home reference sensor is currently latched
        /// active. Polled through `SensorBank`.
        virtual bool isHomeActive() const = 0;

        /// True when the last commanded move is no longer in flight
        /// (engine reports `!isRunning`).
        virtual bool isMotionIdle() const = 0;

        /// Set commanded position to `positionSteps` (typically 0 at the
        /// home reference). Requires the axis to be idle; returns
        /// `MotionInProgress` otherwise.
        virtual Status resetPosition(Position positionSteps) = 0;
};

} // namespace ungula::motor
