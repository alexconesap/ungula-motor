// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motor_state.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

class MotorAxis; // forward — strategies command motion on their owning axis

/// Homing FSM contract. The strategy owns its own state and is ticked by
/// the axis from `service()`. Stage 1 ships one default impl —
/// `HomeToLimitStrategy` — which drives the axis backward fast, hits
/// the home sensor, backs off, approaches slowly, then declares the
/// current position to be zero.
///
/// Other strategies (encoder-index, stall-based homing, two-sensor
/// span-and-centre, …) can drop in by implementing this interface.
class IHomingStrategy {
    public:
        virtual ~IHomingStrategy() = default;

        /// Called by `MotorAxis::home()`. Returns immediately after
        /// arming the first move; subsequent progress happens through
        /// `tick()`.
        virtual Status start(MotorAxis &axis) = 0;

        /// Ticked by the axis from `service()`. The strategy inspects
        /// the axis state, advances its phase, and may issue the next
        /// motion command.
        virtual void tick(MotorAxis &axis, int64_t nowMs) = 0;

        /// True while the strategy is mid-flight. Goes false when
        /// `succeeded()` is true OR `failureReason()` is set.
        virtual bool isActive() const = 0;

        virtual bool succeeded() const = 0;
        virtual StopReason failureReason() const = 0;

        /// Abort the strategy NOW and return it to an inactive, neutral
        /// state. Called by `MotorAxis::stop()` / `emergencyStop()` so a
        /// manual stop during homing doesn't leave the strategy lingering
        /// (which would fight the next motion command). Default no-op for
        /// strategies that don't hold abortable state.
        virtual void cancel()
        {
        }
};

} // namespace ungula::motor
