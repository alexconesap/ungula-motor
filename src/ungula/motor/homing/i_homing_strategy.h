// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/result.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/axis_state.h"

namespace ungula::motor
{

class IHomingAxis; // forward — defined in i_homing_axis.h

enum class HomingPhase : uint8_t; // forward — defined in homing_controller.h

/// Strategy interface for homing routines. The strategy never touches
/// GPIO directly — it issues abstract commands through `IHomingAxis`
/// (jog/stop/await-completion/sense-active/reset-position).
///
/// Each `step()` call returns whether homing is complete. The controller
/// invokes `step()` from task context after each motion event delivered
/// by the actuator, NOT on a polling timer. That is the key fix vs. the
/// old `IHomingStrategy::tick()` which raced against auto-clearing FSM
/// states.
/// State returned by `IHomingStrategy::step()`. Named `HomingProgress` —
/// not `Result` — to avoid shadowing the project's generic `Result<T>`
/// template inside homing code.
enum class HomingProgress : uint8_t {
        InProgress = 0,
        Succeeded,
        Failed,
};

class IHomingStrategy {
    public:
        virtual ~IHomingStrategy() = default;

        /// Called once when the controller starts homing.
        virtual Status begin(IHomingAxis &axis) = 0;

        /// Called every time the axis reports a completion event
        /// (MotionCompleted, MotionStopped, LimitActivated). Returns
        /// `InProgress` while more steps are queued; `Succeeded`/`Failed`
        /// when the strategy is done.
        virtual HomingProgress step(IHomingAxis &axis) = 0;

        /// Final cleanup. Always called once `step()` returns Succeeded or
        /// Failed (or the controller aborts).
        virtual void finish(IHomingAxis &axis, bool succeeded) = 0;

        /// Reports whether the axis is currently sitting on the home
        /// reference at boot — used to seed `isHomed()` without having to
        /// run a full homing cycle on every reset for absolute encoders.
        virtual bool isAtHomeReference(const IHomingAxis &axis) const
        {
                (void)axis;
                return false;
        }

        /// Current strategy phase. Exposed so `HomingController` can
        /// surface it without redefining the same FSM. Strategies must
        /// keep this consistent with their internal state.
        virtual HomingPhase currentPhase() const = 0;
};

} // namespace ungula::motor
