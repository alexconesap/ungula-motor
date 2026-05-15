// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/axis_state.h"
#include "ungula/motor/homing/i_homing_axis.h"
#include "ungula/motor/homing/i_homing_strategy.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Coarse phase of the homing routine. Exposed so hosts (or tests)
/// can observe progress without subscribing to events.
enum class HomingPhase : uint8_t {
        Idle = 0,
        FastApproach,
        Backoff,
        SlowApproach,
        SetHomePosition,
        Complete,
        Failed,
};

inline const char *homingPhaseToString(HomingPhase p)
{
        switch (p) {
        case HomingPhase::Idle:
                return "Idle";
        case HomingPhase::FastApproach:
                return "FastApproach";
        case HomingPhase::Backoff:
                return "Backoff";
        case HomingPhase::SlowApproach:
                return "SlowApproach";
        case HomingPhase::SetHomePosition:
                return "SetHomePosition";
        case HomingPhase::Complete:
                return "Complete";
        case HomingPhase::Failed:
                return "Failed";
        }
        return "Unknown";
}

/// Drives the homing state machine on top of an `IHomingStrategy`.
///
/// The controller doesn't know HOW to home — that's the strategy's
/// job. It only sequences the phases, hands the strategy the axis
/// abstraction, and decides when each phase has succeeded or failed.
///
/// ## Why this exists
///
/// The original `IHomingStrategy::tick()` was polled from the main
/// loop and depended on FSM states that auto-cleared between ticks
/// (`TargetReached`, `LimitReached`). That race produced
/// not-quite-homed axes intermittently. The new contract:
///
///   - The strategy's `step()` is called ONLY when something the
///     controller deems significant happens (motion completes, sensor
///     activates, an axis fault is raised).
///   - The phase is explicit and stable — `SetHomePosition` doesn't
///     auto-clear; the next `tick()` reads it and advances exactly
///     once.
///   - Failure reasons are latched on `Failed` and exposed via
///     `failureReason()`; reading them does not clear them.
///
/// ## Lifecycle
///
///   1. `setStrategy(s, timeoutMs)` — wire a strategy.
///   2. `begin(axis)` — strategy initialises; phase becomes
///      `FastApproach` (the strategy queues the first command in its
///      `begin()` call).
///   3. `tick(nowMs)` — call from `Axis::service()`. The controller
///      observes the axis (motion idle? sensor active?), invokes
///      `strategy.step()` on transitions, and advances the phase.
///   4. Phase becomes `Complete` (succeeded) or `Failed` (timeout /
///      strategy reported failure). `IHomingAxis::stopMove()` is
///      called on Failed to make sure motion is halted.
class HomingController {
    public:
        HomingController() = default;

        HomingController(const HomingController &) = delete;
        HomingController &operator=(const HomingController &) = delete;

        /// `strategy` must outlive any active homing cycle. Pass nullptr
        /// to disable homing — subsequent `begin()` calls return
        /// `InvalidConfig`. `timeoutMs` of zero disables the timeout
        /// (use this carefully — homing strategies that get stuck just
        /// quietly never finish).
        void setStrategy(IHomingStrategy *strategy, uint32_t timeoutMs);

        /// Start a homing cycle. `axis` must outlive the cycle.
        Status begin(IHomingAxis &axis, int64_t nowMs);

        /// Service tick — call from `Axis::service()` regardless of
        /// whether homing is active. No-op if the controller is idle.
        void tick(IHomingAxis &axis, int64_t nowMs);

        /// Abort the current cycle. Stops motion via the axis, marks the
        /// phase Failed with `HomingFailed` reason, latches the supplied
        /// stop reason if non-`None`.
        void abort(IHomingAxis &axis, StopReason reason);

        HomingPhase phase() const
        {
                return phase_;
        }
        bool isActive() const
        {
                return phase_ != HomingPhase::Idle && phase_ != HomingPhase::Complete &&
                       phase_ != HomingPhase::Failed;
        }
        bool succeeded() const
        {
                return phase_ == HomingPhase::Complete;
        }
        StopReason failureReason() const
        {
                return failureReason_;
        }

    private:
        /// Advance the strategy and update phase based on its progress.
        /// Called from `tick()` only when the axis is considered "ready
        /// for the next step" (motion idle, no fault).
        void advance(IHomingAxis &axis);

        IHomingStrategy *strategy_ = nullptr;
        uint32_t timeoutMs_ = 0;

        HomingPhase phase_ = HomingPhase::Idle;
        StopReason failureReason_ = StopReason::None;
        int64_t startedAtMs_ = 0;
        bool stepPendingSetHome_ = false;
};

} // namespace ungula::motor
