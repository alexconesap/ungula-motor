// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/homing/i_homing_strategy.h"

namespace ungula::motor
{

/// Sensorless homing using the StallGuard input as the reference.
/// Same FSM shape as `LimitSwitchHomingStrategy` but reads stall
/// activation instead of a wired home switch:
///
///   1. **FastApproach**: jog towards the mechanical hard-stop at
///      `fastFeedSps`. Motion ends when the carrier hits the stop and
///      the TMC2209's DIAG pin pulses → the SensorBank latches a
///      stall → the Axis stops the motion.
///   2. **Backoff**: move `backoffSteps` AWAY from the stop so the
///      slow approach starts from a known free-motion state.
///   3. **SlowApproach**: jog towards the stop again at
///      `slowFeedSps`. Same trigger — stall halts the motion. The
///      slower speed makes the contact point repeatable.
///   4. **SetHomePosition**: stop motion, call
///      `axis.resetPosition(homePositionSteps)`.
///
/// Requires a `SensorRole::Stall` sensor wired in the axis config —
/// the strategy reads `isStallActive()` through `IHomingAxis`. If no
/// stall sensor is present every approach will treat motion-end as a
/// jog-safety-bound completion (no stall) and the strategy fails.
///
/// ## Why this differs from a switch-based strategy
///
/// Stall detection has higher latency than a hard switch (the chip
/// needs a few SG_RESULT updates to declare a stall, typically tens
/// of ms). For very precise homing this matters; for "find the
/// mechanical limit" use cases it's fine. The Axis ALWAYS halts the
/// pulse engine in the ISR the moment DIAG fires, so the worst-case
/// overshoot is the chip's stall-detection latency, not the
/// software service tick.
class StallHomingStrategy final : public IHomingStrategy {
public:
    struct Config {
        Direction approachDirection  = Direction::Backward;
        Velocity  fastFeedSps        = 1500;
        Velocity  slowFeedSps        = 200;
        Distance  backoffSteps       = 200;
        /// Position to write into the axis once the slow approach has
        /// stalled. Usually 0 (the mechanical stop IS the origin),
        /// but if you want a calibrated home offset from the stop,
        /// put it here.
        Position  homePositionSteps  = 0;
    };

    explicit StallHomingStrategy(const Config& cfg) : cfg_(cfg) {}

    StallHomingStrategy(const StallHomingStrategy&)            = delete;
    StallHomingStrategy& operator=(const StallHomingStrategy&) = delete;

    // ---- IHomingStrategy --------------------------------------------

    Status         begin(IHomingAxis& axis) override;
    HomingProgress step(IHomingAxis& axis)  override;
    void           finish(IHomingAxis& axis, bool succeeded) override;
    HomingPhase    currentPhase() const override { return phase_; }

private:
    Config       cfg_;
    HomingPhase  phase_ = HomingPhase::Idle;
};

}  // namespace ungula::motor
