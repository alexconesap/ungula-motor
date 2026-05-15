// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/homing/i_homing_strategy.h"

namespace ungula::motor
{

/// "Classic" homing routine using a single reference switch:
///
///   1. **FastApproach**: jog towards the home direction at
///      `fastFeedSps` until the home sensor activates.
///   2. **Backoff**: move `backoffSteps` AWAY from the switch so the
///      slow approach starts from a stable known-off position.
///   3. **SlowApproach**: jog towards the switch at `slowFeedSps`
///      until it activates again. The slower speed reduces the
///      mechanical overshoot at the activation point — a typical
///      improvement of 10–20× position repeatability.
///   4. **SetHomePosition**: stop motion, call
///      `axis.resetPosition(homePositionSteps)`.
///   5. **Complete**.
///
/// At any phase, motion completing without the home sensor having
/// fired is treated as a stall / hardware-issue failure.
///
/// Direction convention: `approachDirection` is the direction TOWARDS
/// the switch. Backoff goes the opposite way. The Axis layer's
/// TravelLimit sensors are NOT consulted during homing — they may
/// well sit on the same wire as the home switch on cheap setups, and
/// the host's responsibility is to wire them appropriately. (A
/// dedicated home sensor is the strong recommendation.)
class LimitSwitchHomingStrategy final : public IHomingStrategy {
    public:
        struct Config {
                Direction approachDirection = Direction::Backward;
                Velocity fastFeedSps = 2000;
                Velocity slowFeedSps = 200;
                Distance backoffSteps = 200;
                /// Position to write into the axis once the slow approach has
                /// captured the switch. Usually 0 (the home sensor IS the
                /// origin), but absolute machines that want home offset from
                /// the switch use this.
                Position homePositionSteps = 0;
        };

        explicit LimitSwitchHomingStrategy(const Config &cfg)
                : cfg_(cfg)
        {
        }

        LimitSwitchHomingStrategy(const LimitSwitchHomingStrategy &) = delete;
        LimitSwitchHomingStrategy &operator=(const LimitSwitchHomingStrategy &) = delete;

        // ---- IHomingStrategy --------------------------------------------

        Status begin(IHomingAxis &axis) override;
        HomingProgress step(IHomingAxis &axis) override;
        void finish(IHomingAxis &axis, bool succeeded) override;
        HomingPhase currentPhase() const override
        {
                return phase_;
        }

        /// True if the axis is already sitting on the home sensor. The
        /// controller uses this to seed `isHomed()` at boot.
        bool isAtHomeReference(const IHomingAxis &axis) const override
        {
                return axis.isHomeActive();
        }

    private:
        Config cfg_;
        HomingPhase phase_ = HomingPhase::Idle;
};

} // namespace ungula::motor
