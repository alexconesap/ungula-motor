// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_homing_strategy.h"
#include "ungula/motor/i_limit_system.h"
#include "ungula/motor/motor_state.h"

namespace ungula::motor
{

/// Simple single-phase homing: jog backward until the home sensor
/// activates, then declare the current position to be zero. Sufficient
/// for the documented hardware (rachel_board, nicky_*, fs_uv) where the
/// home reference is mechanically reliable and a single seek is enough.
///
/// A follow-up two-pass strategy (fast approach → backoff → slow
/// approach) can be added later by writing another `IHomingStrategy`
/// implementation and selecting it at axis construction. This Stage 1
/// implementation deliberately keeps homing simple so the FSM coverage
/// is the focus, not the homing math.
class HomeToLimitStrategy final : public IHomingStrategy {
    public:
        explicit HomeToLimitStrategy(ILimitSystem &limits);

        Status start(MotorAxis &axis) override;
        void tick(MotorAxis &axis, int64_t nowMs) override;
        bool isActive() const override
        {
                return active_;
        }
        bool succeeded() const override
        {
                return succeeded_;
        }
        StopReason failureReason() const override
        {
                return failureReason_;
        }
        void cancel() override
        {
                active_ = false;
                phase_ = Phase::Idle;
        }

    private:
        enum class Phase : uint8_t {
                Idle = 0,
                Seeking, // jogging backward toward home
                Done,
        };

        ILimitSystem &limits_;
        Phase phase_ = Phase::Idle;
        bool active_ = false;
        bool succeeded_ = false;
        StopReason failureReason_ = StopReason::None;
};

} // namespace ungula::motor
