// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "../motor_types.h"
#include "i_homing_strategy.h"

namespace motor {

    /// @brief Home against a limit switch (mechanical, optical, hall…).
    ///
    /// Sequence:
    ///   1. Fast approach in the homing direction until the motor reports
    ///      LimitReached. If the switch was already closed at start(), the
    ///      FSM fires LimitReached on the first service tick and the flow
    ///      still works — no separate pre-check needed.
    ///   2. Back off a small distance to release the switch.
    ///   3. Optional slow re-approach so the final hit is repeatable.
    ///   4. Reset the motor position to 0.
    ///
    /// Requirements on the motor before start():
    ///   - The relevant limit switch must be registered on the motor via
    ///     addLimitBackward() or addLimitForward(), matching homingDirection.
    ///     LocalMotor stops the motor and sets LimitReached automatically.
    ///   - The driver and FSM must be enabled.
    ///
    /// This strategy never reads the switch GPIO itself — LocalMotor owns the
    /// debounce and polarity logic. The strategy just watches the FSM.
    class LimitSwitchHomingStrategy : public IHomingStrategy {
        public:
            struct Config {
                    Direction homingDirection = Direction::BACKWARD;
                    int32_t fastSpeedSps = 2000;
                    uint32_t fastAccelMs = 200U;
                    int32_t slowSpeedSps = 500;
                    uint32_t slowAccelMs = 100U;
                    int32_t backoffSteps = 200;
                    bool finalApproach = true;
            };

            explicit LimitSwitchHomingStrategy(const Config& cfg);

            void begin(IHomeableMotor& motor) override;
            bool tick(IHomeableMotor& motor) override;
            void finish(IHomeableMotor& motor, bool succeeded) override;

            bool succeeded() const override {
                return succeeded_;
            }

        private:
            enum class Phase : uint8_t {
                FastApproach,  // drive into the switch at full speed.
                Backoff,       // step off the switch.
                SlowApproach,  // repeatable final hit.
                Done
            };

            void startApproach(IHomeableMotor& motor, int32_t speedSps, uint32_t accelMs);
            void startBackoff(IHomeableMotor& motor);

            Config cfg_;
            Phase phase_ = Phase::FastApproach;
            bool succeeded_ = false;
    };

}  // namespace motor
