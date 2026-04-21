// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "../motor_types.h"
#include "i_homing_strategy.h"

namespace motor {

    /// @brief Home against a hard mechanical stop using the driver's stall
    /// detection (TMC2209 StallGuard, DIAG pin, etc.).
    ///
    /// Sequence:
    ///   1. Fast approach in the homing direction until the motor stalls.
    ///   2. Back off a small distance to clear the stop.
    ///   3. Optional slow re-approach until it stalls again — gives a repeatable
    ///      reference. For applications that only need rough homing, disable
    ///      finalApproach in the Config and the first stall is taken as home.
    ///   4. Reset the motor position to 0.
    ///
    /// Requirements on the motor before start():
    ///   - Driver attached and begin() called.
    ///   - setAutoStopOnStall(true) so the FSM enters Stall on detection.
    ///     The strategy enforces this in begin() but flagging it here too.
    ///   - A mechanical hard stop in the homingDirection that the motor can
    ///     safely push against.
    ///
    /// Overwrites the HOMING motion profile internally. Callers that also use
    /// the HOMING profile for other moves should re-configure it after homing.
    class StallHomingStrategy : public IHomingStrategy {
        public:
            struct Config {
                    Direction homingDirection = Direction::BACKWARD;
                    int32_t fastSpeedSps = 2000;
                    uint32_t fastAccelMs = 200U;
                    int32_t slowSpeedSps = 500;
                    uint32_t slowAccelMs = 100U;
                    int32_t backoffSteps = 200;
                    bool finalApproach = true;  // false = single-stall, faster but less repeatable.
            };

            explicit StallHomingStrategy(const Config& cfg);

            void begin(IHomeableMotor& motor) override;
            bool tick(IHomeableMotor& motor) override;
            void finish(IHomeableMotor& motor, bool succeeded) override;

            bool succeeded() const override {
                return succeeded_;
            }

        private:
            enum class Phase : uint8_t { FastApproach, Backoff, SlowApproach, Done };

            void startApproach(IHomeableMotor& motor, int32_t speedSps, uint32_t accelMs);
            void startBackoff(IHomeableMotor& motor);

            Config cfg_;
            Phase phase_ = Phase::FastApproach;
            bool succeeded_ = false;
    };

}  // namespace motor
