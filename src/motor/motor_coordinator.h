// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "i_motor.h"
#include "i_motor_event_listener.h"

/// @brief System-level orchestrator holding a collection of IMotor.
///
/// Works at the domain level: issues commands, consumes events, orchestrates
/// multi-motor behavior. Does not generate step pulses or manage timing.
/// Motors may be local or remote — the coordinator doesn't know or care.

namespace motor {

    constexpr uint8_t MAX_COORDINATOR_MOTORS = 8;

    class MotorCoordinator : public IMotorEventListener {
        public:
            /// @brief Register a motor. Returns false if full.
            bool addMotor(IMotor* mot) {
                if (mot == nullptr || motorCount_ >= MAX_COORDINATOR_MOTORS) {
                    return false;
                }
                motors_[motorCount_] = mot;
                motorCount_++;
                return true;
            }

            /// @brief Access a motor by index. Returns nullptr if out of range.
            IMotor* motor(uint8_t index) {
                if (index >= motorCount_) {
                    return nullptr;
                }
                return motors_[index];
            }

            /// @brief Number of registered motors.
            uint8_t motorCount() const {
                return motorCount_;
            }

            /// @brief Enable all registered motors.
            void enableAll() {
                for (uint8_t idx = 0; idx < motorCount_; idx++) {
                    motors_[idx]->enable();
                }
            }

            /// @brief Emergency stop all motors.
            void emergencyStopAll() {
                for (uint8_t idx = 0; idx < motorCount_; idx++) {
                    motors_[idx]->emergencyStop();
                }
            }

            /// @brief Event handler — receives events from subscribed motors.
            void onMotorEvent(const MotorEvent& event) override {
                lastEvent_ = event;
            }

        private:
            IMotor* motors_[MAX_COORDINATOR_MOTORS] = {};
            uint8_t motorCount_ = 0;
            MotorEvent lastEvent_ = {};
    };

}  // namespace motor
