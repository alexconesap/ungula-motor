// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "stepper_config.h"

namespace ungula {
    namespace motor {

        class IMotorDriver;

        /// Stepper motor runtime state
        struct StepperState {
                float targetSPS;
                float currentSPS;
                int moveDir;
                bool running;
                bool hardStopReq;
                int32_t positionSteps;
                uint32_t lastServiceMs;

                StepperState()
                    : targetSPS(0.0f),
                      currentSPS(0.0f),
                      moveDir(0),
                      running(false),
                      hardStopReq(false),
                      positionSteps(0),
                      lastServiceMs(0) {}
        };

        /// Generic stepper controller - step generation and ramping only.
        /// No limit switches, jog, watchdog, or driver-specific logic.
        class StepperController {
            public:
                StepperController();

                // Configuration
                void configure(const StepperConfig& config);
                void setDriver(IMotorDriver* driver);
                void setSpeed(int speedSPS);
                void setRampTime(uint32_t rampTimeMs);

                // Control
                void start();
                void stop();
                void hardStop();
                void requestHardStop();
                void setDirection(int dir);

                // Service (call from main loop) - returns true if hard-stopped
                bool service(uint32_t nowMs);

                // State access
                const StepperState& getState() const {
                    return state_;
                }
                bool isRunning() const {
                    return state_.running;
                }
                float getCurrentSPS() const {
                    return state_.currentSPS;
                }
                int getDirection() const {
                    return state_.moveDir;
                }
                int32_t getPositionSteps() const {
                    return state_.positionSteps;
                }
                void resetPosition() {
                    state_.positionSteps = 0;
                }

                // For ISR access (step generation)
                volatile bool* getRunningPtr() {
                    return &stepperRun_;
                }
                volatile uint32_t* getAlarmTicksPtr() {
                    return &alarmTicks_;
                }

            private:
                void applyHardStop();
                void updateRamp(uint32_t dtMs);
                void updateStepTimer();

                IMotorDriver* driver_;
                StepperConfig config_;
                StepperState state_;

                // Volatile for ISR access
                volatile bool stepperRun_;
                volatile uint32_t alarmTicks_;

                static constexpr uint32_t SERVICE_INTERVAL_MS = 10;
        };

    }  // namespace motor
}  // namespace ungula
