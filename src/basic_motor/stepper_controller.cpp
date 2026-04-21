// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "stepper_controller.h"

#include <hal/gpio/gpio_access.h>

#include <cmath>

#include "i_motor_driver.h"

namespace ungula {
    namespace motor {

        StepperController::StepperController()
            : driver_(nullptr), config_{}, state_{}, stepperRun_(false), alarmTicks_(50) {}

        void StepperController::configure(const StepperConfig& config) {
            config_ = config;
        }

        void StepperController::setDriver(IMotorDriver* driver) {
            driver_ = driver;
        }

        void StepperController::setSpeed(int speedSPS) {
            state_.targetSPS = static_cast<float>(speedSPS);
        }

        void StepperController::setRampTime(uint32_t rampTimeMs) {
            config_.rampTimeMs = rampTimeMs;
        }

        void StepperController::start() {
            if (driver_) {
                driver_->setEnable(true);
            }
        }

        void StepperController::stop() {
            state_.targetSPS = 0.0f;
        }

        void StepperController::hardStop() {
            applyHardStop();
        }

        void StepperController::requestHardStop() {
            state_.hardStopReq = true;
        }

        void StepperController::setDirection(int dir) {
            state_.moveDir = (dir != 0) ? 1 : 0;
            if (driver_) {
                driver_->setDirection(dir != 0);
            }
            ungula::gpio::write(config_.directionPin, state_.moveDir != 0);
        }

        bool StepperController::service(uint32_t nowMs) {
            // Check service interval
            if (state_.lastServiceMs == 0) {
                state_.lastServiceMs = nowMs;
            }
            uint32_t dtMs = nowMs - state_.lastServiceMs;
            if (dtMs < SERVICE_INTERVAL_MS) {
                return false;
            }
            state_.lastServiceMs = nowMs;

            // Check hard stop request
            if (state_.hardStopReq) {
                state_.hardStopReq = false;
                applyHardStop();
                return true;
            }

            // Update ramping
            updateRamp(dtMs);

            // Update step timer
            updateStepTimer();

            return false;
        }

        void StepperController::applyHardStop() {
            stepperRun_ = false;
            state_.targetSPS = 0.0f;
            state_.currentSPS = 0.0f;
            state_.running = false;
            ungula::gpio::setLow(config_.stepPin);
            if (driver_) {
                driver_->setEnable(false);
            }
        }

        void StepperController::updateRamp(uint32_t dtMs) {
            constexpr float MS_TO_SECONDS = 1000.0f;
            constexpr float MIN_ACCEL = 1.0f;

            float accel = (config_.rampTimeMs > 0)
                                  ? (fabsf(state_.targetSPS) / (config_.rampTimeMs / MS_TO_SECONDS))
                                  : 1e9f;
            if (accel < MIN_ACCEL) {
                accel = MIN_ACCEL;
            }

            float diff = state_.targetSPS - state_.currentSPS;
            float maxStep = accel * (dtMs / MS_TO_SECONDS);

            if (diff > maxStep) {
                state_.currentSPS += maxStep;
            } else if (diff < -maxStep) {
                state_.currentSPS -= maxStep;
            } else {
                state_.currentSPS = state_.targetSPS;
            }
        }

        void StepperController::updateStepTimer() {
            if (state_.currentSPS < MotorTimer::MIN_RUNNING_SPS) {
                stepperRun_ = false;
                state_.running = false;
                ungula::gpio::setLow(config_.stepPin);
            } else {
                float togglesPerSec = state_.currentSPS * MotorTimer::TOGGLES_PER_STEP;
                uint32_t ticks = static_cast<uint32_t>(
                        static_cast<float>(MotorTimer::TIMER_FREQ_HZ) / togglesPerSec);
                if (ticks < MotorTimer::MIN_ALARM_TICKS) {
                    ticks = MotorTimer::MIN_ALARM_TICKS;
                }
                alarmTicks_ = ticks;
                stepperRun_ = true;
                state_.running = true;
            }
        }

    }  // namespace motor
}  // namespace ungula
