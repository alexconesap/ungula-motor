// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "step_generator.h"

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_attr.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <hal/gpio/gpio_access.h>

namespace motor {

    // Spinlock protecting shared state between the step ISR, ramp timer,
    // and caller context. On dual-core ESP32, this prevents torn reads
    // when the caller updates multi-field state (speed + rates, target + flags).
    static portMUX_TYPE g_stepMux = portMUX_INITIALIZER_UNLOCKED;

    // ---- Step pulse ISR (IRAM — minimal, no flash access) ----

    static bool IRAM_ATTR onTimerAlarm(gptimer_handle_t timer,
                                       const gptimer_alarm_event_data_t* eventData, void* userCtx) {
        static_cast<StepGenerator*>(userCtx)->handleStepIsr();
        return false;
    }

    void IRAM_ATTR StepGenerator::handleStepIsr() {
        portENTER_CRITICAL_ISR(&g_stepMux);

        if (!running_) {
            stepPinState_ = false;
            portEXIT_CRITICAL_ISR(&g_stepMux);
            ungula::gpio::setLow(stepPin_);
            return;
        }

        stepPinState_ = !stepPinState_;
        const bool isHigh = stepPinState_;

        // Count on rising edge (two toggles per full step).
        // Position increment and target check are one atomic block —
        // prevents dual-core races where another core reads positionSteps_
        // between the increment and the flag update.
        if (isHigh) {
            if (directionFwd_) {
                positionSteps_ = positionSteps_ + 1;
            } else {
                positionSteps_ = positionSteps_ - 1;
            }

            if (hasTarget_) {
                const bool reached = directionFwd_ ? (positionSteps_ >= targetPosition_)
                                                   : (positionSteps_ <= targetPosition_);
                if (reached) {
                    running_ = false;
                    hasTarget_ = false;
                    targetReached_ = true;
                    // Signal serviceRamp() to zero currentSps_/targetSps_.
                    // Float writes belong in task context, not in the ISR.
                    stopRequestedFromIsr_ = true;
                }
            }
        }

        portEXIT_CRITICAL_ISR(&g_stepMux);

        if (isHigh) {
            ungula::gpio::setHigh(stepPin_);
        } else {
            ungula::gpio::setLow(stepPin_);
        }
    }

    // ---- Ramp service (caller's task context — NOT ISR) ----
    //
    // Called by the host on a periodic tick (LocalMotor's 10 ms service).
    // Single-task ownership of ramp + alarm reconfig — no cross-task race
    // against gptimer_set_alarm_action(). Uses measured dt instead of a
    // fixed RAMP_SERVICE_MS so an irregular call cadence (e.g. when the
    // main loop is mid-ADC burst) still produces correct cumulative
    // speed.

    void StepGenerator::serviceRamp(uint32_t nowMs) {
        // Single lock: drain ISR stop request + snapshot running_ atomically.
        portENTER_CRITICAL(&g_stepMux);
        if (stopRequestedFromIsr_) {
            stopRequestedFromIsr_ = false;
            currentSps_ = 0.0F;
            targetSps_ = 0;
        }
        const bool snapRunning = running_;
        portEXIT_CRITICAL(&g_stepMux);

        if (!snapRunning) {
            // Re-arm the dt baseline so the first tick after start uses
            // a sensible delta (whatever start() sets, typically 0).
            lastRampMs_ = nowMs;
            return;
        }

        uint32_t dtMs = nowMs - lastRampMs_;
        lastRampMs_ = nowMs;
        if (dtMs == 0) {
            return;  // called twice in the same ms — nothing to integrate
        }

        // Snapshot ramp parameters under lock, compute outside, write back under lock.
        portENTER_CRITICAL(&g_stepMux);
        const int32_t snapTarget = targetSps_;
        const float snapAccel = accelRate_;
        const float snapDecel = decelRate_;
        const float snapCurrent = currentSps_;
        portEXIT_CRITICAL(&g_stepMux);

        const float newSps = updateRamp(dtMs, snapTarget, snapAccel, snapDecel, snapCurrent);

        portENTER_CRITICAL(&g_stepMux);
        currentSps_ = newSps;
        portEXIT_CRITICAL(&g_stepMux);

        const uint32_t newTicks = computeAlarmTicks(newSps);
        if (newTicks != lastAppliedTicks_) {
            applyAlarmTicks(newTicks);
        }

        // Auto-stop when ramp reaches zero
        if (snapTarget == 0 && newSps < step::MIN_RUNNING_SPS) {
            portENTER_CRITICAL(&g_stepMux);
            running_ = false;
            currentSps_ = 0.0F;
            autoStopped_ = true;
            portEXIT_CRITICAL(&g_stepMux);
            applyAlarmTicks(step::IDLE_ALARM_TICKS);
        }
    }

    // ---- Lifecycle ----

    bool StepGenerator::begin(uint8_t stepPin) {
        if (stepTimerHandle_ != nullptr) {
            end();
        }

        stepPin_ = stepPin;

        // Step pin: output, initially LOW
        // Must use gpio_config() — gpio_set_direction() alone does NOT set
        // IOMUX function to GPIO, so the output never reaches the physical pin.
        gpio_config_t stepConfig = {};
        stepConfig.pin_bit_mask = (1ULL << stepPin_);
        stepConfig.mode = GPIO_MODE_OUTPUT;
        stepConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
        stepConfig.pull_up_en = GPIO_PULLUP_DISABLE;
        stepConfig.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&stepConfig);
        gpio_set_level(static_cast<gpio_num_t>(stepPin_), 0);

        // 1 MHz general-purpose timer for step pulses
        gptimer_config_t timerConfig = {};
        timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        timerConfig.direction = GPTIMER_COUNT_UP;
        timerConfig.resolution_hz = step::TIMER_FREQ_HZ;
        timerConfig.flags.intr_shared = false;

        gptimer_handle_t handle = nullptr;
        if (gptimer_new_timer(&timerConfig, &handle) != ESP_OK) {
            return false;
        }
        stepTimerHandle_ = handle;

        // Alarm callback with this instance as user context
        gptimer_event_callbacks_t callbacks = {};
        callbacks.on_alarm = onTimerAlarm;
        if (gptimer_register_event_callbacks(handle, &callbacks, this) != ESP_OK) {
            gptimer_del_timer(handle);
            stepTimerHandle_ = nullptr;
            return false;
        }

        // Order matters: enable -> set alarm -> start (per ESP-IDF docs)
        if (gptimer_enable(handle) != ESP_OK) {
            gptimer_del_timer(handle);
            stepTimerHandle_ = nullptr;
            return false;
        }

        applyAlarmTicks(step::IDLE_ALARM_TICKS);

        if (gptimer_start(handle) != ESP_OK) {
            gptimer_disable(handle);
            gptimer_del_timer(handle);
            stepTimerHandle_ = nullptr;
            return false;
        }

        // Ramp service is now driven by the host (LocalMotor) via
        // serviceRamp(nowMs) — no internal esp_timer task. Single owner
        // for ramp + alarm reconfig avoids cross-task races and uneven
        // cadence under main-loop ADC bursts.

        return true;
    }

    void StepGenerator::end() {
        // Stop step timer
        if (stepTimerHandle_ != nullptr) {
            gptimer_handle_t handle = static_cast<gptimer_handle_t>(stepTimerHandle_);
            gptimer_stop(handle);
            gptimer_disable(handle);
            gptimer_del_timer(handle);
            stepTimerHandle_ = nullptr;
        }
    }

    // ---- Speed and ramp ----

    void StepGenerator::setSpeed(int32_t targetSps, uint32_t accelMs, uint32_t decelMs) {
        // Compute rates before entering critical section to minimize lock time
        float newAccelRate = accelRate_;
        float newDecelRate = decelRate_;

        if (targetSps > 0) {
            float speedFloat = static_cast<float>(targetSps);
            newAccelRate =
                    (accelMs > 0) ? speedFloat / (static_cast<float>(accelMs) / 1000.0F) : 0.0F;
            newDecelRate =
                    (decelMs > 0) ? speedFloat / (static_cast<float>(decelMs) / 1000.0F) : 0.0F;
        }

        // Atomic group write — ramp timer and ISR see consistent speed + rates
        portENTER_CRITICAL(&g_stepMux);
        targetSps_ = targetSps;
        if (targetSps > 0) {
            accelRate_ = newAccelRate;
            decelRate_ = newDecelRate;
        }
        portEXIT_CRITICAL(&g_stepMux);
    }

    void StepGenerator::start() {
        portENTER_CRITICAL(&g_stepMux);
        autoStopped_ = false;
        currentSps_ = 0.0F;
        running_ = true;
        portEXIT_CRITICAL(&g_stepMux);
    }

    void StepGenerator::stop() {
        portENTER_CRITICAL(&g_stepMux);
        targetSps_ = 0;
        portEXIT_CRITICAL(&g_stepMux);
        // Ramp timer brings currentSps_ to zero, then auto-stops
    }

    void StepGenerator::hardStop() {
        portENTER_CRITICAL(&g_stepMux);
        running_ = false;
        currentSps_ = 0.0F;
        targetSps_ = 0;
        portEXIT_CRITICAL(&g_stepMux);
        applyAlarmTicks(step::IDLE_ALARM_TICKS);
    }

    bool StepGenerator::consumeAutoStop() {
        portENTER_CRITICAL(&g_stepMux);
        bool was = autoStopped_;
        autoStopped_ = false;
        portEXIT_CRITICAL(&g_stepMux);
        return was;
    }

    void StepGenerator::resetPosition() {
        portENTER_CRITICAL(&g_stepMux);
        positionSteps_ = 0;
        portEXIT_CRITICAL(&g_stepMux);
    }

    void StepGenerator::setTargetPosition(int32_t target) {
        portENTER_CRITICAL(&g_stepMux);
        targetPosition_ = target;
        targetReached_ = false;
        hasTarget_ = true;
        portEXIT_CRITICAL(&g_stepMux);
    }

    void StepGenerator::clearTargetPosition() {
        portENTER_CRITICAL(&g_stepMux);
        hasTarget_ = false;
        portEXIT_CRITICAL(&g_stepMux);
    }

    bool StepGenerator::consumeTargetReached() {
        portENTER_CRITICAL(&g_stepMux);
        bool was = targetReached_;
        targetReached_ = false;
        portEXIT_CRITICAL(&g_stepMux);
        return was;
    }

    // ---- Ramp computation (called from esp_timer task, not ISR) ----

    float StepGenerator::updateRamp(uint32_t deltaMs, int32_t snapTarget, float snapAccel,
                                    float snapDecel, float snapCurrent) const {
        const float targetFloat = static_cast<float>(snapTarget);
        const float diff = targetFloat - snapCurrent;

        const float rate = (diff > 0.0F) ? snapAccel : snapDecel;

        float newSps;
        if (rate <= 0.0F) {
            newSps = targetFloat;
        } else {
            const float maxChange = rate * (static_cast<float>(deltaMs) / 1000.0F);
            if (diff > maxChange) {
                newSps = snapCurrent + maxChange;
            } else if (diff < -maxChange) {
                newSps = snapCurrent - maxChange;
            } else {
                newSps = targetFloat;
            }
        }

        return (newSps < 0.0F) ? 0.0F : newSps;
    }

    uint32_t StepGenerator::computeAlarmTicks(float sps) {
        if (sps < step::MIN_RUNNING_SPS) {
            return step::IDLE_ALARM_TICKS;
        }
        uint32_t ticks = static_cast<uint32_t>(
                static_cast<float>(step::TIMER_FREQ_HZ) / (sps * step::TOGGLES_PER_STEP));
        return (ticks < step::MIN_ALARM_TICKS) ? step::MIN_ALARM_TICKS : ticks;
    }

    void StepGenerator::applyAlarmTicks(uint32_t ticks) {
        lastAppliedTicks_ = ticks;
        if (stepTimerHandle_ == nullptr) {
            return;
        }
        gptimer_handle_t handle = static_cast<gptimer_handle_t>(stepTimerHandle_);
        gptimer_alarm_config_t alarmConfig = {};
        alarmConfig.alarm_count = ticks;
        alarmConfig.reload_count = 0;
        alarmConfig.flags.auto_reload_on_alarm = true;
        gptimer_set_alarm_action(handle, &alarmConfig);
    }

}  // namespace motor
