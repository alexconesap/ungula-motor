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
        if (!running_) {
            stepPinState_ = false;
            gpio_set_level(static_cast<gpio_num_t>(stepPin_), 0);
            return;
        }

        stepPinState_ = !stepPinState_;
        gpio_set_level(static_cast<gpio_num_t>(stepPin_), stepPinState_ ? 1 : 0);

        // Count on rising edge (two toggles per full step)
        if (stepPinState_) {
            if (directionFwd_) {
                positionSteps_ = positionSteps_ + 1;
            } else {
                positionSteps_ = positionSteps_ - 1;
            }

            // ISR-level target check — single-step precision, zero overshoot.
            // Stops immediately when position reaches (or passes) the target.
            // Critical section protects multi-field write (running, hasTarget, targetReached)
            // and ensures consistent read of targetPosition_ set by caller.
            portENTER_CRITICAL_ISR(&g_stepMux);
            if (hasTarget_) {
                bool reached = directionFwd_ ? (positionSteps_ >= targetPosition_)
                                             : (positionSteps_ <= targetPosition_);
                if (reached) {
                    running_ = false;
                    currentSps_ = 0.0F;
                    targetSps_ = 0;
                    hasTarget_ = false;
                    targetReached_ = true;
                }
            }
            portEXIT_CRITICAL_ISR(&g_stepMux);
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
        if (!running_) {
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

        // Snapshot shared state under lock, then compute outside critical section
        portENTER_CRITICAL(&g_stepMux);
        int32_t snapTarget = targetSps_;
        float snapAccel = accelRate_;
        float snapDecel = decelRate_;
        float snapCurrent = currentSps_;
        portEXIT_CRITICAL(&g_stepMux);

        updateRamp(dtMs, snapTarget, snapAccel, snapDecel, snapCurrent);

        uint32_t newTicks = computeAlarmTicks();
        if (newTicks != lastAppliedTicks_) {
            applyAlarmTicks(newTicks);
        }

        // Auto-stop when ramp reaches zero
        if (snapTarget == 0 && currentSps_ < step::MIN_RUNNING_SPS) {
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
        timerConfig.flags.intr_shared = true;

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

    void StepGenerator::updateRamp(uint32_t deltaMs, int32_t snapTarget, float snapAccel,
                                   float snapDecel, float snapCurrent) {
        float targetFloat = static_cast<float>(snapTarget);
        float diff = targetFloat - snapCurrent;

        // Pick the right rate: accelerating (speeding up) or decelerating (slowing down)
        bool speeding = (diff > 0.0F);
        float rate = speeding ? snapAccel : snapDecel;

        float newSps = snapCurrent;

        // Rate of 0 means instant change (no ramp)
        if (rate <= 0.0F) {
            newSps = targetFloat;
        } else {
            float maxChange = rate * (static_cast<float>(deltaMs) / 1000.0F);

            if (diff > maxChange) {
                newSps = snapCurrent + maxChange;
            } else if (diff < -maxChange) {
                newSps = snapCurrent - maxChange;
            } else {
                newSps = targetFloat;
            }
        }

        if (newSps < 0.0F) {
            newSps = 0.0F;
        }

        // Single atomic write back
        currentSps_ = newSps;
    }

    uint32_t StepGenerator::computeAlarmTicks() const {
        if (currentSps_ < step::MIN_RUNNING_SPS) {
            return step::IDLE_ALARM_TICKS;
        }

        float togglesPerSec = currentSps_ * step::TOGGLES_PER_STEP;
        uint32_t ticks =
                static_cast<uint32_t>(static_cast<float>(step::TIMER_FREQ_HZ) / togglesPerSec);

        if (ticks < step::MIN_ALARM_TICKS) {
            ticks = step::MIN_ALARM_TICKS;
        }
        return ticks;
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
