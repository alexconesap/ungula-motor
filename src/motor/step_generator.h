// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <climits>
#include <cstdint>
#include "motor_types.h"

/// @brief Autonomous step pulse generator with internal ramp timer.
///
/// Manages an ESP-IDF gptimer for step pulses (ISR toggles the pin and
/// counts position) plus an esp_timer for the acceleration ramp (~10 ms).
///
/// No external service() call is needed — once started, the motor runs
/// autonomously until it reaches the target speed (or zero on stop).
///
/// ## Concurrency model
///
/// Three execution contexts share mutable state:
///   1. Step ISR (gptimer, hardware interrupt) — toggles pin, counts position
///   2. Ramp timer (esp_timer task) — computes acceleration ramp, updates speed
///   3. Caller (any task/context) — configures speed, starts/stops motion
///
/// All multi-field writes and reads are protected by a portMUX spinlock
/// (portENTER_CRITICAL / portENTER_CRITICAL_ISR). Single 32-bit aligned
/// reads (getters) rely on hardware atomicity and volatile ordering.

namespace motor {

    class StepGenerator {
        public:
            /// @brief Initialize step pin GPIO, hardware timer, and ramp timer.
            /// @param stepPin ESP32 GPIO number for step pulse output.
            /// @return true on success, false if timer creation failed.
            bool begin(uint8_t stepPin);

            /// @brief Stop and release all timers.
            void end();

            /// @brief Configure target speed and acceleration/deceleration ramps.
            /// @param targetSps Absolute speed in steps per second.
            /// @param accelMs Ramp duration from 0 to targetSps (0 = instant).
            /// @param decelMs Ramp duration from targetSps to 0 (0 = instant stop).
            void setSpeed(int32_t targetSps, uint32_t accelMs, uint32_t decelMs);

            /// @brief Start step generation. ISR will begin toggling.
            void start();

            /// @brief Request graceful stop (ramp speed to zero).
            void stop();

            /// @brief Immediate stop — no deceleration.
            void hardStop();

            /// @brief Reset position counter to zero (after homing).
            void resetPosition();

            /// @brief Set a target position for the ISR to stop at.
            /// When the position reaches the target, the ISR stops immediately
            /// (single-step precision, zero overshoot). Pass INT32_MAX to clear.
            void setTargetPosition(int32_t target);

            /// @brief Clear target position (disable ISR-level position check).
            void clearTargetPosition();

            /// @brief True if the ISR stopped because it reached the target position.
            /// Reading this flag clears it (consume-once semantics).
            bool consumeTargetReached();

            // ---- Getters (read volatile ISR state — individually atomic on ESP32) ----

            /// @brief Current absolute position in steps.
            int32_t position() const {
                return positionSteps_;
            }

            /// @brief Current speed in steps per second.
            float currentSpeed() const {
                return currentSps_;
            }

            /// @brief True while the step timer is firing step pulses.
            ///
            /// This is the low-level "pulse train is active" flag — not the
            /// high-level "motor is in a motion FSM state" answer. Use
            /// IMotor::isMoving() for the latter. Renamed from isRunning()
            /// (too easy to confuse with HomingRunner::isRunning()).
            bool isPulsing() const {
                return running_;
            }

            /// @brief Current acceleration rate in steps/s per second.
            float accelerationRate() const {
                return accelRate_;
            }

            /// @brief Current deceleration rate in steps/s per second.
            float decelerationRate() const {
                return decelRate_;
            }

            /// @brief True if the stepper auto-stopped (ramp reached zero).
            /// Reading this flag clears it (consume-once semantics).
            bool consumeAutoStop();

            /// @brief Set direction for ISR position counting.
            /// @param forward true = forward (position increments).
            void setDirectionForward(bool forward) {
                directionFwd_ = forward;
            }

            /// @brief Timer ISR handler — called from internal alarm callback.
            /// Do not call from application code.
            void handleStepIsr();

            /// @brief Ramp timer handler — called from internal esp_timer.
            /// Do not call from application code.
            void handleRampTimer();

        private:
            // Hardware timer for step pulses (opaque, cast to gptimer_handle_t)
            void* stepTimerHandle_ = nullptr;

            // Software timer for ramp updates (opaque, cast to esp_timer_handle_t)
            void* rampTimerHandle_ = nullptr;

            uint8_t stepPin_ = GPIO_NONE;

            // ISR-shared state (volatile)
            volatile bool stepPinState_ = false;
            volatile int32_t positionSteps_ = 0;
            volatile bool directionFwd_ = true;
            volatile bool running_ = false;
            volatile bool autoStopped_ = false;
            volatile bool targetReached_ = false;
            volatile int32_t targetPosition_ = INT32_MAX;
            volatile bool hasTarget_ = false;

            // Ramp state — written by ramp timer, read by ramp timer + getters.
            // targetSps_ and rates also written by caller (setSpeed/stop).
            volatile int32_t targetSps_ = 0;
            volatile float accelRate_ = 0.0F;  // SPS per second (speeding up)
            volatile float decelRate_ = 0.0F;  // SPS per second (slowing down), 0 = instant
            volatile float currentSps_ = 0.0F;
            uint32_t lastAppliedTicks_ = step::IDLE_ALARM_TICKS;

            void updateRamp(uint32_t deltaMs, int32_t snapTarget, float snapAccel, float snapDecel,
                            float snapCurrent);
            uint32_t computeAlarmTicks() const;
            void applyAlarmTicks(uint32_t ticks);
    };

}  // namespace motor
