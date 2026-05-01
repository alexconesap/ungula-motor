// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <climits>
#include <cstdint>
#include "motor_types.h"

/// @brief Step pulse generator + acceleration ramp service.
///
/// Manages an ESP-IDF gptimer for step pulses (ISR toggles the pin and
/// counts position). The acceleration ramp is *not* run on its own timer —
/// the host calls serviceRamp(nowMs) periodically (e.g. from LocalMotor's
/// 10 ms service tick). One task owns ramp + alarm reconfig, eliminating
/// cross-task races against gptimer_set_alarm_action() and uneven update
/// cadence under main-loop ADC bursts (PID, fan tach, etc.).
///
/// ## Concurrency model
///
/// Two execution contexts share mutable state:
///   1. Step ISR (gptimer, hardware interrupt) — toggles pin, counts position
///   2. Caller — serviceRamp(), setSpeed(), start/stop()
///
/// All shared state is protected by a portMUX spinlock
/// (portENTER_CRITICAL / portENTER_CRITICAL_ISR). volatile is used only
/// to prevent compiler caching; it is not the synchronisation mechanism.

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

            /// @brief Acceleration ramp service. Call from a periodic context
            /// (LocalMotor calls this from its 10 ms service tick). Computes
            /// the next currentSps_ from the configured target/accel/decel
            /// using the actual elapsed time since the previous call, then
            /// reapplies the gptimer alarm period if the ticks changed.
            ///
            /// Robust against irregular call cadence — the actual dt is
            /// measured per call, so a delayed-then-catch-up pattern
            /// still produces correct cumulative speed (won't overshoot
            /// like a fixed-dt scheme would).
            void serviceRamp(uint32_t nowMs);

        private:
            // Hardware timer for step pulses (opaque, cast to gptimer_handle_t)
            void* stepTimerHandle_ = nullptr;

            uint8_t stepPin_ = GPIO_NONE;
            uint32_t lastRampMs_ = 0;  // tracks dt across serviceRamp() calls

            // ISR-shared state (volatile)
            volatile bool stepPinState_ = false;
            volatile int32_t positionSteps_ = 0;
            volatile bool directionFwd_ = true;
            volatile bool running_ = false;
            volatile bool autoStopped_ = false;
            volatile bool targetReached_ = false;
            volatile int32_t targetPosition_ = INT32_MAX;
            volatile bool hasTarget_ = false;
            // Set by ISR when a target-reached stop happens. Consumed by
            // serviceRamp() to zero currentSps_/targetSps_ outside ISR context
            // (avoids float writes in the ISR).
            volatile bool stopRequestedFromIsr_ = false;

            // Ramp state — written by ramp timer, read by ramp timer + getters.
            // targetSps_ and rates also written by caller (setSpeed/stop).
            volatile int32_t targetSps_ = 0;
            volatile float accelRate_ = 0.0F;  // SPS per second (speeding up)
            volatile float decelRate_ = 0.0F;  // SPS per second (slowing down), 0 = instant
            volatile float currentSps_ = 0.0F;
            uint32_t lastAppliedTicks_ = step::IDLE_ALARM_TICKS;

            // Returns the computed new speed — does NOT write currentSps_.
            // Caller writes currentSps_ under lock after this returns.
            float updateRamp(uint32_t deltaMs, int32_t snapTarget, float snapAccel, float snapDecel,
                             float snapCurrent) const;
            static uint32_t computeAlarmTicks(float sps);
            void applyAlarmTicks(uint32_t ticks);
    };

}  // namespace motor
