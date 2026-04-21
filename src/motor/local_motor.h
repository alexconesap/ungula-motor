// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "homing/i_homeable_motor.h"
#include "i_motor.h"
#include "i_motor_driver.h"
#include "limit_switch.h"
#include "motion_profile.h"
#include "motor_event_publisher.h"
#include "motor_fsm.h"
#include "motor_types.h"
#include "step_generator.h"

/// @brief Autonomous motor controller backed by local hardware.
///
/// Composes IMotorDriver + StepGenerator + MotorFsm +
/// MotorEventPublisher + LimitSwitch into a single self-contained facade.
///
/// ## Autonomous operation — no service() needed
///
/// After calling begin() and enable(), the motor is fully autonomous.
/// Motion commands (moveBy, moveForward, stop, etc.) are non-blocking
/// and return immediately. The motor handles everything internally:
///
///   - Step pulses: hardware timer ISR (GPTimer, zero jitter)
///   - Acceleration ramp: computed inside the step ISR every ~10 ms
///   - Safety monitoring: internal periodic timer (esp_timer, 10 ms)
///     checks stall detection, limit switches, target position,
///     and manages FSM transitions
///
/// The caller is completely free between commands. No polling, no
/// service() calls, no timing obligations. The main loop can handle
/// HTTP, MQTT, display, or any other work without affecting motor motion.
///
/// ## Usage
///
///   // Create and wire (once at boot)
///   LocalMotor motor;
///   motor.setDriver(driver);    // driver owns STEP, EN, DIR, DIAG pins
///   motor.setStepsPerMm(400.0F);
///   motor.setAutoStopOnStall(true);
///   motor.begin();
///
///   // Enable and move
///   motor.enable();
///   motor.setProfileSpeed(MotionProfile::CYCLE, 1000);
///   motor.setProfileRamp(MotionProfile::CYCLE, 800);
///   motor.moveBy(3000);       // Non-blocking, motor runs autonomously
///   motor.moveBy(5.0F, DistanceUnit::CM);  // Move 5 cm
///
///   // Check state whenever convenient (no timing obligation)
///   if (motor.state() == MotorFsmState::TargetReached) { ... }
///   int32_t pos = motor.positionSteps();  // Safe anytime, reads ISR counter
///
///   // Stop commands (can interrupt any motion)
///   motor.stop();          // Soft: decelerates using ramp
///   motor.emergencyStop(); // Hard: instant stop
///
/// ## Threading model
///
/// - **Step pulses + ramp**: GPTimer ISR (hardware timer, runs on the
///   core that called begin()). Zero jitter, independent of CPU load.
/// - **Safety monitoring**: esp_timer callback (10 ms periodic). Handles
///   stall detection, limit switches, target deceleration, FSM.
/// - **Motion commands**: called from any context (main loop, task, etc.).
///   Thread-safe via critical sections around shared mutable state.
///
/// ## Stall detection
///
/// Stall detection is fully owned by the motor driver (IMotorDriver).
/// The driver handles pin reading, register polling, speed-based
/// thresholds, and blanking. LocalMotor only asks the driver "are you
/// stalling?" and decides what to do about it (stop, FSM transition,
/// notify listeners). This separation means changing the driver chip
/// (e.g. TMC2209 → DRV8825) doesn't require changes to LocalMotor.
///
namespace motor {

    class LocalMotor : public IHomeableMotor {
        public:
            LocalMotor() = default;

            // ---- Wiring (call before begin) ----

            /// @brief Attach motor driver. Required before begin().
            void setDriver(IMotorDriver& driver);

            /// @brief Add backward (home-side) limit switch.
            void addLimitBackward(uint8_t pin);

            /// @brief Add forward (end-side) limit switch.
            void addLimitForward(uint8_t pin);

            /// @brief Subscribe to motor events.
            bool subscribe(IMotorEventListener* listener);

            /// @brief Unsubscribe from motor events.
            bool unsubscribe(IMotorEventListener* listener);

            // ---- Configuration (call before or after begin) ----

            /// @brief When true, motor auto-stops on stall. Callback fires regardless.
            void setAutoStopOnStall(bool enabled) override;

            /// @brief Set microstep resolution. Updates the driver.
            void setMicrosteps(uint16_t microsteps);

            /// @brief Set motor run current in milliamps. Passed to the driver.
            void setRunCurrent(uint16_t milliAmps);

            /// @brief Install a speed→current curve. Takes effect only when the
            /// feature is enabled via setCurrentCurveEnabled(true). Does not
            /// change the driver current until the next motion command.
            void setCurrentCurve(const CurrentCurve& curve);

            /// @brief Enable/disable automatic run-current adjustment on every
            /// speed change. Default off — existing projects behave exactly as
            /// before. When enabled, the motor computes mA from the curve and
            /// calls driver.setRunCurrent() at each startMotion / updateSpeed.
            void setCurrentCurveEnabled(bool enabled);

            /// @brief Set conversion factor for mm-based units.
            void setStepsPerMm(float stepsPerMm);

            /// @brief Set conversion factor for degree-based units.
            void setStepsPerDegree(float stepsPerDeg);

            /// @brief Set speed for a named profile (raw SPS).
            void setProfileSpeed(MotionProfile profile, int32_t speedSps) override;

            /// @brief Set speed for a named profile using real-world units.
            void setProfileSpeed(MotionProfile profile, SpeedValue speed);

            /// @brief Set acceleration ramp time for a named profile (ms).
            /// 0 = instant acceleration (no ramp up).
            void setProfileAccel(MotionProfile profile, uint32_t accelMs) override;

            /// @brief Set deceleration ramp time for a named profile (ms).
            /// 0 = instant stop when stop() is called.
            void setProfileDecel(MotionProfile profile, uint32_t decelMs) override;

            /// @brief Select which profile is used for continuous motion commands.
            void setActiveProfile(MotionProfile profile) override;

            // ---- Lifecycle ----

            /// @brief Initialize driver, timer, internal service timer.
            /// After this, the motor is fully autonomous — no external polling needed.
            /// @return true on success, false if timer setup failed.
            bool begin();

            /// @brief Stop all timers and release resources. Safe to call if begin() was never
            /// called.
            void end();

            // ---- IMotor interface ----

            void enable() override;
            void disable() override;

            /// All motion commands are non-blocking. They configure the motion
            /// and return immediately. The motor runs autonomously via the
            /// hardware timer ISR (step pulses + ramp) and the internal service
            /// timer (safety + FSM). The main loop is completely free.
            void moveForward() override;
            void moveBackward() override;
            void moveTo(float target, DistanceUnit unit = DistanceUnit::STEPS) override;
            void moveBy(float delta, DistanceUnit unit = DistanceUnit::STEPS) override;
            void executeProfile(const MotionProfileSpec& profile) override;

            /// @brief Update speed and ramp while the motor is running.
            /// Ramps smoothly to the new target — no stop/restart needed.
            /// Does nothing if the motor is not currently moving.
            void updateSpeed(int32_t speedSps, uint32_t accelMs, uint32_t decelMs);

            /// @brief Update speed using real-world units while the motor is running.
            void updateSpeed(SpeedValue speed, uint32_t accelMs, uint32_t decelMs);

            /// @brief Soft stop: decelerates to zero using the configured ramp time.
            void stop() override;

            /// @brief Hard stop: instant stop with no deceleration.
            void emergencyStop() override;

            MotorFsmState state() const override;
            int32_t positionSteps() const override;
            bool isMoving() const override;

            // ---- Fault acknowledgement ----

            /// @brief Acknowledge stall — returns to Idle if in Stall state.
            void clearStall() override;

            /// @brief Acknowledge fault — returns to Idle if in Fault state.
            void clearFault() override;

            // ---- Position ----

            /// @brief Reset the step counter to zero (homing reference point).
            /// Only allowed when the motor is not moving.
            void resetPosition() override;

            // ---- Diagnostics ----

            /// @brief Current speed in steps per second (from step generator).
            float currentSpeed() const;

            /// @brief Current direction.
            Direction direction() const {
                return direction_;
            }

            // ---- Internal (do not call from application code) ----

            /// @brief Internal service handler — called by the esp_timer callback.
            void handleServiceTimer();

        private:
            // Step generator owns the timer
            StepGenerator stepper_;

            // Injected motor driver (TMC2209, DRV8825, etc.)
            IMotorDriver* driver_ = nullptr;

            // Internal service timer (opaque, cast to esp_timer_handle_t in .cpp)
            void* serviceTimer_ = nullptr;

            // Stall policy (detection is in the driver, policy is here)
            bool autoStopOnStall_ = false;

            // Speed→current curve. Off by default — opt-in via setCurrentCurveEnabled.
            CurrentCurve currentCurve_ = {};
            bool currentCurveEnabled_ = false;

            // Limit switches
            LimitSwitch limitsBackward_[limit::MAX_PER_DIRECTION];
            LimitSwitch limitsForward_[limit::MAX_PER_DIRECTION];
            int32_t backwardLimitCount_ = 0;
            int32_t forwardLimitCount_ = 0;

            // FSM and events
            MotorFsm fsm_;
            MotorEventPublisher<MAX_MOTOR_EVENT_LISTENERS> eventPublisher_;

            // Position snapshot — updated every service tick for FSM event stamping.
            // The FSM reads this through a pointer, so it must outlive the FSM.
            int32_t cachedPosition_ = 0;

            // Motion state — protected by g_motorMux critical sections.
            // Accessed from both caller context and service timer.
            Direction direction_ = Direction::FORWARD;
            int32_t moveTarget_ = 0;
            bool hasTarget_ = false;
            bool decelerating_ = false;

            // Pending profile execution — protected by g_motorMux.
            MotionProfileSpec pendingProfile_ = {};
            bool hasPendingProfile_ = false;

            // Motion profiles (indexed by MotionProfile enum)
            ProfileConfig profiles_[PROFILE_COUNT] = {};
            MotionProfile activeProfile_ = MotionProfile::CYCLE;

            // Unit conversion
            float stepsPerMm_ = 0.0F;
            float stepsPerDeg_ = 0.0F;

            // Helpers
            void moveToAbsolute(int32_t absoluteTarget);
            void applyDirection();
            void applyCurrentForSpeed(int32_t speedSps);
            bool isBackwardLimitHit() const;
            bool isForwardLimitHit() const;
            bool isLimitHitInDirection() const;
            void handleStall();
            void serviceMoving();
            void servicePendingProfile(uint32_t nowMs);
            void startMotion(int32_t speedSps, uint32_t accelMs, uint32_t decelMs);
            int32_t convertToSps(SpeedValue speed) const;
            int32_t convertToSteps(float value, DistanceUnit unit) const;

            // Service timer trampoline
            static void onServiceTimer(void* arg);
    };

}  // namespace motor
