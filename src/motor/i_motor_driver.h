// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "motor_types.h"

/// @brief Generic motor driver interface — the complete hardware layer.
///
/// Owns all GPIO pins that connect to the driver chip (STEP, EN, DIR, DIAG)
/// and any register-level communication (UART, SPI, etc.). Concrete
/// drivers (TMC2209, DRV8825, A4988, etc.) implement this.
///
/// The motor controller (LocalMotor) never touches GPIO directly —
/// it delegates all hardware access through this interface.
///
/// ## Stall detection
///
/// The driver owns the entire stall detection mechanism: pin reading,
/// register polling, scoring, blanking. Different drivers detect stalls
/// differently (DIAG pin, SG register, back-EMF sensing, etc.) — the
/// motor controller doesn't know or care about the mechanism.
///
/// The motor controller calls:
///   - prepareMotion() when motion starts (driver configures for speed)
///   - serviceStallDetection() every ~10 ms (driver does its work)
///   - isStalling() to check the driver's verdict
///   - clearStall() to reset after handling
///
/// Policy (what to DO about a stall) stays in the motor controller.
/// Detection (whether a stall IS happening) stays in the driver.
///
namespace motor {

    class IMotorDriver {
        public:
            virtual ~IMotorDriver() = default;

            /// @brief Module identifier for logging (e.g. "motor_driver").
            virtual const char* module() const = 0;

            /// @brief Human-readable description (e.g. "TMC2209 v0x21 @ UART2 addr=0").
            virtual const char* info() const = 0;

            /// @brief Initialize driver hardware: GPIO pins, UART, register defaults.
            virtual void begin() = 0;

            // ---- GPIO control ----

            /// @brief Enable the driver output stage (motor energised).
            virtual void enable() = 0;

            /// @brief Disable the driver output stage (motor free-spinning).
            virtual void disable() = 0;

            /// @brief Set motor rotation direction via the DIR pin.
            virtual void setDirection(Direction dir) = 0;

            /// @brief GPIO pin used for step pulses. The motor controller's
            /// StepGenerator reads this to configure its hardware timer.
            virtual uint8_t stepPin() const = 0;

            /// @brief Invert motor direction. When true, FORWARD commands
            /// produce backward rotation and vice-versa. Use this to
            /// compensate for reversed coil wiring without changing cables.
            virtual void setDirectionInverted(bool inverted) = 0;

            // ---- Configuration ----

            /// @brief Set microstep resolution.
            virtual void setMicrosteps(uint16_t microsteps) = 0;

            /// @brief Set run current in milliamps.
            virtual void setRunCurrent(uint16_t milliAmps) = 0;

            // ---- Stall detection lifecycle ----

            /// @brief Prepare stall detection for upcoming motion.
            /// Called when the motor starts moving. The driver resets internal
            /// counters, configures speed-dependent thresholds, and sets a
            /// blanking period to suppress false triggers during acceleration.
            /// @param speedSps Target speed in steps per second.
            /// @param accelMs Acceleration ramp duration in milliseconds.
            virtual void prepareStallDetection(int32_t speedSps, uint32_t accelMs) {
                (void)speedSps;
                (void)accelMs;
            }

            /// @brief Update stall thresholds for a new speed (mid-motion change).
            virtual void updateStallDetectionSpeed(int32_t speedSps) {
                (void)speedSps;
            }

            /// @brief Periodic stall detection service — call every ~10 ms.
            /// The driver reads pins, polls registers, updates scoring.
            virtual void serviceStallDetection() {}

            /// @brief Driver's verdict: is a stall happening right now?
            /// Drivers without stall detection must return false.
            virtual bool isStalling() const = 0;

            /// @brief Reset stall detection state after the motor controller
            /// has handled the stall (stopped motor, notified listeners, etc.).
            /// Drivers without stall detection implement as empty.
            virtual void clearStall() = 0;

            // ---- Status (optional, not all drivers support these) ----

            /// @brief Returns last driver status register. Returns 0 if not supported.
            virtual uint32_t lastDrvStatus() {
                return 0;
            }

            /// @brief Read driver version/ID. Returns 0 if not supported.
            virtual uint8_t version() {
                return 0;
            }
    };

}  // namespace motor
