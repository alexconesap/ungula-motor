// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

namespace motor {

    class IHomeableMotor;

    /// @brief Policy object that tells a HomingRunner how to find position zero.
    ///
    /// Homing logic varies per machine: hard stop with stall detection, limit
    /// switch, optical sensor, encoder index. The strategy isolates that logic
    /// from the motor itself so the same IHomeableMotor can be homed different ways
    /// in different projects.
    ///
    /// Lifecycle, called by HomingRunner:
    ///   1. begin(motor)  — set up profiles, enable sensors, start first move.
    ///   2. tick(motor)   — poll on each loop iteration. Returns true when done
    ///                      (either homed or gave up).
    ///   3. finish(motor, succeeded) — cleanup: stop motor if running, reset
    ///                                 position on success.
    ///
    /// The strategy only talks to the motor through IHomeableMotor. No direct GPIO
    /// access, no ISR coupling. Keeps strategies portable across drivers.
    class IHomingStrategy {
        public:
            virtual ~IHomingStrategy() = default;

            /// @brief Prepare sensors and command the first motion.
            virtual void begin(IHomeableMotor& motor) = 0;

            /// @brief Advance the internal phase. Returns true when homing has
            /// finished (check succeeded() to know the outcome).
            virtual bool tick(IHomeableMotor& motor) = 0;

            /// @brief Terminal cleanup. Called once after tick() returns true or
            /// when the runner aborts/timeouts.
            virtual void finish(IHomeableMotor& motor, bool succeeded) = 0;

            /// @brief Outcome of the last run. Valid after tick() returns true or
            /// finish() has been called.
            virtual bool succeeded() const = 0;
    };

}  // namespace motor
