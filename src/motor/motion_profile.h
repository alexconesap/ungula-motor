// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

/// @brief Complete motion description for autonomous execution.
///
/// Contains enough data for a motor to execute the full motion without
/// external intervention. The motor waits until startTimeMs, then
/// executes the velocity profile autonomously.

namespace motor {

    /// @brief Velocity profile shape.
    enum class ProfileShape : uint8_t {
        Trapezoidal = 0,  /// Accel → cruise → decel.
        Triangular        /// Accel → decel (never reaches maxVelocity).
    };

    /// @brief Full motion specification for autonomous execution.
    ///
    /// Currently only startTimeMs, targetPosition, and maxVelocitySps are
    /// implemented by LocalMotor. The remaining fields are reserved for
    /// future trapezoidal/triangular profile execution.
    struct MotionProfileSpec {
            uint32_t startTimeMs;   /// Absolute start time (from syncNow()). 0 = start immediately.
            int32_t startPosition;  /// Expected position at start (for validation). TODO: not yet
                                    /// used.
            int32_t targetPosition;     /// Absolute target position in steps.
            int32_t startVelocitySps;   /// Initial velocity (steps/s). Usually 0. TODO: not yet
                                        /// used.
            int32_t maxVelocitySps;     /// Cruise velocity (steps/s).
            int32_t endVelocitySps;     /// Final velocity (steps/s). Usually 0. TODO: not yet used.
            int32_t accelerationSpsPs;  /// Acceleration in steps/s per second. TODO: not yet used.
            int32_t decelerationSpsPs;  /// Deceleration in steps/s per second. TODO: not yet used.
            ProfileShape shape;         /// Velocity curve shape. TODO: not yet used.
    };

}  // namespace motor
