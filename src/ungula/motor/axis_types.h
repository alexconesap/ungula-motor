// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// Axis identifier. Used in event payloads to disambiguate when a single
/// listener subscribes to multiple axes. 0xFF is the unassigned sentinel.
struct AxisId {
        uint8_t value = 0xFF;
        constexpr explicit AxisId(uint8_t v = 0xFF)
                : value(v)
        {
        }
};

/// User-facing unit for moveTo / moveBy calls. `Steps` is always exact;
/// the others require a configured conversion factor before use.
enum class DistanceUnit : uint8_t {
        Steps = 0,
        Mm = 1,
        Cm = 2,
        Degrees = 3,
};

/// Direction of commanded motion.
enum class Direction : uint8_t {
        Forward = 0,
        Backward = 1,
};

inline Direction reverse(Direction d)
{
        return d == Direction::Forward ? Direction::Backward : Direction::Forward;
}

/// Manner in which a stop is requested.
///   - Decelerate: ramp to zero using the configured decel profile.
///   - Immediate:  best-effort fast stop, may overshoot mechanically.
///   - Emergency:  hardware-fast stop; latches a fault.
enum class StopMode : uint8_t {
        Decelerate = 0,
        Immediate = 1,
        Emergency = 2,
};

/// Generic position value in user units (integer steps). Wrapping is a
/// runtime concern of the axis, not the type.
using Position = int32_t;

/// Signed relative distance in steps.
using Distance = int32_t;

/// Velocity in steps per second. Negative is reverse for planner inputs;
/// commands take a magnitude + Direction instead.
using Velocity = int32_t;

/// Microseconds, used by the planner / pulse engine. Matches lib's
/// `ungula::core::time::duration_us_t` semantics (signed 64-bit so it
/// composes with the time API without truncation).
using Micros = int64_t;

/// Trajectory limits supplied to the planner. Acceleration & deceleration
/// are in steps/s² (always positive).
struct TrajectoryLimits {
        Velocity maxVelocitySps = 0;
        uint32_t accelSpsPerSec = 0;
        uint32_t decelSpsPerSec = 0;
        /// Minimum STEP high time, in microseconds. Driver-dependent (TMC2209
        /// can tolerate ~50 ns; many CNC opto-isolated drivers want ≥2.5 µs).
        /// Set conservatively per drive.
        uint32_t minPulseHighUs = 2;
        /// Minimum STEP low time, in microseconds. Symmetric default.
        uint32_t minPulseLowUs = 2;
        /// Hard ceiling on commanded step rate, in SPS. Bounds the planner
        /// even if maxVelocitySps is set higher.
        uint32_t maxStepRateSps = 200'000;
};

/// Unit conversion factors. Set the ones the axis will actually use.
/// Zero means "not configured" — any move in that unit will be rejected
/// with `ErrorCode::InvalidConfig`.
struct UnitScaling {
        float stepsPerMm = 0.0f;
        float stepsPerDegree = 0.0f;
};

/// Black-box motion / position snapshot returned by Axis::feedback().
/// Fields not provided by the actuator are set to their "unavailable"
/// sentinels (commandedPosition is always valid; actualPosition uses
/// `hasActualPosition` to gate).
struct AxisFeedback {
        Position commandedPosition = 0;
        Position actualPosition = 0;
        bool hasActualPosition = false;
        Velocity currentSps = 0;
        Direction direction = Direction::Forward;
        bool inPosition = true;
        bool alarmActive = false;
        Distance followingErrorSteps = 0;
        bool hasFollowingError = false;
};

} // namespace ungula::motor
