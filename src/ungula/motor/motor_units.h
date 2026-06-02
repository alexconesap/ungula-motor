// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

// Arduino.h (pulled in by arduino-cli on every build, even when the
// sketch doesn't use Arduino APIs) defines `degrees(rad)` as a
// function-like macro that mangles every textual occurrence of
// `degrees(...)` - including member-function declarations inside
// namespaces. The lib needs its own `DistanceValue::degrees(float)`
// factory, so undef the macro here before declaring it. Hosts that
// actually use Arduino's degrees/radians math helpers must
// `#include <Arduino.h>` after this header (or compute the
// conversion directly - it's `value * 180.0f / PI`).
#ifdef degrees
#undef degrees
#endif

namespace ungula::motor
{

// =====================================================================
// Foundation aliases
// =====================================================================

/// Sentinel for "no GPIO pin wired" in any pin field. Matches the same
/// convention used by lib_hal — kept here so the public motor API does
/// not have to include the HAL header to reference unused pins.
constexpr uint8_t GPIO_NONE = 0xFF;

/// Step-domain types. The whole internal motion machinery lives in this
/// vocabulary:
///   - `Position` is signed (the axis can be at negative coordinates
///     relative to its home).
///   - `Distance` here means "the integer step count of a relative move"
///     and is signed (negative = backward). User-facing `Distance` (the
///     value-with-unit type) is a separate struct further down.
///   - `Velocity` / SPS is unsigned — direction is carried separately
///     by `Direction`.
///   - `Microseconds` is signed 64-bit to match lib's
///     `ungula::core::time` API without narrowing.
using Position = int32_t;
using StepCount = int32_t;
using Velocity = uint32_t; // steps per second (SPS)
using Microseconds = int64_t;

/// Direction of commanded motion. Forward / Backward are the
/// host-facing labels; the driver maps them to wire-level DIR levels per
/// its polarity configuration.
enum class Direction : uint8_t {
        Forward = 0,
        Backward = 1,
};

/// Inversion helper. Useful inside drivers and planners.
constexpr Direction reverseOf(Direction d)
{
        return (d == Direction::Forward) ? Direction::Backward : Direction::Forward;
}

/// Typed wrapper for an axis identifier. Using a struct rather than a
/// raw uint8 keeps `axis.id()` self-documenting and avoids accidental
/// comparison against arbitrary integers.
struct MotorAxisId {
        uint8_t value = 0;
        constexpr MotorAxisId() = default;
        constexpr explicit MotorAxisId(uint8_t v)
                : value(v)
        {
        }
};

constexpr bool operator==(MotorAxisId a, MotorAxisId b)
{
        return a.value == b.value;
}
constexpr bool operator!=(MotorAxisId a, MotorAxisId b)
{
        return !(a == b);
}

// =====================================================================
// Value-with-unit types
// =====================================================================
//
// The public API exposes Speed / Distance / Acceleration as tagged value
// types. The host writes them in whatever unit feels natural; the axis
// converts to its internal SPS / steps / SPS² domain at the call
// boundary using its configured `MotorUnits`. Conversion failure (e.g.
// `Speed::rpm(...)` on an axis whose `stepsPerRevolution` is zero) is
// surfaced as `ErrorCode::InvalidConfig` from the calling method.
//
// Value types are integers (uint32_t for speed / acceleration magnitudes,
// int32_t for distance so direction is encoded by sign). Motor-domain
// quantities — RPM, steps/sec, mm/sec, mm — are inherently integer at
// the resolution any stepper or servo can act on, and float just forces
// every call site to cast its integer literals. Hosts needing sub-unit
// precision pick a smaller unit (e.g. `Um` instead of `Mm`, `MmPerSec`
// instead of `CmPerSec`). The conversion math inside the axis still
// uses float internally where the mechanical factors (stepsPerMm,
// stepsPerDegree) are fractional.

enum class SpeedUnit : uint8_t {
        StepsPerSec,
        Rpm,
        DegreesPerSec,
        MmPerSec,
        CmPerSec,
        CmPerMin,
};

enum class DistanceUnit : uint8_t {
        Steps,
        Revolutions,
        Degrees,
        Mm,
        Cm,
        // Micrometres — for sub-mm jog steps or fine positioning where
        // Mm would round to zero. 1 mm = 1000 um.
        Um,
};

enum class AccelUnit : uint8_t {
        StepsPerSecSquared,
        RpmPerSec,
        DegreesPerSecSquared,
        MmPerSecSquared,
        // Ramp duration in milliseconds: time to go from 0 to the axis's
        // configured cruise speed. Resolution happens at the axis boundary
        // using `resolvedCruiseSps_`, so the same Acceleration value
        // produces different SPS² rates on axes with different cruise.
        // Useful for "always reach speed in X ms" specs from mechanical
        // engineers — the host states the ramp shape, the lib computes
        // the rate.
        RampMs,
        // Sentinel: "no ramp at all" — the planner emits a single cruise
        // segment, the motor jumps to cruise on the first edge. Distinct
        // from numeric `value == 0` so `MotionProfile` can carry the
        // intent unambiguously (a value-only `0` in `setProfile` means
        // "no override, leave the current accel alone" — see
        // `MotorAxis::setProfile`). `value` is ignored for this unit.
        NoRamp,
};

struct Speed {
        uint32_t value = 0;
        SpeedUnit unit = SpeedUnit::StepsPerSec;

        static constexpr Speed stepsPerSec(uint32_t v)
        {
                return { v, SpeedUnit::StepsPerSec };
        }
        static constexpr Speed rpm(uint32_t v)
        {
                return { v, SpeedUnit::Rpm };
        }
        static constexpr Speed degreesPerSec(uint32_t v)
        {
                return { v, SpeedUnit::DegreesPerSec };
        }
        static constexpr Speed mmPerSec(uint32_t v)
        {
                return { v, SpeedUnit::MmPerSec };
        }
        static constexpr Speed cmPerSec(uint32_t v)
        {
                return { v, SpeedUnit::CmPerSec };
        }
        static constexpr Speed cmPerMin(uint32_t v)
        {
                return { v, SpeedUnit::CmPerMin };
        }
};

struct DistanceValue {
        int32_t value = 0;
        DistanceUnit unit = DistanceUnit::Steps;

        static constexpr DistanceValue steps(int32_t v)
        {
                return { v, DistanceUnit::Steps };
        }
        static constexpr DistanceValue revolutions(int32_t v)
        {
                return { v, DistanceUnit::Revolutions };
        }
        static constexpr DistanceValue degrees(int32_t v)
        {
                return { v, DistanceUnit::Degrees };
        }
        static constexpr DistanceValue mm(int32_t v)
        {
                return { v, DistanceUnit::Mm };
        }
        static constexpr DistanceValue cm(int32_t v)
        {
                return { v, DistanceUnit::Cm };
        }
        /// Micrometre-grain distance for sub-mm jog steps where Mm
        /// would round to zero. 1 mm = 1000 um.
        static constexpr DistanceValue um(int32_t v)
        {
                return { v, DistanceUnit::Um };
        }
};

struct Acceleration {
        uint32_t value = 0;
        AccelUnit unit = AccelUnit::StepsPerSecSquared;

        static constexpr Acceleration stepsPerSecSquared(uint32_t v)
        {
                return { v, AccelUnit::StepsPerSecSquared };
        }
        static constexpr Acceleration rpmPerSec(uint32_t v)
        {
                return { v, AccelUnit::RpmPerSec };
        }
        static constexpr Acceleration degreesPerSecSquared(uint32_t v)
        {
                return { v, AccelUnit::DegreesPerSecSquared };
        }
        static constexpr Acceleration mmPerSecSquared(uint32_t v)
        {
                return { v, AccelUnit::MmPerSecSquared };
        }
        /// Ramp duration in milliseconds — time from 0 to the axis's
        /// configured cruise speed. The axis resolves to SPS² at the
        /// call boundary using its current cruise SPS, so this stays
        /// consistent if the host later changes cruise via `setSpeed`.
        /// A value of 0 ms is rejected as `InvalidConfig` (the host
        /// must use `Acceleration::noRamp()` to ask for "no ramp at all").
        static constexpr Acceleration rampMs(uint32_t v)
        {
                return { v, AccelUnit::RampMs };
        }
        /// "No ramp at all" sentinel — planner emits a single cruise
        /// segment, motor jumps to cruise on the first edge. Use this
        /// inside a `MotionProfile` to express "explicitly disable
        /// ramping" without colliding with the `value == 0` "no
        /// override" encoding `setProfile` uses. Direct `setAcceleration`
        /// callers can pass either this or `stepsPerSecSquared(0)` and
        /// get the same effect.
        static constexpr Acceleration noRamp()
        {
                return { 0u, AccelUnit::NoRamp };
        }
};

// =====================================================================
// Mechanical conversion factors
// =====================================================================
//
// Filled by the host once per axis. The axis uses these to resolve any
// host-facing Speed / Distance / Acceleration to the internal step
// domain. Zero means "this unit is not configured" — any attempt to
// command in that unit returns `ErrorCode::InvalidConfig`.

struct MotorUnits {
        /// Total microsteps for one full revolution of the OUTPUT shaft.
        /// For a 1.8° NEMA at 16× microstep with no gearing this is
        /// 200 × 16 = 3200. With a 1:5 gearbox it would be 16 000.
        uint32_t stepsPerRevolution = 0;

        /// Optional linear conversion: how many microsteps per millimetre
        /// of carriage travel. Set when the mechanical chain (lead screw,
        /// belt + pulley) translates rotation into linear motion. 0 =
        /// linear units (mm / cm) are not allowed for this axis.
        float stepsPerMm = 0.0f;

        /// Optional rotary conversion: how many microsteps per degree of
        /// shaft rotation. Usually derived from
        /// `stepsPerRevolution / 360.0f`, but exposed independently so a
        /// host can override (e.g. for a gear chain with non-integer
        /// ratios). 0 = degree-based units are not allowed.
        float stepsPerDegree = 0.0f;
};

} // namespace ungula::motor
