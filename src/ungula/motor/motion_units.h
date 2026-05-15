// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include "ungula/motor/axis_types.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

// ============================================================================
// Speed
// ============================================================================

/// Unit a `Speed` value carries. Resolved to `Velocity` (steps/sec) at
/// the point of use against an `UnitScaling`.
enum class SpeedUnit : uint8_t {
    StepsPerSec    = 0,
    MmPerSec       = 1,
    MmPerMin       = 2,
    CmPerSec       = 3,
    CmPerMin       = 4,
    InchesPerSec   = 5,
    InchesPerMin   = 6,
    DegreesPerSec  = 7,
    Rpm            = 8,   // revolutions per minute
    Rps            = 9,   // revolutions per second
};

/// User-facing speed type. Carries a value AND a unit so the call site
/// can't mix them up. Named factories (preferred): `Speed::mmPerSec(150.0f)`,
/// `Speed::rpm(60.0f)`, ...
///
/// Resolved to internal `Velocity` (steps/sec) via `toStepsPerSec()`
/// with an `UnitScaling`. Rotary units (`Rpm`, `Rps`, `DegreesPerSec`)
/// require `stepsPerDegree`; linear units (`MmPerSec`, `CmPerMin`,
/// `InchesPerMin`, ...) require `stepsPerMm`. An unconfigured scaling
/// produces `ErrorCode::InvalidConfig` at resolve time — never a silent
/// zero.
struct Speed {
    float     value = 0.0f;
    SpeedUnit unit  = SpeedUnit::StepsPerSec;

    static constexpr Speed stepsPerSec(float v)   { return Speed{v, SpeedUnit::StepsPerSec};   }
    static constexpr Speed mmPerSec(float v)      { return Speed{v, SpeedUnit::MmPerSec};      }
    static constexpr Speed mmPerMin(float v)      { return Speed{v, SpeedUnit::MmPerMin};      }
    static constexpr Speed cmPerSec(float v)      { return Speed{v, SpeedUnit::CmPerSec};      }
    static constexpr Speed cmPerMin(float v)      { return Speed{v, SpeedUnit::CmPerMin};      }
    static constexpr Speed inchesPerSec(float v)  { return Speed{v, SpeedUnit::InchesPerSec};  }
    static constexpr Speed inchesPerMin(float v)  { return Speed{v, SpeedUnit::InchesPerMin};  }
    static constexpr Speed degreesPerSec(float v) { return Speed{v, SpeedUnit::DegreesPerSec}; }
    static constexpr Speed rpm(float v)           { return Speed{v, SpeedUnit::Rpm};           }
    static constexpr Speed rps(float v)           { return Speed{v, SpeedUnit::Rps};           }
};

// ============================================================================
// Acceleration
// ============================================================================

/// Unit an `Acceleration` value carries. Resolved to `uint32_t`
/// steps/sec² at the point of use. The conversion factor is the same
/// as for `Speed` of the same dimensional family — accel and velocity
/// share their distance-to-steps math, only the time-power differs.
enum class AccelerationUnit : uint8_t {
    StepsPerSecSquared   = 0,
    MmPerSecSquared      = 1,
    CmPerSecSquared      = 2,
    InchesPerSecSquared  = 3,
    DegreesPerSecSquared = 4,
    RpmPerSec            = 5,   // revolutions-per-minute / sec
    RpsPerSec            = 6,   // revolutions-per-sec   / sec
};

/// User-facing acceleration type. Same shape and lifecycle as `Speed`.
/// Named factories: `Acceleration::mmPerSecSquared(500.0f)`,
/// `Acceleration::rpmPerSec(120.0f)`, ...
struct Acceleration {
    float            value = 0.0f;
    AccelerationUnit unit  = AccelerationUnit::StepsPerSecSquared;

    static constexpr Acceleration stepsPerSecSquared(float v)
        { return Acceleration{v, AccelerationUnit::StepsPerSecSquared}; }
    static constexpr Acceleration mmPerSecSquared(float v)
        { return Acceleration{v, AccelerationUnit::MmPerSecSquared}; }
    static constexpr Acceleration cmPerSecSquared(float v)
        { return Acceleration{v, AccelerationUnit::CmPerSecSquared}; }
    static constexpr Acceleration inchesPerSecSquared(float v)
        { return Acceleration{v, AccelerationUnit::InchesPerSecSquared}; }
    static constexpr Acceleration degreesPerSecSquared(float v)
        { return Acceleration{v, AccelerationUnit::DegreesPerSecSquared}; }
    static constexpr Acceleration rpmPerSec(float v)
        { return Acceleration{v, AccelerationUnit::RpmPerSec}; }
    static constexpr Acceleration rpsPerSec(float v)
        { return Acceleration{v, AccelerationUnit::RpsPerSec}; }
};

// ============================================================================
// Conversion internals — exposed only for tests / advanced use.
// ============================================================================

namespace detail
{

/// Convert a user-units value to steps per (time^N) given a linear or
/// rotary scaling. Returns the converted magnitude as a float; the
/// caller saturates / casts to the right integer type.
///
/// `secondsDivisor` distinguishes per-second (1.0f) vs per-minute (60.0f)
/// time bases. Distance-family factors (mm, cm, inches) multiply by
/// `stepsPerMm`; angular-family factors (degrees, rev) multiply by
/// `stepsPerDegree` (with rev expanded to 360°). `wantStepsPerDegree`
/// reports back which scaling field the caller needs configured —
/// used to surface a precise `InvalidConfig` error.
inline float scaleFactor(SpeedUnit unit, bool& wantStepsPerMm, bool& wantStepsPerDegree,
                          float stepsPerMm, float stepsPerDegree)
{
    wantStepsPerMm = false;
    wantStepsPerDegree = false;
    switch (unit) {
    case SpeedUnit::StepsPerSec:    return 1.0f;
    case SpeedUnit::MmPerSec:       wantStepsPerMm = true; return stepsPerMm;
    case SpeedUnit::MmPerMin:       wantStepsPerMm = true; return stepsPerMm / 60.0f;
    case SpeedUnit::CmPerSec:       wantStepsPerMm = true; return stepsPerMm * 10.0f;
    case SpeedUnit::CmPerMin:       wantStepsPerMm = true; return stepsPerMm * 10.0f / 60.0f;
    case SpeedUnit::InchesPerSec:   wantStepsPerMm = true; return stepsPerMm * 25.4f;
    case SpeedUnit::InchesPerMin:   wantStepsPerMm = true; return stepsPerMm * 25.4f / 60.0f;
    case SpeedUnit::DegreesPerSec:  wantStepsPerDegree = true; return stepsPerDegree;
    case SpeedUnit::Rpm:            wantStepsPerDegree = true; return stepsPerDegree * 360.0f / 60.0f;
    case SpeedUnit::Rps:            wantStepsPerDegree = true; return stepsPerDegree * 360.0f;
    }
    return 0.0f;
}

/// Acceleration mirrors Speed — the steps/<unit> conversion is the
/// same; only the time dimension differs and that's already accounted
/// for in the per-second / per-minute factor. Implemented separately
/// so the enum types don't accidentally bleed across.
inline float scaleFactor(AccelerationUnit unit, bool& wantStepsPerMm, bool& wantStepsPerDegree,
                          float stepsPerMm, float stepsPerDegree)
{
    wantStepsPerMm = false;
    wantStepsPerDegree = false;
    switch (unit) {
    case AccelerationUnit::StepsPerSecSquared:   return 1.0f;
    case AccelerationUnit::MmPerSecSquared:      wantStepsPerMm = true; return stepsPerMm;
    case AccelerationUnit::CmPerSecSquared:      wantStepsPerMm = true; return stepsPerMm * 10.0f;
    case AccelerationUnit::InchesPerSecSquared:  wantStepsPerMm = true; return stepsPerMm * 25.4f;
    case AccelerationUnit::DegreesPerSecSquared: wantStepsPerDegree = true; return stepsPerDegree;
    case AccelerationUnit::RpmPerSec:            wantStepsPerDegree = true; return stepsPerDegree * 360.0f / 60.0f;
    case AccelerationUnit::RpsPerSec:            wantStepsPerDegree = true; return stepsPerDegree * 360.0f;
    }
    return 0.0f;
}

}  // namespace detail

// ============================================================================
// Public resolvers
// ============================================================================

/// Convert a `Speed` to `Velocity` (steps/sec) using the supplied
/// `UnitScaling`. Returns `InvalidConfig` if the input unit needs a
/// scaling field that is zero, or if the magnitude is negative or
/// overflows `int32_t`. Use `Direction` separately to express sign —
/// `Speed` is always a magnitude.
inline Result<Velocity> toStepsPerSec(Speed s, const UnitScaling& units)
{
    bool wantStepsPerMm = false;
    bool wantStepsPerDegree = false;
    const float factor = detail::scaleFactor(s.unit,
                                              wantStepsPerMm, wantStepsPerDegree,
                                              units.stepsPerMm, units.stepsPerDegree);
    if (wantStepsPerMm     && units.stepsPerMm     <= 0.0f) return Result<Velocity>::Err(ErrorCode::InvalidConfig);
    if (wantStepsPerDegree && units.stepsPerDegree <= 0.0f) return Result<Velocity>::Err(ErrorCode::InvalidConfig);

    const float result = s.value * factor;
    if (result < 0.0f) return Result<Velocity>::Err(ErrorCode::InvalidConfig);
    if (result > static_cast<float>(std::numeric_limits<Velocity>::max())) {
        return Result<Velocity>::Err(ErrorCode::InvalidConfig);
    }
    return Result<Velocity>::Ok(static_cast<Velocity>(std::lround(result)));
}

/// Convert an `Acceleration` to steps/sec² using the supplied
/// `UnitScaling`. Same failure modes as `toStepsPerSec`. Acceleration
/// must be strictly positive — zero would make the planner refuse to
/// build a ramp and is treated as `InvalidConfig`.
inline Result<uint32_t> toStepsPerSecSquared(Acceleration a, const UnitScaling& units)
{
    bool wantStepsPerMm = false;
    bool wantStepsPerDegree = false;
    const float factor = detail::scaleFactor(a.unit,
                                              wantStepsPerMm, wantStepsPerDegree,
                                              units.stepsPerMm, units.stepsPerDegree);
    if (wantStepsPerMm     && units.stepsPerMm     <= 0.0f) return Result<uint32_t>::Err(ErrorCode::InvalidConfig);
    if (wantStepsPerDegree && units.stepsPerDegree <= 0.0f) return Result<uint32_t>::Err(ErrorCode::InvalidConfig);

    const float result = a.value * factor;
    if (result <= 0.0f) return Result<uint32_t>::Err(ErrorCode::InvalidConfig);
    if (result > static_cast<float>(std::numeric_limits<uint32_t>::max())) {
        return Result<uint32_t>::Err(ErrorCode::InvalidConfig);
    }
    return Result<uint32_t>::Ok(static_cast<uint32_t>(std::lround(result)));
}

// ============================================================================
// Setup-time helpers — mutate a TrajectoryLimits before the axis is built.
// ============================================================================
//
// Usage:
//   StepDirStepperAxisConfig cfg;
//   cfg.common.units.stepsPerMm     = 80.0f;
//   cfg.common.units.stepsPerDegree = 8.889f;
//   applyMaxVelocity (cfg.common.limits, Speed::mmPerSec(150.0f),       cfg.common.units);
//   applyAcceleration(cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);
//   applyDeceleration(cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);
//
// Returns the first `InvalidConfig` and leaves the limits struct
// unchanged on failure (caller can chain checks).

inline Status applyMaxVelocity(TrajectoryLimits& L, Speed s, const UnitScaling& units)
{
    auto r = toStepsPerSec(s, units);
    if (!r.ok()) return Status::Err(r.error());
    L.maxVelocitySps = r.takeValue();
    return Status::Ok();
}

inline Status applyAcceleration(TrajectoryLimits& L, Acceleration a, const UnitScaling& units)
{
    auto r = toStepsPerSecSquared(a, units);
    if (!r.ok()) return Status::Err(r.error());
    L.accelSpsPerSec = r.takeValue();
    return Status::Ok();
}

inline Status applyDeceleration(TrajectoryLimits& L, Acceleration a, const UnitScaling& units)
{
    auto r = toStepsPerSecSquared(a, units);
    if (!r.ok()) return Status::Err(r.error());
    L.decelSpsPerSec = r.takeValue();
    return Status::Ok();
}

/// Convenience: set accel == decel in one call. Common for symmetric
/// profiles where ramp-up and ramp-down match.
inline Status applyRampProfile(TrajectoryLimits& L, Acceleration a, const UnitScaling& units)
{
    auto r = toStepsPerSecSquared(a, units);
    if (!r.ok()) return Status::Err(r.error());
    const uint32_t sps2 = r.takeValue();
    L.accelSpsPerSec = sps2;
    L.decelSpsPerSec = sps2;
    return Status::Ok();
}

}  // namespace ungula::motor
