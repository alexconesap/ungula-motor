// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include "ungula/motor/motor_intent.h"
#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// Bundle of motion-shape parameters. A host might keep several profiles
/// (cruise / jog / home) in static memory and swap them at runtime via
/// `MotorAxis::setProfile()`. Profile-level `intent` overrides the
/// axis-level intent for the duration of motions issued under this
/// profile — useful for "be quiet during homing, normal during cruise".
///
/// Switching profiles mid-motion does NOT retroactively reshape the
/// currently-running move; the new profile takes effect on the next
/// motion command. This keeps the planner simple and the host
/// predictable.
struct MotionProfile {
        Speed maxSpeed = Speed::stepsPerSec(0);
        Acceleration accel = Acceleration::stepsPerSecSquared(0);
        Acceleration decel = Acceleration::stepsPerSecSquared(0);
        MotorIntent intent = MotorIntent::Default;

        // -------- Named factories with sensible defaults ------------------
        //
        // These return "shape-only" profiles — `maxSpeed` / `accel` /
        // `decel` are left as the axis's configured `MotorLimits`
        // defaults (the axis fills them at apply time if the profile's
        // value is zero). Host code can mutate any field after calling
        // the factory.

        static constexpr MotionProfile cruise()
        {
                // Default behaviour. Axis runs at configured maxSpeed
                // with configured ramps and no intent overrides.
                MotionProfile p{};
                p.intent = MotorIntent::Default;
                return p;
        }

        static constexpr MotionProfile jog()
        {
                // Manual-jog defaults: gentler ramp, quiet operation.
                // Host can override before applying.
                MotionProfile p{};
                p.intent = MotorIntent::Quiet;
                return p;
        }

        static constexpr MotionProfile home()
        {
                // Homing should approach the switch carefully. No
                // adaptive-current games; predictable behaviour wins.
                MotionProfile p{};
                p.intent = MotorIntent::Precision;
                return p;
        }
};

} // namespace ungula::motor
