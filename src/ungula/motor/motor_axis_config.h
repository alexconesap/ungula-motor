// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_limit_system.h"
#include "ungula/motor/motion_profile.h"
#include "ungula/motor/motor_intent.h"
#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// Motion-shape limits, in host-facing units. The axis resolves these to
/// the internal SPS / steps domain at `begin()` time using
/// `MotorUnits`. `hardStepRateCeilingSps` is the absolute SPS ceiling
/// the planner refuses to exceed even if `maxSpeed` resolves to
/// something higher — a safety net against host bugs that command
/// runaway speeds.
struct MotorLimits {
        Speed maxSpeed = Speed::stepsPerSec(0);
        Acceleration accel = Acceleration::stepsPerSecSquared(0);
        Acceleration decel = Acceleration::stepsPerSecSquared(0);
        uint32_t hardStepRateCeilingSps = 200000u;
};

/// The only host-facing config struct. Contains no driver-specific
/// fields — driver-specific config (`Tmc2209Config`, `YpmcConfig`, …)
/// lives next to the driver and is owned by the driver instance the
/// host hands to the axis constructor.
struct MotorAxisConfig {
        MotorAxisId id;
        const char *name = "axis"; // string literal only — not copied

        MotorUnits units;
        MotorLimits limits;
        MotionProfile profile = MotionProfile::cruise();
        MotorIntent intent = MotorIntent::Default;

        /// Fixed-size limit wiring table. Fill the slots you need;
        /// leave the rest as default-constructed (their pin field
        /// stays at `GPIO_NONE`, which the limit system treats as
        /// unused). The auto-count overload of
        /// `LimitSystem::begin(cfg.limits_wiring, engine)` figures
        /// out how many slots are in use
        LimitWiring limits_wiring[MAX_LIMIT_INPUTS];
};

} // namespace ungula::motor
