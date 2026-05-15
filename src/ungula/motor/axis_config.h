// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/motor/axis_types.h"
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/pulse/i_pulse_engine.h"

namespace ungula::motor
{

inline constexpr uint8_t MAX_SENSOR_INPUTS = 6; // home + 2× travel + crash + estop + spare

/// Compile-time strongly-typed pin wrappers. These exist purely to avoid
/// the "which uint8_t was step and which was dir" footgun in user code.
struct StepPin {
        uint8_t value = GPIO_NONE;
};
struct DirectionPin {
        uint8_t value = GPIO_NONE;
};
struct EnablePin {
        uint8_t value = GPIO_NONE;
};
struct InputPin {
        uint8_t value = GPIO_NONE;
};

/// Common to all axis configs.
struct AxisCommonConfig {
        AxisId axisId = AxisId(0);
        const char *name = "axis";
        UnitScaling units = {};
        TrajectoryLimits limits = {};
};

/// Open-loop STEP/DIR stepper axis (TMC2209, A4988, DRV8825, ...).
struct StepDirStepperAxisConfig {
        AxisCommonConfig common{};

        StepPin stepPin;
        DirectionPin dirPin;
        EnablePin enablePin; // may be GPIO_NONE if not wired

        PulseMode pulseMode = PulseMode::Internal;

        bool dirActiveHigh = true; // common convention; flip per drive
        bool enableActiveLow = true; // most stepper drivers are active-LOW EN
        uint32_t dirSetupUs = 5; // hold DIR stable before STEP rising edge

        SensorInputConfig sensors[MAX_SENSOR_INPUTS]{};
        uint8_t sensorCount = 0;
};

/// STEP/DIR servo axis. Same pulse stream as a stepper but the drive
/// owns position truth — optional encoder feedback, optional alarm,
/// optional in-position digital.
struct StepDirServoAxisConfig {
        AxisCommonConfig common{};

        StepPin stepPin;
        DirectionPin dirPin;
        EnablePin enablePin; // optional; many servo drives manage enable in hardware
        InputPin alarmInputPin; // optional; drive ALM output
        InputPin inPositionInputPin; // optional; drive INP/COIN output

        PulseMode pulseMode = PulseMode::Internal;

        bool dirActiveHigh = true;
        bool enableActiveLow = false; // industrial servos usually active-HIGH (SRV-ON)
        bool alarmActiveLow = true; // industrial servos: ALM is open-collector, LOW = fault
        bool inPositionActiveHigh = true;
        uint32_t dirSetupUs = 5;

        SensorInputConfig sensors[MAX_SENSOR_INPUTS]{};
        uint8_t sensorCount = 0;
};

/// CAN servo axis. No STEP/DIR pins at all.
struct CanServoAxisConfig {
        AxisCommonConfig common{};

        uint8_t canNodeId = 0;
        uint8_t canControllerNumber = 0; // index into ungula::hal::can
        /// Conversion factor from drive encoder counts to user steps. For
        /// CAN servos the planner still works in "steps"; this scales the
        /// drive's native units to that domain.
        float unitsPerEncoderCount = 1.0f;
};

} // namespace ungula::motor
