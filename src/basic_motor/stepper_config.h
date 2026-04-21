// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula {
    namespace motor {

        /// Generic stepper motor pin and ramp configuration
        struct StepperConfig {
                int stepPin;
                int enablePin;
                int directionPin;
                uint32_t rampTimeMs;
        };

        /// Timer constants for step pulse generation
        namespace MotorTimer {
            static constexpr uint32_t TIMER_FREQ_HZ = 1000000;  // 1 MHz step timer
            static constexpr uint32_t MIN_ALARM_TICKS = 2;
            static constexpr float TOGGLES_PER_STEP = 2.0f;
            static constexpr float MIN_RUNNING_SPS = 1.0f;
        }  // namespace MotorTimer

    }  // namespace motor
}  // namespace ungula
