// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/motor/axis_types.h"

namespace ungula::motor
{

/// Sentinel pin value. Matches the project-wide convention.
inline constexpr uint8_t GPIO_NONE = 0xFF;

/// What role this input plays. Each role has different timing and
/// safety semantics — they are intentionally separate types in the
/// public API rather than a single "switch".
enum class SensorRole : uint8_t {
        Home = 0, // reference; polled with debounce.
        TravelLimit = 1, // prevents commanded travel past physical range.
        CrashLimit = 2, // hard limit; interrupt-driven, immediate stop.
        EmergencyStop = 3, // E-stop; interrupt-driven, immediate stop + fault.
};

/// Active level of the sensor. NormallyClosed = pressed → reads LOW.
/// NormallyOpen  = pressed → reads HIGH. The default for safety inputs
/// is NormallyClosed so a cut wire reads as "pressed" (fail-safe).
enum class SensorPolarity : uint8_t {
        NormallyClosed = 0, // fail-safe; pressed = LOW
        NormallyOpen = 1, // pressed = HIGH
};

/// Configuration for a single sensor input. Pulls are external by
/// default on the project (matches the GPIO34/35 wiring discipline).
struct SensorInputConfig {
        uint8_t pin = GPIO_NONE;
        SensorRole role = SensorRole::Home;
        SensorPolarity polarity = SensorPolarity::NormallyClosed;
        Direction direction = Direction::Backward; // which direction this sensor blocks
        uint16_t debounceMs = 20; // for polled roles (Home, TravelLimit)
};

} // namespace ungula::motor
