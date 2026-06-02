// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// Composable behaviour hints. Application code asks the axis for a
/// behaviour ("be quiet", "be cool", "prefer torque"); the driver
/// decides how to honour it on its specific hardware. Drivers report
/// the result via `IntentSupport`.
///
/// Bitmask: combine flags with `|`. Default (zero) means the driver
/// applies its standard configuration with no preference.
enum MotorIntent : uint16_t {
        Default = 0,
        Quiet = 1u << 0, // prefer silent operation
        HighTorque = 1u << 1, // prefer torque over silence / heat
        EnergySaving = 1u << 2, // prefer lower coil current
        AdaptiveCurrent = 1u << 3, // dynamically scale current to load
        Precision = 1u << 4, // prefer interpolation / microstep depth
        Cool = 1u << 5, // prefer reduced heat dissipation
};

/// Bitwise OR for composing intent flags. Returns a `MotorIntent` so the
/// result is type-checked end to end.
constexpr MotorIntent operator|(MotorIntent a, MotorIntent b)
{
        return static_cast<MotorIntent>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

constexpr MotorIntent &operator|=(MotorIntent &a, MotorIntent b)
{
        a = a | b;
        return a;
}

/// Test whether a specific flag is set in a composite intent.
constexpr bool has(MotorIntent set, MotorIntent flag)
{
        return (static_cast<uint16_t>(set) & static_cast<uint16_t>(flag)) != 0;
}

/// Driver's response to an `applyIntent()` request.
enum class IntentSupport : uint8_t {
        Supported, // applied fully
        PartiallySupported, // applied with compromise (e.g. one of two flags ignored)
        Unsupported, // nothing in the intent applied to this driver
        Conflicted, // requested combination is contradictory
};

inline const char *intentSupportToString(IntentSupport s)
{
        switch (s) {
        case IntentSupport::Supported:
                return "Supported";
        case IntentSupport::PartiallySupported:
                return "PartiallySupported";
        case IntentSupport::Unsupported:
                return "Unsupported";
        case IntentSupport::Conflicted:
                return "Conflicted";
        }
        return "Unknown";
}

} // namespace ungula::motor
