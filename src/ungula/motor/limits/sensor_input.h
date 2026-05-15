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
        CrashLimit = 2, // hard limit; ISR; immediate stop with `LimitSwitch` reason.
        EmergencyStop = 3, // E-stop; ISR; immediate stop + fault with `EmergencyStop` reason.
        Stall = 4, // stall detector (e.g. TMC2209 DIAG); ISR with hit-count debounce.
                   // Reports `StopReason::StallDetected` + `FaultCode::Stall` so the host
                   // can tell a real driver fault apart from a chip stall event. See the
                   // `stallHitsToTrigger` / `stallArmDelayMs` knobs below.
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

        // ---- Stall-specific tuning (role == Stall only) ----------------
        //
        // The TMC2209's DIAG pin tends to glitch high at least once during
        // the first ~100 ms of motion while StealthChop's auto-tune is
        // still characterising the coils. Without filtering, that boot
        // transient looks identical to a genuine stall. Two complementary
        // knobs handle it:

        /// Number of ISR hits required before the bank latches a stall.
        /// 1 = trigger on the first DIAG pulse (no filtering); 4 (default)
        /// rejects a single spurious pulse but still fires within a few
        /// ms of a real stall (genuine stalls hold DIAG high → many
        /// repeated edges per service tick).
        uint8_t stallHitsToTrigger = 4;

        /// Milliseconds after motion start during which DIAG hits are
        /// counted but the bank does NOT latch a stall. Filters the
        /// StealthChop auto-tune transient entirely. Increase if your
        /// motor's auto-tune is slower; decrease to 0 to disable.
        uint16_t stallArmDelayMs = 200;
};

} // namespace ungula::motor
