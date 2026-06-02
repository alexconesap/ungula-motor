// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstddef>
#include <cstdint>

#include "ungula/motor/driver_identity.h"
#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_units.h"

namespace ungula::motor
{

/// Single flat structure aggregating every diagnostic a `MotorAxis` can
/// report. Driver-side fields are gated by their `*_valid` flag — a
/// driver without StallGuard leaves `stall_valid = false` and the host
/// ignores `stallReadingPct`. Lib-level fields are always meaningful.
///
/// This is the canonical form. JSON serialisation (`toJson`) is
/// convenience for serial / web UIs; programs querying the state via
/// `MotorAxis::diagnostics()` should consume the struct directly.
struct MotorDiagnostics {
        // ---- Always valid (lib-level) ------------------------------------
        MotorAxisId axisId;
        MotorState state = MotorState::Uninitialized;
        Position commandedPosition = 0;
        uint32_t currentSps = 0; // live planner output, SPS
        uint32_t targetSps = 0; // configured cruise, SPS
        int32_t stepsToTarget = 0; // signed; sign = direction left
        StopReason lastStopReason = StopReason::None;
        FaultCode lastFault = FaultCode::None;
        uint32_t totalStepsIssued = 0;
        bool homed = false;
        DriverIdentity identity;

        // ---- Driver-side optional: stall ---------------------------------
        bool stall_valid = false;
        uint8_t stallSensitivityPct = 0; // 0..100, configured
        uint8_t stallReadingPct = 0; // 0..100, live load reading
        uint32_t stallHitsSinceClear = 0;

        // ---- Driver-side optional: adaptive current ----------------------
        bool adaptive_current_valid = false;
        uint16_t adaptiveCurrentMa = 0; // live coil current target

        // ---- Driver-side optional: raw blob ------------------------------
        // Pointer to chip-private diagnostic text in driver-owned storage
        // (e.g. a small char buffer the driver maintains). May be nullptr.
        // Host must not free.
        const char *driverRawDiagnostics = nullptr;
};

/// Render diagnostics to a JSON string in the caller-owned buffer.
/// Returns the number of bytes written excluding the terminating NUL,
/// or 0 if the buffer is too small. Buffer is always NUL-terminated on
/// non-zero return.
///
/// Implementation is intentionally allocation-free — `snprintf` only,
/// fixed-format keys. Safe to call from any task context (NOT from
/// ISR — `snprintf` is not IRAM-safe on ESP32).
size_t toJson(const MotorDiagnostics &diag, char *outBuf, size_t outLen);

} // namespace ungula::motor
