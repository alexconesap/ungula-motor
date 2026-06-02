// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/driver_identity.h"
#include "ungula/motor/motor_diagnostics.h"
#include "ungula/motor/motor_intent.h"
#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_units.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Motion status the driver reports each service tick. Compact: the
/// axis only needs to know "still moving?" / "faulted?" / "why did it
/// finish?" for its FSM.
struct DriverMotionStatus {
        bool running = false;
        bool faulted = false;
        StopReason finishedReason = StopReason::None;
};

/// The ENTIRE contract a new chip driver implements. Writing a new
/// driver = one header + one source file that fulfils these methods. No
/// other library entry points; no inheritance hierarchy below this.
///
/// Drivers may internally compose helpers (configurator, transport,
/// stallguard, …) — that's an implementation detail. The public-facing
/// shape stays a flat ~15-method interface.
///
/// Lifecycle expected by `MotorAxis`:
///
///   1. Host instantiates the concrete driver with its private config.
///   2. Host constructs `MotorAxis(cfg, driver_ref)`.
///   3. `MotorAxis::begin()` calls `driver.begin()`.
///   4. `MotorAxis::enable()` calls `driver.enable()`.
///   5. Motion verbs on the axis dispatch to `armMove` / `armJog` /
///      `stop` on the driver. The axis converts host-facing units to
///      internal SPS / steps before the call — drivers see only the
///      step domain.
///   6. The axis polls `motionStatus()` / `commandedPositionSteps()` /
///      `commandedSpsNow()` every service tick. The driver is
///      responsible for ISR-level halt latching (limit / stall via the
///      shared `ILimitSystem`); the axis surfaces FSM transitions in
///      task context.
///   7. `MotorAxis::clearFault()` calls `driver.clearFault()`. The
///      driver is responsible for clearing any chip-side latched fault
///      flags.
class IMotorDriver {
    public:
        virtual ~IMotorDriver() = default;

        // ---- Lifecycle ---------------------------------------------------
        virtual Status begin() = 0;
        virtual Status enable() = 0;
        virtual Status disable() = 0;
        virtual Status clearFault() = 0;

        // ---- Motion ------------------------------------------------------
        //
        // All arguments are already in the internal step domain. The
        // axis is responsible for unit conversion + range checking
        // BEFORE calling these. Drivers should not silently re-clamp
        // SPS / steps; they should refuse invalid arguments with
        // `ErrorCode::InvalidConfig`.
        virtual Status armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps,
                               uint32_t accelSps2, uint32_t decelSps2) = 0;
        virtual Status armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2) = 0;
        virtual Status stop(StopMode mode) = 0;

        // ---- Status (polled from service tick) ---------------------------
        virtual DriverMotionStatus motionStatus() const = 0;
        virtual Position commandedPositionSteps() const = 0;
        virtual uint32_t commandedSpsNow() const = 0;

        /// Timer / RMT resolution the driver's pulse generator runs at,
        /// in Hz. Hosts feed it to `TrapezoidalPlanner::actualSpsFor`
        /// to compute the real step rate the hardware emits for a
        /// given request (the planner ceiling-rounds the half-period;
        /// `actual_sps = resolutionHz / (2 * hp)`).
        ///
        /// Default returns 0 = "unknown" for drivers without a host-
        /// visible timer (RMD CAN servos plan internally). STEP/DIR
        /// drivers override to forward the step generator's resolution.
        virtual uint32_t timerResolutionHz() const
        {
                return 0u;
        }

        /// Move the driver's internal step counter without issuing
        /// pulses. Used by homing strategies to zero the position at
        /// the home reference.
        virtual Status resetPosition(Position newSteps) = 0;

        // ---- Identity ----------------------------------------------------
        //
        // Always answers. Drivers without a runtime identity register
        // return compile-time constants; those with one (TMC2209 IOIN
        // byte) cache the read at `begin()` time. If identity is
        // genuinely unknown, return `vendor = model = "Unknown"`.
        virtual DriverIdentity identity() = 0;

        // ---- Intent application -----------------------------------------
        //
        // The axis routes its current `MotorIntent` (axis-level OR
        // profile-level override) through this call. Drivers translate
        // to whatever chip-specific settings best honour the intent and
        // report support level back to the axis.
        virtual IntentSupport applyIntent(MotorIntent intent) = 0;

        // ---- Diagnostics ------------------------------------------------
        //
        // Driver fills its share of the struct (stall fields,
        // adaptive-current fields, identity, raw blob pointer). The
        // axis fills lib-level fields (state, position, totals)
        // before/after calling this. Drivers without optional fields
        // leave the `*_valid` flags false.
        virtual void fillDriverDiagnostics(MotorDiagnostics &out) const = 0;
};

} // namespace ungula::motor
