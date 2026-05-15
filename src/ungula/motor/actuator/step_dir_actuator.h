// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/actuator/actuator_capabilities.h"
#include "ungula/motor/actuator/i_axis_actuator.h"
#include "ungula/motor/limits/sensor_input.h" // GPIO_NONE
#include "ungula/motor/pulse/i_pulse_engine.h"

namespace ungula::motor
{

/// Distinguishes the two flavours of STEP/DIR actuation. The two share
/// the same wire protocol (STEP pulses + DIR level) but differ in what
/// "actual position" means:
///
///   - `OpenLoopStepper` — no feedback. Commanded == actual is an
///     ASSUMPTION; the host accepts the risk of lost steps under
///     mechanical overload. `capabilities().hasActualPosition` is
///     always false. Common for TMC2209, A4988, DRV8825.
///   - `StepDirServo` — a closed-loop servo drive (Yaskawa, Delta, etc.)
///     that internally tracks position. Actual position may be
///     available via an optional encoder feedback channel; if no
///     encoder is wired, the actuator treats commanded as
///     authoritative but the host knows the drive is responsible
///     for tracking. `capabilities().hasInPositionInput` /
///     `hasAlarmInput` are typically true.
enum class StepDirActuatorKind : uint8_t {
        OpenLoopStepper = 0,
        StepDirServo = 1,
};

/// Open-loop stepper AND STEP/DIR servo actuator. One implementation
/// covers both because the wire protocol is identical; the
/// `StepDirActuatorKind` selects feedback semantics, not pulse
/// behaviour.
///
/// ## Ownership and lifetime
///
/// The actuator does NOT own its pulse engine — it holds a reference.
/// The Axis facade composes timer + engine + actuator at boot via the
/// production factory and manages their lifetimes through `unique_ptr`s
/// declared in destruction-safe order. Tests construct components on
/// the stack and pass them in.
///
/// ## Direction-timing contract (IMPORTANT)
///
/// The pulse engine OWNS the DIR pin. Specifically, `HalPulseEngine`:
///   1. Writes DIR to match the loaded `PlannedMove`'s direction
///      inside `start()`, before the first STEP pulse.
///   2. Waits `dirSetupUs` microseconds for the level to settle on the
///      driver side BEFORE arming the timer.
///
/// `StepDirActuator` MUST NOT touch the DIR pin. Any "casual" DIR write
/// from outside the engine timing path can violate driver setup-time
/// guarantees and produce a lost step or a reversed step. The actuator
/// only:
///   - Drives the ENABLE pin (if configured).
///   - Forwards `armMotion` / `startMotion` / `stop` calls to the
///     engine, which handles all STEP and DIR timing.
///
/// ## Scope
///
///   - `begin` / `enable` / `disable` (ENABLE pin only).
///   - `armMotion(PlannedMove)` → `engine.loadMove(move)`.
///   - `startMotion()` → `engine.start()`.
///   - `stop(mode)` → `engine.stop(mode)`. `Decelerate` propagates
///     `Unsupported` from the engine; we do NOT silently downgrade.
///   - `emergencyStop()` → `engine.emergencyStop()`.
///   - `feedback()` wraps the engine's `PulseEngineStatus` into
///     `AxisFeedback`. `currentSps` derived from the engine's current
///     half-period if running, 0 otherwise.
///   - `faultStatus()` maps engine state to `FaultStatus` (with code).
///   - `clearFault()` forwards to `engine.clearFault()`.
///
/// Not yet implemented:
///   - Alarm / in-position digital input read paths (wire via
///     `SensorBank` on the Axis layer).
///   - Encoder feedback for STEP/DIR servos.
class StepDirActuator final : public IAxisActuator {
    public:
        struct Config {
                /// Which flavour of STEP/DIR drive this is. Drives feedback
                /// semantics (`capabilities().hasActualPosition` etc.).
                StepDirActuatorKind kind = StepDirActuatorKind::OpenLoopStepper;

                /// Optional. `GPIO_NONE` means no enable pin is wired — some
                /// industrial servo drives manage SRV-ON in hardware. When
                /// `GPIO_NONE`, `enable()` and `disable()` are no-ops that
                /// return `Ok`.
                uint8_t enablePin = GPIO_NONE;

                /// Active-LOW is the convention for most open-loop stepper
                /// drivers (TMC2209, A4988, DRV8825). Active-HIGH for
                /// industrial servo drives that use SRV-ON / SVON inputs.
                bool enableActiveLow = true;

                /// Capability flags. The corresponding digital-input read
                /// paths are wired through `SensorBank` on the Axis layer;
                /// storing the flags here keeps `capabilities()` honest so
                /// hosts can plan around the read API without needing a
                /// code change later.
                bool hasAlarmInput = false;
                bool hasInPositionInput = false;
        };

        /// `engine` must outlive the actuator. Axis composes both and
        /// guarantees the lifetime ordering via `unique_ptr` declaration
        /// order (engine declared BEFORE actuator → destructed AFTER).
        StepDirActuator(IPulseEngine &engine, const Config &cfg);

        StepDirActuator(const StepDirActuator &) = delete;
        StepDirActuator &operator=(const StepDirActuator &) = delete;

        // ---- IAxisActuator ----------------------------------------------

        Status begin() override;
        Status enable() override;
        Status disable() override;

        Status armMotion(const PlannedMove &move) override;
        Status startMotion() override;

        Status stop(StopMode mode) override;
        Status emergencyStop() override;

        AxisFeedback feedback() const override;
        ActuatorCapabilities capabilities() const override;

        FaultStatus faultStatus() const override;
        Status clearFault() override;

    private:
        IPulseEngine &engine_;
        Config cfg_;
        bool begun_ = false;
        bool enabled_ = false;
};

} // namespace ungula::motor
