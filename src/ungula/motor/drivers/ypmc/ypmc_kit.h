// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include <memory>

#include "ungula/motor/axis.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/drivers/ypmc/ypmc_servo.h"
#include "ungula/motor/result.h"

namespace ungula::motor::ypmc
{

/// Single-call bundle for one (or two-tandem) YPMC + S2SVD15 servo
/// axis. Builds the underlying `Axis` via `Axis::createStepDirServo`,
/// applies the S2SVD15 timing / polarity defaults, and optionally
/// constructs + binds a `BrakeController` to the axis event bus.
///
/// ## Tandem (two servos on one STEP pin)
///
/// The use case is two YPMC servos mounted face-to-face on the same
/// shaft, both driven by the same STEP signal but with separate
/// DIR + SRV-ON inputs. Set the `secondary*` pin fields; the pulse
/// engine writes both DIRs atomically inside the `dirSetupUs` window
/// and the actuator drives both SRV-ON inputs from `enable()` /
/// `disable()`. Set `secondaryDirInverted = true` for the typical
/// face-to-face mounting (one motor must rotate the OPPOSITE
/// electrical direction to produce the SAME physical rotation).
///
/// ## Lifetime
///
/// All members are `unique_ptr` owned by the kit. No file-scope or
/// singleton state — instantiate as many kits as your project needs.
struct ServoKitConfig {
        AxisCommonConfig common{};

        // ---- Pin wiring -------------------------------------------------
        StepPin stepPin{};
        DirectionPin dirPin{};
        EnablePin enablePin{}; // SRV-ON; GPIO_NONE allowed when hardwired
        InputPin alarmInputPin{}; // ALM− (open-collector, active-LOW)
        InputPin inPositionInputPin{}; // COIN / INP (active-HIGH)

        /// Tandem wiring (see class doc). All default to GPIO_NONE.
        DirectionPin secondaryDirPin{};
        bool secondaryDirActiveHigh = true;
        bool secondaryDirInverted = false;
        EnablePin secondaryEnablePin{}; // secondary SRV-ON
        bool secondaryEnableActiveLow = false; // S2SVD15 SRV-ON is active-HIGH

        /// Drive timing + polarity. Defaults match the S2SVD15
        /// datasheet (CMD+DIR mode, 5..10 µs dirSetup).
        DriveTiming timing{};
        DrivePolarity polarity{};

        // ---- Optional brake controller ----------------------------------
        /// When true, the kit constructs a `BrakeController` and binds
        /// it to the axis event bus from `begin()`. Without this,
        /// brake-handling is the host's responsibility.
        bool useBrake = false;
        BrakeController::Config brake{};

        // ---- Sensors ----------------------------------------------------
        /// Additional sensors (home, travel limits, etc.). The ALM
        /// input, when set on `alarmInputPin`, is wired automatically
        /// by `applyDriveDefaults()` and does NOT need to appear here.
        SensorInputConfig sensors[MAX_SENSOR_INPUTS]{};
        uint8_t sensorCount = 0;
};

/// Bundle returned by `makeServoKit()`. `axis` is the motion handle;
/// `brake` is null when `useBrake == false`.
class YpmcServoKit {
    public:
        /// One-shot lifecycle:
        ///   1. `brake->begin()` if present (seeds the relay to ENGAGED).
        ///   2. `axis->begin()` (configures STEP/DIR/EN pins, sensors).
        ///   3. The brake controller is bound to the axis event bus so
        ///      it auto-engages on `MotionCompleted` / fault.
        ///
        /// The host is responsible for calling `axis->enable()` and
        /// `brake->release()` (in that order) before commanding motion.
        /// Pre-`axis->enable()` brake-release leaves the motor floating
        /// against gravity on a vertical axis.
        Status begin();

        /// Declared BEFORE `axis` so it outlives it: the axis holds a
        /// raw pointer to the brake (via `subscribe()`), so the brake
        /// must remain valid for the entire lifetime of the axis.
        /// Members destruct in reverse declaration order — `axis`
        /// goes first, then `brake`. Null when `useBrake == false`.
        std::unique_ptr<BrakeController> brake;
        std::unique_ptr<Axis> axis;

        ServoKitConfig storedCfg{};
};

/// Construct a YPMC + S2SVD15 servo kit. Returns `InvalidConfig` if
/// `stepPin` or `dirPin` is unset, or if `applyDriveDefaults()`
/// rejects the sensor-array population.
Result<std::unique_ptr<YpmcServoKit>> makeServoKit(const ServoKitConfig &cfg);

} // namespace ungula::motor::ypmc
