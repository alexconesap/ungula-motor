// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/ypmc/ypmc_kit.h"

#include "ungula/motor/limits/sensor_input.h" // GPIO_NONE

namespace ungula::motor::ypmc
{

namespace
{

bool pinSet(uint8_t v)
{
        return v != GPIO_NONE;
}

Status buildAxisConfig(const ServoKitConfig &cfg, StepDirServoAxisConfig &out)
{
        out = StepDirServoAxisConfig{};
        out.common = cfg.common;
        out.stepPin = cfg.stepPin;
        out.dirPin = cfg.dirPin;
        out.enablePin = cfg.enablePin;
        out.alarmInputPin = cfg.alarmInputPin;
        out.inPositionInputPin = cfg.inPositionInputPin;
        out.secondaryDirPin = cfg.secondaryDirPin;
        out.secondaryDirActiveHigh = cfg.secondaryDirActiveHigh;
        out.secondaryDirInverted = cfg.secondaryDirInverted;
        out.secondaryEnablePin = cfg.secondaryEnablePin;
        out.secondaryEnableActiveLow = cfg.secondaryEnableActiveLow;
        for (uint8_t i = 0; i < cfg.sensorCount && i < MAX_SENSOR_INPUTS; ++i) {
                out.sensors[i] = cfg.sensors[i];
        }
        out.sensorCount = cfg.sensorCount;

        // applyDriveDefaults() fills timing (dirSetupUs), polarity
        // (srvOn / alarm / inPosition), and routes ALM into the
        // sensor array if `alarmInputPin` is set. Propagate any
        // sensor-array-overflow error verbatim.
        return applyDriveDefaults(out, cfg.timing, cfg.polarity);
}

} // namespace

Status YpmcServoKit::begin()
{
        if (brake) {
                const auto s = brake->begin();
                if (!s.ok())
                        return s;
        }
        if (!axis) {
                return Status::Err(ErrorCode::InternalError);
        }
        const auto s = axis->begin();
        if (!s.ok() && s.error() != ErrorCode::AlreadyInitialized) {
                return s;
        }
        if (brake) {
                // Listener auto-engages on MotionCompleted / fault.
                // `subscribe()` returns Ok() / InvalidConfig — propagate.
                const auto sub = axis->subscribe(brake.get());
                if (!sub.ok())
                        return sub;
        }
        return Status::Ok();
}

Result<std::unique_ptr<YpmcServoKit>> makeServoKit(const ServoKitConfig &cfg)
{
        if (!pinSet(cfg.stepPin.value) || !pinSet(cfg.dirPin.value)) {
                return Result<std::unique_ptr<YpmcServoKit>>::Err(ErrorCode::InvalidConfig);
        }

        StepDirServoAxisConfig axisCfg{};
        if (const auto bs = buildAxisConfig(cfg, axisCfg); !bs.ok()) {
                return Result<std::unique_ptr<YpmcServoKit>>::Err(bs.error());
        }
        // S2SVD15 exposes no identity register over plain STEP/DIR, so
        // the kit answers with hardcoded compile-time identity. String
        // literals have static lifetime — safe for the
        // `StaticDriverIdentity` to hold by pointer.
        auto identity = std::make_unique<StaticDriverIdentity>(
            "RATTMOTOR", "YPMC + S2SVD15", /*firmwareMajor=*/0,
            /*firmwareMinor=*/0, /*rawId=*/0);
        axisCfg.identityProvider = identity.get();

        auto axisRes = Axis::createStepDirServo(axisCfg);
        if (!axisRes.ok()) {
                return Result<std::unique_ptr<YpmcServoKit>>::Err(axisRes.error());
        }

        std::unique_ptr<BrakeController> brake;
        if (cfg.useBrake) {
                brake = std::make_unique<BrakeController>(cfg.brake);
        }

        auto kit = std::make_unique<YpmcServoKit>();
        kit->axis = axisRes.takeValue();
        kit->brake = std::move(brake);
        kit->identity = std::move(identity);
        kit->storedCfg = cfg;
        return Result<std::unique_ptr<YpmcServoKit>>::Ok(std::move(kit));
}

} // namespace ungula::motor::ypmc
