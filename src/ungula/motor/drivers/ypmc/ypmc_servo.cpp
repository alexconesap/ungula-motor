// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/ypmc/ypmc_servo.h"

#include "ungula/core/time/time.h"
#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor::ypmc
{

namespace gpio = ungula::hal::gpio;
namespace time = ungula::core::time;

Status applyDriveDefaults(StepDirServoAxisConfig &cfg, const DriveTiming &timing,
                          const DrivePolarity &polarity)
{
        cfg.dirActiveHigh   = polarity.dirActiveHigh;
        cfg.enableActiveLow = !polarity.srvOnActiveHigh;
        cfg.dirSetupUs      = timing.dirSetupUs;

        cfg.common.limits.minPulseHighUs = timing.minPulseHighUs;
        cfg.common.limits.minPulseLowUs  = timing.minPulseLowUs;
        if (cfg.common.limits.maxStepRateSps == 0 ||
            cfg.common.limits.maxStepRateSps > timing.maxStepRateSps) {
                cfg.common.limits.maxStepRateSps = timing.maxStepRateSps;
        }

        // Wire ALM as a CrashLimit sensor when the caller has plumbed
        // the alarm input pin. COIN stays out of the SensorBank — hosts
        // that care about it poll `feedback().inPosition`.
        if (cfg.alarmInputPin.value != GPIO_NONE) {
                if (cfg.sensorCount >= MAX_SENSOR_INPUTS) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                auto &s    = cfg.sensors[cfg.sensorCount++];
                s.pin      = cfg.alarmInputPin.value;
                s.role     = SensorRole::CrashLimit;
                s.polarity = polarity.alarmActiveLow ? SensorPolarity::NormallyClosed :
                                                      SensorPolarity::NormallyOpen;
                s.direction = Direction::Forward; // alarm is direction-independent
        }

        return Status::Ok();
}

BrakeController::BrakeController(const Config &cfg)
        : cfg_(cfg)
{
}

void BrakeController::writeCoil(bool energised)
{
        if (cfg_.brakeReleasePin == GPIO_NONE)
                return;
        // energised == "brake released" — the coil is held by current.
        const bool level = cfg_.brakeReleaseActiveHigh ? energised : !energised;
        gpio::write(cfg_.brakeReleasePin, level);
}

Status BrakeController::begin()
{
        if (cfg_.brakeReleasePin == GPIO_NONE) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (!gpio::configOutput(cfg_.brakeReleasePin)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Seed engaged: at boot we don't know the axis state, and the
        // safe default for a holding brake is engaged.
        writeCoil(false);
        released_ = false;
        begun_    = true;
        return Status::Ok();
}

Status BrakeController::release()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (released_)
                return Status::Ok(); // idempotent

        writeCoil(true);
        if (cfg_.releaseSettleMs > 0) {
                time::delayMs(static_cast<int64_t>(cfg_.releaseSettleMs));
        }
        released_ = true;
        return Status::Ok();
}

Status BrakeController::engage()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!released_)
                return Status::Ok(); // already engaged

        if (cfg_.engageSettleMs > 0) {
                time::delayMs(static_cast<int64_t>(cfg_.engageSettleMs));
        }
        writeCoil(false);
        released_ = false;
        return Status::Ok();
}

void BrakeController::onAxisEvent(const AxisEvent &ev)
{
        if (!begun_)
                return;

        switch (ev.type) {
        case AxisEventType::MotionCompleted:
        case AxisEventType::MotionStopped:
                if (cfg_.autoEngageOnMotionEnd) {
                        (void)engage();
                }
                break;

        case AxisEventType::EmergencyStopped:
        case AxisEventType::FaultRaised:
                // Always engage on a fault path, regardless of the
                // auto-engage flag. The motor is no longer under
                // controlled deceleration; the brake catches it.
                (void)engage();
                break;

        default:
                break;
        }
}

} // namespace ungula::motor::ypmc
