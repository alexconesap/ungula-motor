// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/ypmc/ypmc_driver.h"

#include "ungula/core/time/time.h"
#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor::ypmc
{

namespace gpio = ungula::hal::gpio;

YpmcStepDirDriver::YpmcStepDirDriver(YpmcConfig cfg, IStepSignalGenerator &stepSignal,
                                     IMotionPlanner &planner)
        : GenericStepDirDriver(cfg.generic, stepSignal, planner)
        , ypmcCfg_(cfg)
{
}

YpmcStepDirDriver::YpmcStepDirDriver(YpmcConfig cfg)
        : GenericStepDirDriver(cfg.generic)
        , ypmcCfg_(cfg)
{
}

Status YpmcStepDirDriver::begin()
{
        const auto s = GenericStepDirDriver::begin();
        if (!s.ok()) {
                return s;
        }

        if (ypmcCfg_.secondaryDirPin != GPIO_NONE) {
                if (!gpio::configOutput(ypmcCfg_.secondaryDirPin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                // Idle level — opposite of the "forward" level so a
                // STEP edge can't be mis-interpreted at boot.
                gpio::write(ypmcCfg_.secondaryDirPin,
                            ypmcCfg_.secondaryDirActiveHigh ? false : true);
        }

        if (ypmcCfg_.brakePin != GPIO_NONE) {
                if (!gpio::configOutput(ypmcCfg_.brakePin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                // Brake engaged at boot — safe default.
                writeBrake(false);
        }
        return Status::Ok();
}

Status YpmcStepDirDriver::enable()
{
        return GenericStepDirDriver::enable();
}

Status YpmcStepDirDriver::disable()
{
        // Engage the brake before de-energising the drive — otherwise
        // a vertical load could drop the moment SRV-ON releases.
        engageBrake();
        return GenericStepDirDriver::disable();
}

Status YpmcStepDirDriver::stop(StopMode mode)
{
        const auto s = GenericStepDirDriver::stop(mode);
        if (mode == StopMode::Immediate) {
                // Emergency / fault path — force-engage the brake
                // regardless of `autoEngageOnMotionEnd`.
                engageBrake();
        }
        return s;
}

DriverIdentity YpmcStepDirDriver::identity()
{
        DriverIdentity id;
        id.vendor = "RATTMOTOR";
        id.model = "YPMC + S2SVD15";
        id.firmwareMajor = 1;
        id.firmwareMinor = 0;
        id.rawId = 0;
        return id;
}

// =====================================================================
// Subclass hooks
// =====================================================================

void YpmcStepDirDriver::onBeforeArm(Direction dir)
{
        // Release the brake BEFORE the move arms — the generator's
        // armMove writes the primary DIR + sleeps `dirSetupUs`, and
        // we want the brake mechanically clear by the time the first
        // STEP edge fires. The settle delay is a busy-wait in task
        // context (same idiom as the generator's dirSetupUs).
        releaseBrake();

        // Write the secondary DIR BEFORE the generator's armMove so
        // both DIRs are stable during the dirSetupUs window the
        // generator uses for the primary DIR.
        writeSecondaryDir(dir);
}

void YpmcStepDirDriver::onMotionStarted()
{
        // Nothing extra — release happened in onBeforeArm so the
        // brake is already clear when the first edge fires.
}

void YpmcStepDirDriver::onMotionStopped()
{
        if (ypmcCfg_.autoEngageOnMotionEnd) {
                engageBrake();
        }
}

// =====================================================================
// Helpers
// =====================================================================

void YpmcStepDirDriver::writeSecondaryDir(Direction dir)
{
        if (ypmcCfg_.secondaryDirPin == GPIO_NONE) {
                return;
        }
        // Map host direction to "logical forward / backward" for the
        // secondary motor:
        //   secondaryDirInverted = false → same logical direction.
        //   secondaryDirInverted = true  → opposite logical direction
        //                                  (face-to-face mounting).
        const bool wantForward = (dir == Direction::Forward) != ypmcCfg_.secondaryDirInverted;
        const bool level = wantForward ? ypmcCfg_.secondaryDirActiveHigh :
                                         !ypmcCfg_.secondaryDirActiveHigh;
        gpio::write(ypmcCfg_.secondaryDirPin, level);
}

void YpmcStepDirDriver::writeBrake(bool released)
{
        if (ypmcCfg_.brakePin == GPIO_NONE) {
                return;
        }
        const bool level = released ? ypmcCfg_.brakeReleaseActiveHigh :
                                      !ypmcCfg_.brakeReleaseActiveHigh;
        gpio::write(ypmcCfg_.brakePin, level);
}

void YpmcStepDirDriver::releaseBrake()
{
        if (ypmcCfg_.brakePin == GPIO_NONE || brakeReleased_) {
                return;
        }
        writeBrake(true);
        if (ypmcCfg_.brakeReleaseSettleMs > 0) {
                ungula::core::time::delayMs(static_cast<int64_t>(ypmcCfg_.brakeReleaseSettleMs));
        }
        brakeReleased_ = true;
}

void YpmcStepDirDriver::engageBrake()
{
        if (ypmcCfg_.brakePin == GPIO_NONE || !brakeReleased_) {
                return;
        }
        if (ypmcCfg_.brakeEngageSettleMs > 0) {
                ungula::core::time::delayMs(static_cast<int64_t>(ypmcCfg_.brakeEngageSettleMs));
        }
        writeBrake(false);
        brakeReleased_ = false;
}

} // namespace ungula::motor::ypmc
