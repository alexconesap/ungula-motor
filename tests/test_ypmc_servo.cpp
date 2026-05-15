// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/axis_config.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/drivers/ypmc/ypmc_servo.h"
#include "ungula/motor/events/axis_event.h"
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

StepDirServoAxisConfig makeBaseCfg()
{
        StepDirServoAxisConfig cfg;
        cfg.common.axisId = AxisId(0);
        cfg.common.units.stepsPerMm = 1000.0f;
        cfg.common.limits.maxVelocitySps = 100'000;
        cfg.common.limits.accelSpsPerSec = 400'000;
        cfg.common.limits.decelSpsPerSec = 400'000;
        cfg.common.limits.maxStepRateSps = 0; // forces applyDriveDefaults to fill
        cfg.stepPin = StepPin{ 18 };
        cfg.dirPin  = DirectionPin{ 19 };
        return cfg;
}

TEST(YpmcServoTest, ApplyDriveDefaultsPopulatesTimingAndPolarity)
{
        auto cfg = makeBaseCfg();
        cfg.enablePin = EnablePin{ 21 };

        ASSERT_TRUE(ypmc::applyDriveDefaults(cfg).ok());

        EXPECT_EQ(cfg.dirSetupUs, ypmc::kDefaultDriveTiming.dirSetupUs);
        EXPECT_EQ(cfg.common.limits.minPulseHighUs, ypmc::kDefaultDriveTiming.minPulseHighUs);
        EXPECT_EQ(cfg.common.limits.minPulseLowUs, ypmc::kDefaultDriveTiming.minPulseLowUs);
        EXPECT_EQ(cfg.common.limits.maxStepRateSps, ypmc::kDefaultDriveTiming.maxStepRateSps);

        // SRV-ON is active-HIGH on the S2SVD15 → enableActiveLow=false.
        EXPECT_FALSE(cfg.enableActiveLow);
        EXPECT_TRUE(cfg.dirActiveHigh);
}

TEST(YpmcServoTest, ApplyDriveDefaultsClampsMaxRateButPreservesLower)
{
        // User supplied a SLOWER max rate — defaults must not raise it.
        auto cfg = makeBaseCfg();
        cfg.common.limits.maxStepRateSps = 100'000;

        ASSERT_TRUE(ypmc::applyDriveDefaults(cfg).ok());

        EXPECT_EQ(cfg.common.limits.maxStepRateSps, 100'000u);
}

TEST(YpmcServoTest, ApplyDriveDefaultsWiresAlarmAsCrashLimit)
{
        auto cfg = makeBaseCfg();
        cfg.alarmInputPin = InputPin{ 34 };

        ASSERT_TRUE(ypmc::applyDriveDefaults(cfg).ok());

        ASSERT_EQ(cfg.sensorCount, 1);
        EXPECT_EQ(cfg.sensors[0].pin, 34);
        EXPECT_EQ(cfg.sensors[0].role, SensorRole::CrashLimit);
        EXPECT_EQ(cfg.sensors[0].polarity, SensorPolarity::NormallyClosed);
}

TEST(YpmcServoTest, ApplyDriveDefaultsSkipsAlarmWhenPinUnset)
{
        auto cfg = makeBaseCfg();
        // alarmInputPin left at GPIO_NONE.

        ASSERT_TRUE(ypmc::applyDriveDefaults(cfg).ok());

        EXPECT_EQ(cfg.sensorCount, 0);
}

TEST(YpmcServoTest, ApplyDriveDefaultsRejectsFullSensorTable)
{
        auto cfg = makeBaseCfg();
        cfg.alarmInputPin = InputPin{ 34 };
        // Fill the sensor slots so applyDriveDefaults can't append.
        for (uint8_t i = 0; i < MAX_SENSOR_INPUTS; ++i) {
                cfg.sensors[i].pin  = static_cast<uint8_t>(40 + i);
                cfg.sensors[i].role = SensorRole::Home;
        }
        cfg.sensorCount = MAX_SENSOR_INPUTS;

        const auto s = ypmc::applyDriveDefaults(cfg);
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
}

TEST(YpmcServoTest, BrakeControllerBeginRejectsUnsetPin)
{
        ypmc::BrakeController::Config bcfg;
        bcfg.brakeReleasePin = GPIO_NONE;
        ypmc::BrakeController brake(bcfg);

        EXPECT_EQ(brake.begin().error(), ErrorCode::InvalidConfig);
        EXPECT_FALSE(brake.isReleased());
}

TEST(YpmcServoTest, BrakeControllerReleaseEngageStateTransitions)
{
        ypmc::BrakeController::Config bcfg;
        bcfg.brakeReleasePin = 25;
        bcfg.releaseSettleMs = 0; // keep tests fast — settle delay is functional, not behavioral
        bcfg.engageSettleMs  = 0;
        ypmc::BrakeController brake(bcfg);

        // release before begin is rejected.
        EXPECT_EQ(brake.release().error(), ErrorCode::NotInitialized);

        ASSERT_TRUE(brake.begin().ok());
        EXPECT_FALSE(brake.isReleased()); // boot-default = engaged

        ASSERT_TRUE(brake.release().ok());
        EXPECT_TRUE(brake.isReleased());

        // Idempotent.
        ASSERT_TRUE(brake.release().ok());
        EXPECT_TRUE(brake.isReleased());

        ASSERT_TRUE(brake.engage().ok());
        EXPECT_FALSE(brake.isReleased());

        // Idempotent the other way too.
        ASSERT_TRUE(brake.engage().ok());
        EXPECT_FALSE(brake.isReleased());
}

TEST(YpmcServoTest, BrakeControllerAutoEngagesOnMotionCompleted)
{
        ypmc::BrakeController::Config bcfg;
        bcfg.brakeReleasePin = 25;
        bcfg.releaseSettleMs = 0;
        bcfg.engageSettleMs  = 0;
        bcfg.autoEngageOnMotionEnd = true;
        ypmc::BrakeController brake(bcfg);
        ASSERT_TRUE(brake.begin().ok());
        ASSERT_TRUE(brake.release().ok());
        ASSERT_TRUE(brake.isReleased());

        AxisEvent ev;
        ev.type = AxisEventType::MotionCompleted;
        brake.onAxisEvent(ev);
        EXPECT_FALSE(brake.isReleased());
}

TEST(YpmcServoTest, BrakeControllerAutoEngageDisableable)
{
        ypmc::BrakeController::Config bcfg;
        bcfg.brakeReleasePin = 25;
        bcfg.releaseSettleMs = 0;
        bcfg.engageSettleMs  = 0;
        bcfg.autoEngageOnMotionEnd = false;
        ypmc::BrakeController brake(bcfg);
        ASSERT_TRUE(brake.begin().ok());
        ASSERT_TRUE(brake.release().ok());

        AxisEvent ev;
        ev.type = AxisEventType::MotionCompleted;
        brake.onAxisEvent(ev);
        // Horizontal-axis flow: motor stays free between moves.
        EXPECT_TRUE(brake.isReleased());

        // Faults still force engage regardless of the flag.
        ev.type = AxisEventType::FaultRaised;
        brake.onAxisEvent(ev);
        EXPECT_FALSE(brake.isReleased());
}

TEST(YpmcServoTest, BrakeControllerEngagesOnEmergencyStop)
{
        ypmc::BrakeController::Config bcfg;
        bcfg.brakeReleasePin = 25;
        bcfg.releaseSettleMs = 0;
        bcfg.engageSettleMs  = 0;
        bcfg.autoEngageOnMotionEnd = false;
        ypmc::BrakeController brake(bcfg);
        ASSERT_TRUE(brake.begin().ok());
        ASSERT_TRUE(brake.release().ok());

        AxisEvent ev;
        ev.type = AxisEventType::EmergencyStopped;
        brake.onAxisEvent(ev);
        EXPECT_FALSE(brake.isReleased());
}

TEST(YpmcServoTest, BrakeControllerIgnoresIrrelevantEvents)
{
        ypmc::BrakeController::Config bcfg;
        bcfg.brakeReleasePin = 25;
        bcfg.releaseSettleMs = 0;
        bcfg.engageSettleMs  = 0;
        ypmc::BrakeController brake(bcfg);
        ASSERT_TRUE(brake.begin().ok());
        ASSERT_TRUE(brake.release().ok());

        AxisEvent ev;
        ev.type = AxisEventType::StateChanged;
        brake.onAxisEvent(ev);
        EXPECT_TRUE(brake.isReleased());

        ev.type = AxisEventType::MotionStarted;
        brake.onAxisEvent(ev);
        EXPECT_TRUE(brake.isReleased());
}

} // namespace
