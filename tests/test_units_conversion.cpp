// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_motor_driver.h"
#include "ungula/motor/motor_axis.h"

using namespace ungula::motor;
using ungula::motor::tests::FakeMotorDriver;

namespace
{

MotorAxisConfig configWithFullUnits()
{
        MotorAxisConfig cfg;
        cfg.id = MotorAxisId{ 1 };
        cfg.units.stepsPerRevolution = 3200;
        cfg.units.stepsPerMm = 80.0f;
        cfg.units.stepsPerDegree = 3200.0f / 360.0f;
        cfg.limits.maxSpeed = Speed::stepsPerSec(800);
        cfg.limits.accel = Acceleration::stepsPerSecSquared(4000);
        cfg.limits.decel = Acceleration::stepsPerSecSquared(4000);
        cfg.limits.hardStepRateCeilingSps = 200000;
        return cfg;
}

} // namespace

TEST(UnitsConversion, DistanceRpmToStepsPerSecond)
{
        FakeMotorDriver drv;
        MotorAxis axis(configWithFullUnits(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // 60 RPM @ 3200 SPR → 3200 SPS.
        ASSERT_TRUE(axis.setSpeed(Speed::rpm(60)).ok());
        // Use a bounded move so MotorAxis goes through armMove(...) and
        // forwards both accel and decel rates to the driver.
        ASSERT_TRUE(axis.moveBy(DistanceValue::steps(200)).ok());
        EXPECT_EQ(drv.lastArmedCruiseSps, 3200u);

        // 120 RPM → 6400 SPS but clamped: hard ceiling is 200000 SPS so
        // 6400 lands unclamped.
        drv.simulateMotionComplete(0, StopReason::UserStop);
        axis.service(1);
        (void)axis.stop();
        drv.simulateMotionComplete(0, StopReason::UserStop);
        axis.service(2);

        ASSERT_TRUE(axis.setSpeed(Speed::rpm(120)).ok());
        // Use a bounded move so MotorAxis calls armMove(...) and forwards
        // both accel and decel rates to the driver.
        ASSERT_TRUE(axis.moveBy(DistanceValue::steps(200)).ok());
        EXPECT_EQ(drv.lastArmedCruiseSps, 6400u);
}

TEST(UnitsConversion, DistanceDegreesToSteps)
{
        FakeMotorDriver drv;
        MotorAxis axis(configWithFullUnits(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // 90° @ 8.888 steps/deg → 800 steps.
        ASSERT_TRUE(axis.moveBy(DistanceValue::degrees(90)).ok());
        EXPECT_EQ(drv.lastArmedTargetSteps, 800u);
        EXPECT_EQ(drv.lastArmedDirection, Direction::Forward);
}

TEST(UnitsConversion, DistanceRevolutionsToSteps)
{
        FakeMotorDriver drv;
        MotorAxis axis(configWithFullUnits(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // -2 rev → -6400 steps → Backward, 6400 magnitude.
        ASSERT_TRUE(axis.moveBy(DistanceValue::revolutions(-2)).ok());
        EXPECT_EQ(drv.lastArmedTargetSteps, 6400u);
        EXPECT_EQ(drv.lastArmedDirection, Direction::Backward);
}

TEST(UnitsConversion, PositionReadBackInMultipleUnits)
{
        FakeMotorDriver drv;
        MotorAxis axis(configWithFullUnits(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.moveBy(DistanceValue::mm(10)).ok());
        // 10 mm × 80 = 800 steps.
        drv.simulateMotionComplete(800, StopReason::TargetReached);
        axis.service(1);
        EXPECT_EQ(axis.state(), MotorState::Idle);

        EXPECT_EQ(axis.positionSteps(), 800);
        EXPECT_FLOAT_EQ(axis.position(DistanceUnit::Mm), 10.0f);
        EXPECT_FLOAT_EQ(axis.position(DistanceUnit::Cm), 1.0f);
        EXPECT_FLOAT_EQ(axis.position(DistanceUnit::Revolutions), 800.0f / 3200.0f);
}

TEST(UnitsConversion, IntentFlagComposition)
{
        constexpr auto combined = MotorIntent::Quiet | MotorIntent::AdaptiveCurrent;
        EXPECT_TRUE(has(combined, MotorIntent::Quiet));
        EXPECT_TRUE(has(combined, MotorIntent::AdaptiveCurrent));
        EXPECT_FALSE(has(combined, MotorIntent::HighTorque));
}

// --- New 1.0.4 units: cm/min for speed, ms ramp for acceleration --------

TEST(UnitsConversion, SpeedCmPerMinResolvesViaStepsPerMm)
{
        FakeMotorDriver drv;
        MotorAxis axis(configWithFullUnits(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // 60 cm/min @ 80 steps/mm
        //   = 600 mm/min = 10 mm/s = 800 steps/s.
        ASSERT_TRUE(axis.setSpeed(Speed::cmPerMin(60)).ok());
        // Use a bounded move so MotorAxis goes through armMove(...) and
        // forwards both accel and decel rates to the driver.
        ASSERT_TRUE(axis.moveBy(DistanceValue::steps(200)).ok());
        EXPECT_EQ(drv.lastArmedCruiseSps, 800u);
}

TEST(UnitsConversion, SpeedCmPerMinRejectsWhenStepsPerMmZero)
{
        auto cfg = configWithFullUnits();
        cfg.units.stepsPerMm = 0.0f;
        FakeMotorDriver drv;
        MotorAxis axis(cfg, drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        const auto r = axis.setSpeed(Speed::cmPerMin(60));
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(UnitsConversion, AccelRampMsResolvesAgainstCurrentCruise)
{
        auto cfg = configWithFullUnits();
        // Pin a clean cruise so the math is checkable by hand.
        cfg.limits.maxSpeed = Speed::stepsPerSec(10000);
        cfg.limits.accel = Acceleration::rampMs(500);
        cfg.limits.decel = Acceleration::rampMs(500);

        FakeMotorDriver drv;
        MotorAxis axis(cfg, drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // Ramp 0 → 10 000 SPS in 500 ms = 20 000 SPS².
        ASSERT_TRUE(axis.moveForward().ok());
        EXPECT_EQ(drv.lastArmedAccelSps2, 20000u);
}

TEST(UnitsConversion, AccelRampMsRescaledWhenSetSpeedChangesCruise)
{
        // A "fixed ramp time" should track changes to cruise — that's
        // the whole point of expressing accel as a duration.
        auto cfg = configWithFullUnits();
        cfg.limits.maxSpeed = Speed::stepsPerSec(10000);
        cfg.limits.accel = Acceleration::stepsPerSecSquared(0);
        cfg.limits.decel = Acceleration::stepsPerSecSquared(0);

        FakeMotorDriver drv;
        MotorAxis axis(cfg, drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // Move cruise to 20 000 SPS, then apply rampMs(500).
        ASSERT_TRUE(axis.setSpeed(Speed::stepsPerSec(20000)).ok());
        ASSERT_TRUE(axis.setAcceleration(Acceleration::rampMs(500)).ok());
        // Use a bounded move so MotorAxis goes through armMove(...) and
        // forwards both accel and decel rates to the driver.
        ASSERT_TRUE(axis.moveBy(DistanceValue::steps(200)).ok());
        // 20 000 SPS / 0.5 s = 40 000 SPS².
        EXPECT_EQ(drv.lastArmedAccelSps2, 40000u);
        EXPECT_EQ(drv.lastArmedDecelSps2, 40000u);
}

TEST(UnitsConversion, AccelRampMsRejectsZeroDuration)
{
        auto cfg = configWithFullUnits();
        cfg.limits.maxSpeed = Speed::stepsPerSec(10000);
        cfg.limits.accel = Acceleration::rampMs(0);
        cfg.limits.decel = Acceleration::stepsPerSecSquared(0);

        FakeMotorDriver drv;
        MotorAxis axis(cfg, drv);
        // Zero ramp duration is meaningless; begin() must refuse rather
        // than divide by zero or silently produce an infinite rate.
        const auto r = axis.begin();
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}
