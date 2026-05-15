// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <memory>

#include "ungula/motor/drivers/ypmc/ypmc_kit.h"

namespace
{

using namespace ungula::motor;
using namespace ungula::motor::ypmc;

ServoKitConfig makeBaseCfg()
{
        ServoKitConfig c;
        c.common.axisId = AxisId(7);
        c.common.name = "ypmc";
        c.common.units.stepsPerMm = 1000.0f;
        c.common.limits.maxVelocitySps = 8000;
        c.common.limits.accelSpsPerSec = 16000;
        c.common.limits.decelSpsPerSec = 16000;
        c.common.limits.maxStepRateSps = 200000;

        c.stepPin = StepPin{ 12 };
        c.dirPin = DirectionPin{ 13 };
        c.enablePin = EnablePin{ 14 };
        return c;
}

TEST(YpmcKitTest, FactoryRejectsMissingStepPin)
{
        auto cfg = makeBaseCfg();
        cfg.stepPin = StepPin{};
        auto r = makeServoKit(cfg);
        ASSERT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(YpmcKitTest, ValidConfigProducesAxis)
{
        auto cfg = makeBaseCfg();
        auto r = makeServoKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        ASSERT_NE(kit, nullptr);
        EXPECT_NE(kit->axis, nullptr);
        EXPECT_EQ(kit->brake, nullptr); // useBrake defaults to false
}

TEST(YpmcKitTest, OptionalBrakeControllerConstructed)
{
        auto cfg = makeBaseCfg();
        cfg.useBrake = true;
        cfg.brake.brakeReleasePin = 27;
        auto r = makeServoKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        EXPECT_NE(kit->brake, nullptr);
}

TEST(YpmcKitTest, TandemFieldsPropagateToStoredCfg)
{
        // Two YPMCs on one STEP pin, face-to-face mounted: secondary
        // DIR inverted so opposite electrical direction → same physical
        // rotation.
        auto cfg = makeBaseCfg();
        cfg.secondaryDirPin = DirectionPin{ 33 };
        cfg.secondaryDirInverted = true;
        cfg.secondaryEnablePin = EnablePin{ 32 };
        cfg.secondaryEnableActiveLow = false; // SRV-ON style
        auto r = makeServoKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        EXPECT_EQ(kit->storedCfg.secondaryDirPin.value, 33);
        EXPECT_TRUE(kit->storedCfg.secondaryDirInverted);
        EXPECT_EQ(kit->storedCfg.secondaryEnablePin.value, 32);
        EXPECT_FALSE(kit->storedCfg.secondaryEnableActiveLow);
}

} // namespace
