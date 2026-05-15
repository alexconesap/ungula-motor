// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "ungula/motor/axis.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

StepDirStepperAxisConfig makeStepperCfg()
{
        StepDirStepperAxisConfig cfg;
        cfg.common.axisId = AxisId(1);
        cfg.common.name = "x";
        cfg.common.units.stepsPerMm = 80.0f;
        cfg.common.limits.maxVelocitySps = 4000;
        cfg.common.limits.accelSpsPerSec = 8000;
        cfg.common.limits.decelSpsPerSec = 8000;
        cfg.common.limits.minPulseHighUs = 2;
        cfg.common.limits.minPulseLowUs = 2;
        cfg.common.limits.maxStepRateSps = 200'000;
        cfg.stepPin = StepPin{ 18 };
        cfg.dirPin = DirectionPin{ 19 };
        cfg.enablePin = EnablePin{ GPIO_NONE };
        cfg.dirActiveHigh = true;
        cfg.enableActiveLow = true;
        cfg.dirSetupUs = 5;
        return cfg;
}

TEST(AxisStepDirStepperTest, FactoryRejectsMissingStepPin)
{
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        cfg.stepPin = StepPin{ GPIO_NONE };
        auto r = Axis::createStepDirStepper(cfg);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(AxisStepDirStepperTest, FactoryRejectsZeroAcceleration)
{
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        cfg.common.limits.accelSpsPerSec = 0;
        auto r = Axis::createStepDirStepper(cfg);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(AxisStepDirStepperTest, FactoryBuildsValidAxis)
{
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        std::unique_ptr<Axis> axis = r.takeValue();
        ASSERT_NE(axis.get(), nullptr);
        EXPECT_EQ(axis->state(), AxisState::Uninitialized);
        EXPECT_EQ(axis->id().value, 1u);
}

TEST(AxisStepDirStepperTest, BeginEnableDisableTransitions)
{
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();

        ASSERT_TRUE(axis->begin().ok());
        EXPECT_EQ(axis->state(), AxisState::Disabled);

        ASSERT_TRUE(axis->enable().ok());
        EXPECT_EQ(axis->state(), AxisState::Idle);

        ASSERT_TRUE(axis->disable().ok());
        EXPECT_EQ(axis->state(), AxisState::Disabled);
}

TEST(AxisStepDirStepperTest, MotionRequiresEnable)
{
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        EXPECT_EQ(axis->moveBy(100).error(), ErrorCode::NotEnabled);
        EXPECT_EQ(axis->moveTo(50).error(), ErrorCode::NotEnabled);
        EXPECT_EQ(axis->jog(Direction::Forward).error(), ErrorCode::NotEnabled);
}

TEST(AxisStepDirStepperTest, StopDecelerateReturnsUnsupported)
{
        // Contract: Axis does NOT silently downgrade Decelerate to
        // Immediate. The pulse engine can't re-plan in-flight yet, so
        // we surface Unsupported all the way to the caller.
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        EXPECT_EQ(axis->stop(StopMode::Decelerate).error(), ErrorCode::Unsupported);
}

TEST(AxisStepDirStepperTest, HomeReturnsUnsupportedWithoutStrategy)
{
        // `home()` actually runs the controller when a strategy is
        // configured. Without one, the controller returns Unsupported
        // and the axis stays in Idle.
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());
        ASSERT_TRUE(axis->enable().ok());

        EXPECT_EQ(axis->home().error(), ErrorCode::Unsupported);
        EXPECT_FALSE(axis->isHoming());
}

TEST(AxisStepDirStepperTest, UnitConversionRejectsMissingScaling)
{
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        cfg.common.units.stepsPerMm = 0.0f;
        cfg.common.units.stepsPerDegree = 0.0f;
        auto r = Axis::createStepDirStepper(cfg);
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());
        ASSERT_TRUE(axis->enable().ok());

        EXPECT_EQ(axis->moveBy(10, DistanceUnit::Mm).error(), ErrorCode::InvalidConfig);
        EXPECT_EQ(axis->moveBy(10, DistanceUnit::Degrees).error(), ErrorCode::InvalidConfig);
}

TEST(AxisStepDirStepperTest, CreateCanServoIsUnsupportedInPhaseD)
{
        CanServoAxisConfig cfg;
        cfg.common.axisId = AxisId(2);
        auto r = Axis::createCanServo(cfg);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::Unsupported);
}

TEST(AxisStepDirStepperTest, OwnershipDestructsCleanly)
{
        // No explicit assertions — this test exists so the address
        // sanitiser / leak detector can run over the full
        // factory → begin → enable → disable → destruct path.
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());
        ASSERT_TRUE(axis->enable().ok());
        ASSERT_TRUE(axis->disable().ok());
        // axis destructs here; unique_ptrs release in reverse declaration
        // order: actuator → engine → timer.
}

} // namespace
