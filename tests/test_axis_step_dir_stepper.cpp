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

TEST(AxisStepDirStepperTest, SafetyInterlockReportsHeldEstop)
{
        // Host gpio stub reads LOW for every pin. A NormallyClosed
        // E-stop interprets LOW as "asserted" (the polarity model in
        // the codebase). The opt-in `isSafetyInterlockActive()` query
        // must report this without blocking `enable()` — the host
        // chooses whether to gate on the query depending on wiring.
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        cfg.sensors[0].pin = 22;
        cfg.sensors[0].role = SensorRole::EmergencyStop;
        cfg.sensors[0].polarity = SensorPolarity::NormallyClosed;
        cfg.sensorCount = 1;

        auto r = Axis::createStepDirStepper(cfg);
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        EXPECT_TRUE(axis->isSafetyInterlockActive());
        // enable() does NOT auto-gate — verifies the regression where
        // an inverted-polarity wiring would block boot. Hosts that
        // want the gate call the interlock themselves.
        EXPECT_TRUE(axis->enable().ok());
        EXPECT_EQ(axis->state(), AxisState::Idle);
}

TEST(AxisStepDirStepperTest, SafetyInterlockReportsHeldCrashLimit)
{
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        cfg.sensors[0].pin = 25;
        cfg.sensors[0].role = SensorRole::CrashLimit;
        cfg.sensors[0].polarity = SensorPolarity::NormallyClosed;
        cfg.sensorCount = 1;

        auto r = Axis::createStepDirStepper(cfg);
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        EXPECT_TRUE(axis->isSafetyInterlockActive());
}

TEST(AxisStepDirStepperTest, SafetyInterlockSilentWhenNoIsrSafetySensors)
{
        // No E-stop / crash-limit configured → interlock query is
        // always false. enable() succeeds.
        auto r = Axis::createStepDirStepper(makeStepperCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());
        EXPECT_FALSE(axis->isSafetyInterlockActive());
        EXPECT_TRUE(axis->enable().ok());
        EXPECT_EQ(axis->state(), AxisState::Idle);
}

// =====================================================================
// E-stop precedence — audit P0-5
// =====================================================================
// Previously the three `consume*Activation` branches in `pumpSensors`
// were independent `if`s, so a stall or crash latch landing in the
// same tick as an E-stop would overwrite `state_`. The fix consumes
// all three latches but only the highest-precedence one writes state.

TEST(AxisStepDirStepperTest, EstopWinsOverCrashAndStallInSameTick)
{
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        // Three ISR-role sensors covering all the relevant precedences.
        cfg.sensors[0].pin = 33;
        cfg.sensors[0].role = SensorRole::EmergencyStop;
        cfg.sensors[0].polarity = SensorPolarity::NormallyClosed;
        cfg.sensors[1].pin = 27;
        cfg.sensors[1].role = SensorRole::CrashLimit;
        cfg.sensors[1].polarity = SensorPolarity::NormallyClosed;
        cfg.sensors[2].pin = 26;
        cfg.sensors[2].role = SensorRole::Stall;
        cfg.sensors[2].polarity = SensorPolarity::NormallyOpen;
        cfg.sensors[2].stallHitsToTrigger = 1; // halt on first edge
        cfg.sensors[2].stallArmDelayMs = 0;
        cfg.sensorCount = 3;

        auto r = Axis::createStepDirStepper(cfg);
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        // All three latches fire in the same window before service().
        axis->simulateSensorIsrForTesting(SensorRole::EmergencyStop);
        axis->simulateSensorIsrForTesting(SensorRole::CrashLimit);
        axis->simulateSensorIsrForTesting(SensorRole::Stall);

        axis->service(100);

        // E-stop dominates: state and stopReason reflect EmergencyStop
        // even though all three latches were set.
        EXPECT_EQ(axis->state(), AxisState::EmergencyStopped);
        EXPECT_EQ(axis->lastStopReason(), StopReason::EmergencyStop);
}

TEST(AxisStepDirStepperTest, CrashWinsOverStallInSameTick)
{
        // Same precedence test, without an E-stop: crash dominates
        // over stall.
        StepDirStepperAxisConfig cfg = makeStepperCfg();
        cfg.sensors[0].pin = 27;
        cfg.sensors[0].role = SensorRole::CrashLimit;
        cfg.sensors[0].polarity = SensorPolarity::NormallyClosed;
        cfg.sensors[1].pin = 26;
        cfg.sensors[1].role = SensorRole::Stall;
        cfg.sensors[1].polarity = SensorPolarity::NormallyOpen;
        cfg.sensors[1].stallHitsToTrigger = 1;
        cfg.sensors[1].stallArmDelayMs = 0;
        cfg.sensorCount = 2;

        auto r = Axis::createStepDirStepper(cfg);
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        axis->simulateSensorIsrForTesting(SensorRole::CrashLimit);
        axis->simulateSensorIsrForTesting(SensorRole::Stall);

        axis->service(100);

        EXPECT_EQ(axis->state(), AxisState::Faulted);
        EXPECT_EQ(axis->lastStopReason(), StopReason::LimitSwitch);
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
