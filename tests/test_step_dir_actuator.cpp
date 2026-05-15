// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/actuator/actuator_capabilities.h"
#include "ungula/motor/actuator/step_dir_actuator.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/planning/planned_move.h"
#include "ungula/motor/pulse/fake_pulse_engine.h"
#include "ungula/motor/pulse/i_pulse_engine.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

PlannedMove makeMove(Direction dir, uint32_t steps)
{
        PlannedMove m;
        m.direction = dir;
        m.totalSteps = steps;
        m.segmentCount = 1;
        m.segments[0].stepCount = steps;
        m.segments[0].halfPeriodTicks = 100;
        return m;
}

StepDirActuator::Config makeCfg()
{
        StepDirActuator::Config c;
        c.kind = StepDirActuatorKind::OpenLoopStepper;
        c.enablePin = GPIO_NONE; // no real GPIO in host tests
        c.enableActiveLow = true;
        return c;
}

TEST(StepDirActuatorTest, BeginEnableDisableHonourOrder)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());

        // Cannot enable before begin.
        EXPECT_EQ(actuator.enable().error(), ErrorCode::NotInitialized);
        EXPECT_EQ(actuator.disable().error(), ErrorCode::NotInitialized);

        EXPECT_TRUE(actuator.begin().ok());
        EXPECT_EQ(engine.beginCallCount, 1u);

        // Double-begin is rejected.
        EXPECT_EQ(actuator.begin().error(), ErrorCode::AlreadyInitialized);

        EXPECT_TRUE(actuator.enable().ok());
        EXPECT_TRUE(actuator.disable().ok());
}

TEST(StepDirActuatorTest, MotionRequiresEnable)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());

        // armMotion / startMotion before enable should fail with NotEnabled.
        EXPECT_EQ(actuator.armMotion(makeMove(Direction::Forward, 10)).error(),
                  ErrorCode::NotEnabled);
        EXPECT_EQ(actuator.startMotion().error(), ErrorCode::NotEnabled);

        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 10)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());
        EXPECT_TRUE(engine.isRunning());
}

TEST(StepDirActuatorTest, StopForwardsMode)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());

        EXPECT_TRUE(actuator.stop(StopMode::Immediate).ok());
        EXPECT_FALSE(engine.isRunning());
        EXPECT_EQ(engine.stopCallCount, 1u);
}

TEST(StepDirActuatorTest, StopDecelerateReturnsUnsupported)
{
        // The whole point: the actuator must not silently downgrade
        // Decelerate to Immediate. Motion-control safety contract.
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());

        EXPECT_EQ(actuator.stop(StopMode::Decelerate).error(), ErrorCode::Unsupported);
}

TEST(StepDirActuatorTest, EmergencyStopLatchesFault)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());

        EXPECT_TRUE(actuator.emergencyStop().ok());

        const auto fs = actuator.faultStatus();
        EXPECT_TRUE(fs.active());
        EXPECT_EQ(fs.code, FaultCode::EmergencyStop);

        // ClearFault should restore a healthy state.
        EXPECT_TRUE(actuator.clearFault().ok());
        EXPECT_EQ(engine.clearFaultCallCount, 1u);
        EXPECT_FALSE(actuator.faultStatus().active());
}

TEST(StepDirActuatorTest, DisableStopsMotionFirst)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());
        ASSERT_TRUE(engine.isRunning());

        EXPECT_TRUE(actuator.disable().ok());
        EXPECT_FALSE(engine.isRunning());
        // disable should have invoked stop() exactly once en route.
        EXPECT_EQ(engine.stopCallCount, 1u);
}

TEST(StepDirActuatorTest, CapabilitiesReflectConfig)
{
        FakePulseEngine engine;

        StepDirActuator::Config c = makeCfg();
        c.enablePin = 0; // any non-GPIO_NONE value
        c.hasAlarmInput = true;
        c.hasInPositionInput = false;

        StepDirActuator actuator(engine, c);
        const auto cap = actuator.capabilities();

        EXPECT_TRUE(cap.hasEnablePin);
        EXPECT_TRUE(cap.hasDirectionPin);
        EXPECT_TRUE(cap.hasPulseEngine);
        EXPECT_FALSE(cap.hasActualPosition); // open-loop: no encoder
        EXPECT_TRUE(cap.hasAlarmInput);
        EXPECT_FALSE(cap.hasInPositionInput);
        EXPECT_FALSE(cap.hasNativeHoming);
}

TEST(StepDirActuatorTest, FeedbackTracksEngine)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 50)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());

        engine.tickSteps(20);
        auto fb = actuator.feedback();
        EXPECT_EQ(fb.commandedPosition, 20);
        EXPECT_FALSE(fb.hasActualPosition); // open-loop reports no truth
        EXPECT_FALSE(fb.inPosition); // still running

        engine.tickSteps(30); // completes the move
        fb = actuator.feedback();
        EXPECT_EQ(fb.commandedPosition, 50);
        EXPECT_TRUE(fb.inPosition);
}

TEST(StepDirActuatorTest, InjectedEngineFaultSurfaces)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());

        engine.injectFault();
        const auto fs = actuator.faultStatus();
        EXPECT_TRUE(fs.active());
        // Engine reports StopReason::DriverFault from injectFault; the
        // actuator maps that ISR-side timer fault to PulseEngineFault at
        // the Axis layer (driver-fault is reserved for stepper-driver
        // faults like TMC2209 alarms).
        EXPECT_EQ(fs.code, FaultCode::PulseEngineFault);
}

} // namespace
