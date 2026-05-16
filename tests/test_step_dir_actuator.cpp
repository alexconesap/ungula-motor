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

TEST(StepDirActuatorTest, LimitSwitchFaultMapsToLimitExceeded)
{
        // The engine latches StopReason::LimitSwitch when a crash-limit
        // ISR halts it. The Axis event path emits FaultCode::LimitExceeded
        // for that reason; `faultStatus()` must agree — otherwise a
        // listener and a status query see different fault codes for
        // the same physical event.
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());

        engine.haltFromIsr(StopReason::LimitSwitch);

        const auto fs = actuator.faultStatus();
        EXPECT_TRUE(fs.active());
        EXPECT_EQ(fs.code, FaultCode::LimitExceeded);
}

TEST(StepDirActuatorTest, TravelLimitFaultMapsToLimitExceeded)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());

        engine.haltFromIsr(StopReason::TravelLimit);

        const auto fs = actuator.faultStatus();
        EXPECT_TRUE(fs.active());
        EXPECT_EQ(fs.code, FaultCode::LimitExceeded);
}

TEST(StepDirActuatorTest, StallFaultMapsToStall)
{
        FakePulseEngine engine;
        StepDirActuator actuator(engine, makeCfg());
        ASSERT_TRUE(actuator.begin().ok());
        ASSERT_TRUE(actuator.enable().ok());
        ASSERT_TRUE(actuator.armMotion(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(actuator.startMotion().ok());

        engine.haltFromIsr(StopReason::StallDetected);

        const auto fs = actuator.faultStatus();
        EXPECT_TRUE(fs.active());
        EXPECT_EQ(fs.code, FaultCode::Stall);
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

// =====================================================================
// Tandem ENABLE (secondaryEnablePin)
// =====================================================================
// Host GPIO is a no-op stub — these tests verify the secondary-EN code
// path is reachable and does not crash. A bench build is the real check
// for both EN pins flipping on `enable()` / `disable()`.

TEST(StepDirActuatorTest, SecondaryEnablePinIsAccepted)
{
        FakePulseEngine engine;
        StepDirActuator::Config c = makeCfg();
        c.enablePin = 14;
        c.secondaryEnablePin = 15;
        c.secondaryEnableActiveLow = false; // industrial servo SRV-ON style
        StepDirActuator actuator(engine, c);
        ASSERT_TRUE(actuator.begin().ok());
        EXPECT_TRUE(actuator.enable().ok());
        EXPECT_TRUE(actuator.disable().ok());
}

TEST(StepDirActuatorTest, SecondaryEnableUnsetLeavesBehaviourUnchanged)
{
        FakePulseEngine engine;
        StepDirActuator::Config c = makeCfg();
        c.enablePin = 14;
        c.secondaryEnablePin = GPIO_NONE; // single-drive setup
        StepDirActuator actuator(engine, c);
        ASSERT_TRUE(actuator.begin().ok());
        EXPECT_TRUE(actuator.enable().ok());
        EXPECT_TRUE(actuator.disable().ok());
}

// =====================================================================
// readDriverIdentity — delegates to the configured provider
// =====================================================================

namespace
{
class FakeIdentityProvider final : public IDriverIdentityProvider {
    public:
        Result<DriverIdentity> readDriverIdentity() override
        {
                callCount++;
                if (failNext) {
                        failNext = false;
                        return Result<DriverIdentity>::Err(ErrorCode::TransportError);
                }
                DriverIdentity id;
                id.vendor = "TestVendor";
                id.model = "TestModel";
                id.firmwareMajor = 0x42;
                id.firmwareMinor = 7;
                id.rawId = 0xDEADBEEFu;
                return Result<DriverIdentity>::Ok(id);
        }
        uint32_t callCount = 0;
        bool failNext = false;
};
} // namespace

TEST(StepDirActuatorTest, ReadIdentityReturnsUnsupportedWithoutProvider)
{
        FakePulseEngine engine;
        StepDirActuator::Config c = makeCfg();
        // identityProvider defaults to nullptr.
        StepDirActuator actuator(engine, c);
        auto r = actuator.readDriverIdentity();
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::Unsupported);
}

TEST(StepDirActuatorTest, ReadIdentityForwardsToProvider)
{
        FakePulseEngine engine;
        StepDirActuator::Config c = makeCfg();
        FakeIdentityProvider provider;
        c.identityProvider = &provider;
        StepDirActuator actuator(engine, c);
        auto r = actuator.readDriverIdentity();
        ASSERT_TRUE(r.ok());
        const auto id = r.takeValue();
        EXPECT_STREQ(id.vendor, "TestVendor");
        EXPECT_STREQ(id.model, "TestModel");
        EXPECT_EQ(id.firmwareMajor, 0x42u);
        EXPECT_EQ(id.firmwareMinor, 7u);
        EXPECT_EQ(id.rawId, 0xDEADBEEFu);
        EXPECT_EQ(provider.callCount, 1u);
}

TEST(StepDirActuatorTest, ReadIdentityPropagatesProviderError)
{
        FakePulseEngine engine;
        StepDirActuator::Config c = makeCfg();
        FakeIdentityProvider provider;
        provider.failNext = true;
        c.identityProvider = &provider;
        StepDirActuator actuator(engine, c);
        auto r = actuator.readDriverIdentity();
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::TransportError);
}

} // namespace
