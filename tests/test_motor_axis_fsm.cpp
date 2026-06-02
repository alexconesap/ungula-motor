// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_limit_system.h"
#include "fakes/fake_motor_driver.h"
#include "ungula/core/time/time.h"
#include "ungula/motor/homing/home_to_limit_strategy.h"
#include "ungula/motor/motor_axis.h"
#include "ungula/motor/motor_axis_config.h"

using namespace ungula::motor;
using ungula::motor::tests::FakeLimitSystem;
using ungula::motor::tests::FakeMotorDriver;

namespace
{

MotorAxisConfig makeConfig()
{
        MotorAxisConfig cfg;
        cfg.id = MotorAxisId{ 0 };
        cfg.name = "axis-under-test";
        cfg.units.stepsPerRevolution = 3200; // 1.8° motor at 16× microstep
        cfg.units.stepsPerMm = 80.0f;
        cfg.units.stepsPerDegree = 3200.0f / 360.0f;
        cfg.limits.maxSpeed = Speed::stepsPerSec(800);
        cfg.limits.accel = Acceleration::stepsPerSecSquared(4000);
        cfg.limits.decel = Acceleration::stepsPerSecSquared(4000);
        cfg.limits.hardStepRateCeilingSps = 200000;
        return cfg;
}

} // namespace

// =====================================================================
// Lifecycle
// =====================================================================

TEST(MotorAxisFsm, BeginTransitionsUninitialisedToDisabled)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        EXPECT_EQ(axis.state(), MotorState::Uninitialized);

        ASSERT_TRUE(axis.begin().ok());
        EXPECT_EQ(axis.state(), MotorState::Disabled);
        EXPECT_EQ(drv.beginCalls, 1u);
}

TEST(MotorAxisFsm, EnableTransitionsDisabledToIdle)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_EQ(drv.enableCalls, 1u);
}

TEST(MotorAxisFsm, EnableBeforeBeginRejects)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        const auto s = axis.enable();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::NotInitialized);
}

// =====================================================================
// Motion verbs
// =====================================================================

TEST(MotorAxisFsm, MoveForwardArmsAndTransitionsToJogging)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.moveForward().ok());
        EXPECT_EQ(axis.state(), MotorState::Jogging);
        EXPECT_EQ(drv.armJogCalls, 1u);
        EXPECT_EQ(drv.lastArmedDirection, Direction::Forward);
        EXPECT_EQ(drv.lastArmedCruiseSps, 800u);
}

TEST(MotorAxisFsm, MoveBackwardArmsAndTransitionsToJogging)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.moveBackward().ok());
        EXPECT_EQ(axis.state(), MotorState::Jogging);
        EXPECT_EQ(drv.lastArmedDirection, Direction::Backward);
}

TEST(MotorAxisFsm, MoveByConvertsMmToStepsAndCommandsDriver)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // 5 mm at 80 steps/mm = 400 steps Forward.
        ASSERT_TRUE(axis.moveBy(DistanceValue::mm(5)).ok());
        EXPECT_EQ(axis.state(), MotorState::Moving);
        EXPECT_EQ(drv.lastArmedDirection, Direction::Forward);
        EXPECT_EQ(drv.lastArmedTargetSteps, 400u);
}

TEST(MotorAxisFsm, MoveByZeroDistanceIsImmediateCompletion)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.moveBy(DistanceValue::steps(0)).ok());
        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_EQ(drv.armMoveCalls, 0u);
        EXPECT_EQ(axis.lastStopReason(), StopReason::TargetReached);
}

TEST(MotorAxisFsm, MoveByRejectsMmWhenStepsPerMmZero)
{
        auto cfg = makeConfig();
        cfg.units.stepsPerMm = 0.0f;
        FakeMotorDriver drv;
        MotorAxis axis(cfg, drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        const auto s = axis.moveBy(DistanceValue::mm(10));
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
        EXPECT_EQ(drv.armMoveCalls, 0u);
}

// =====================================================================
// Speed unit conversion at the axis boundary
// =====================================================================

TEST(MotorAxisFsm, SetSpeedRpmResolvesToSps)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // 60 RPM × 3200 steps/rev / 60s = 3200 SPS.
        ASSERT_TRUE(axis.setSpeed(Speed::rpm(60)).ok());
        ASSERT_TRUE(axis.moveForward().ok());
        EXPECT_EQ(drv.lastArmedCruiseSps, 3200u);
}

TEST(MotorAxisFsm, SetSpeedClampsToHardCeiling)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.setSpeed(Speed::stepsPerSec(500000)).ok());
        ASSERT_TRUE(axis.moveForward().ok());
        EXPECT_EQ(drv.lastArmedCruiseSps, 200000u); // hard ceiling
}

// =====================================================================
// Motion completion
// =====================================================================

TEST(MotorAxisFsm, ServiceCompletesMotionAndTransitionsToIdle)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.moveBy(DistanceValue::steps(400)).ok());
        EXPECT_EQ(axis.state(), MotorState::Moving);

        drv.simulateMotionComplete(400, StopReason::TargetReached);
        axis.service(1);
        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_EQ(axis.lastStopReason(), StopReason::TargetReached);
}

// =====================================================================
// Stop + emergency stop
// =====================================================================

TEST(MotorAxisFsm, StopTransitionsRunningAxisToStoppingThenIdleAfterServiceTick)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        ASSERT_TRUE(axis.moveForward().ok());

        ASSERT_TRUE(axis.stop().ok());
        EXPECT_EQ(axis.state(), MotorState::Stopping);

        // The driver returns Unsupported for Decelerate (default behaviour
        // for a fake) so MotorAxis falls back to Immediate. Either way the
        // motion ends.
        drv.simulateMotionComplete(0, StopReason::UserStop);
        axis.service(1);
        EXPECT_EQ(axis.state(), MotorState::Idle);
}

TEST(MotorAxisFsm, EmergencyStopLatchesAndTransitionsToEmergencyStopped)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        ASSERT_TRUE(axis.moveForward().ok());

        ASSERT_TRUE(axis.emergencyStop().ok());
        EXPECT_EQ(axis.state(), MotorState::EmergencyStopped);
        EXPECT_EQ(axis.lastStopReason(), StopReason::EmergencyStop);
}

// =====================================================================
// Limit-system integration
// =====================================================================

TEST(MotorAxisFsm, JogIntoActiveTravelLimitIsRejected)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        limits.travelFwd = true; // forward limit pre-activated
        MotorAxis axis(makeConfig(), drv, &limits);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        const auto s = axis.moveForward();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::LimitActive);
        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_EQ(drv.armJogCalls, 0u);
}

// notifyMotionStart used to pass a hard-coded sentinel of 1 ms, which
// made the limit system's "arm window" check (`nowMs - armedAt < W`)
// trivially false on every subsequent service tick - the window
// appeared to have expired immediately. That defeated the stall
// arm-window debounce: any DIAG that was already active at the
// moment motion started would latch a stall on the next tick. The
// axis must pass the real wall-clock instead.
TEST(MotorAxisFsm, MoveForwardPassesRealTimestampToLimitArmWindow)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        MotorAxis axis(makeConfig(), drv, &limits);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        const int64_t before = ungula::core::time::millis();
        ASSERT_TRUE(axis.moveForward().ok());
        const int64_t after = ungula::core::time::millis();
        // Real wall-clock at moveForward time, not the sentinel 1.
        EXPECT_GE(limits.motionStartedAtMs, before);
        EXPECT_LE(limits.motionStartedAtMs, after);
}

TEST(MotorAxisFsm, TravelLimitDuringJogStopsAxisFromServiceTick)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        MotorAxis axis(makeConfig(), drv, &limits);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        ASSERT_TRUE(axis.moveForward().ok());

        limits.travelFwd = true; // forward limit just fired
        axis.service(10);
        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_EQ(axis.lastStopReason(), StopReason::TravelLimit);
}

TEST(MotorAxisFsm, EmergencyLimitLatchTransitionsToEmergencyStopped)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        MotorAxis axis(makeConfig(), drv, &limits);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        ASSERT_TRUE(axis.moveForward().ok());

        limits.emergencyActive = true; // ISR-latched emergency
        axis.service(10);
        EXPECT_EQ(axis.state(), MotorState::EmergencyStopped);
        EXPECT_EQ(axis.lastStopReason(), StopReason::EmergencyStop);
}

TEST(MotorAxisFsm, StallLatchTransitionsToFaulted)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        MotorAxis axis(makeConfig(), drv, &limits);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        ASSERT_TRUE(axis.moveForward().ok());

        limits.stallActive = true;
        axis.service(10);
        EXPECT_EQ(axis.state(), MotorState::Faulted);
        EXPECT_EQ(axis.lastStopReason(), StopReason::StallDetected);
}

// =====================================================================
// Fault recovery
// =====================================================================

TEST(MotorAxisFsm, ClearFaultReturnsToDisabledAndDrainsLatches)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        MotorAxis axis(makeConfig(), drv, &limits);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());
        ASSERT_TRUE(axis.moveForward().ok());

        limits.stallActive = true;
        axis.service(10);
        ASSERT_EQ(axis.state(), MotorState::Faulted);

        // Force the fake's faulted flag so clearFault has work to do.
        drv.faulted = true;
        ASSERT_TRUE(axis.clearFault().ok());
        EXPECT_EQ(axis.state(), MotorState::Disabled);
        EXPECT_EQ(drv.clearFaultCalls, 1u);
}

// =====================================================================
// Intent
// =====================================================================

TEST(MotorAxisFsm, SetIntentForwardsToDriver)
{
        FakeMotorDriver drv;
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());

        const auto i = MotorIntent::Quiet | MotorIntent::Cool;
        ASSERT_TRUE(axis.setIntent(i).ok());
        EXPECT_EQ(drv.lastIntent, i);
        EXPECT_GE(drv.applyIntentCalls, 1u);
}

// =====================================================================
// Diagnostics
// =====================================================================

TEST(MotorAxisFsm, DiagnosticsFillIdentityAndStateFromDriverAndAxis)
{
        FakeMotorDriver drv;
        drv.id = DriverIdentity{ "Acme", "MX2", 1, 2, 0xABCD };
        MotorAxis axis(makeConfig(), drv);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        const auto d = axis.diagnostics();
        EXPECT_EQ(d.state, MotorState::Idle);
        EXPECT_STREQ(d.identity.vendor, "Acme");
        EXPECT_STREQ(d.identity.model, "MX2");
        EXPECT_EQ(d.identity.firmwareMajor, 1u);
}

TEST(MotorAxisFsm, IdentityReportsDriverIdentity)
{
        FakeMotorDriver drv;
        drv.id = DriverIdentity{ "X", "Y", 0, 0, 0 };
        MotorAxis axis(makeConfig(), drv);
        const auto id = axis.identity();
        EXPECT_STREQ(id.vendor, "X");
        EXPECT_STREQ(id.model, "Y");
}

// ---- Homing end-to-end --------------------------------------------------
//
// Regression coverage for the bug that lived in 1.0.x: MotorAxis::home()
// transitioned to Homing, then HomeToLimitStrategy::start() called
// MotorAxis::moveBackward(), but moveBackward() rejected any state
// other than Idle - so home() always returned InvalidState unless the
// home sensor happened to be active at boot.

TEST(MotorAxisFsm, HomeArmsBackwardJogFromHomingState)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        HomeToLimitStrategy homing(limits);

        MotorAxis axis(makeConfig(), drv, &limits, &homing);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        // Home sensor NOT yet active. The strategy must arm a backward
        // jog from inside Homing - the bug rejected this with
        // InvalidState.
        ASSERT_TRUE(axis.home().ok());
        EXPECT_EQ(axis.state(), MotorState::Homing);
        EXPECT_EQ(drv.armJogCalls, 1u);
        EXPECT_EQ(drv.lastArmedDirection, Direction::Backward);
}

TEST(MotorAxisFsm, HomingCompletesWhenHomeSensorActivatesMidJog)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        HomeToLimitStrategy homing(limits);

        MotorAxis axis(makeConfig(), drv, &limits, &homing);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.home().ok());
        EXPECT_EQ(axis.state(), MotorState::Homing);

        // Mid-jog the home sensor fires. Strategy halts the axis and
        // declares success on the next service tick; the axis resets
        // commanded position to 0 and transitions to Idle.
        limits.homeActive = true;
        axis.service(10);
        // tick() called axis.stop() -> Stopping. pumpMotion may have
        // moved through to Idle in the same tick depending on the
        // fake's behaviour; either way, another service drains.
        axis.service(20);

        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_TRUE(axis.isHomed());
        EXPECT_EQ(axis.positionSteps(), 0);
}

TEST(MotorAxisFsm, HomingShortCircuitsWhenHomeSensorAlreadyActive)
{
        FakeMotorDriver drv;
        FakeLimitSystem limits;
        limits.homeActive = true;
        HomeToLimitStrategy homing(limits);

        MotorAxis axis(makeConfig(), drv, &limits, &homing);
        ASSERT_TRUE(axis.begin().ok());
        ASSERT_TRUE(axis.enable().ok());

        ASSERT_TRUE(axis.home().ok());
        // No jog should have been armed - the strategy declared
        // success immediately from start().
        EXPECT_EQ(drv.armJogCalls, 0u);

        // Service tick lets the post-process run (Phase::Done -> done,
        // position reset, Idle, isHomed=true).
        axis.service(10);
        EXPECT_EQ(axis.state(), MotorState::Idle);
        EXPECT_TRUE(axis.isHomed());
}
