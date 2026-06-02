// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_step_signal.h"
#include "ungula/motor/drivers/generic_stepdir/generic_stepdir_driver.h"
#include "ungula/motor/drivers/ypmc/ypmc_driver.h"
#include "ungula/motor/planners/trapezoidal_planner.h"

using namespace ungula::motor;
using ungula::motor::tests::FakeStepSignal;
using ungula::motor::stepdir::GenericStepDirConfig;
using ungula::motor::stepdir::GenericStepDirDriver;
using ungula::motor::ypmc::YpmcConfig;
using ungula::motor::ypmc::YpmcStepDirDriver;

// =====================================================================
// GenericStepDirDriver
// =====================================================================

TEST(GenericStepDirDriver, BeginEnableDisableLifecycle)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        GenericStepDirConfig cfg;
        cfg.enablePin = 32;
        cfg.enableActiveLow = true;
        GenericStepDirDriver drv(cfg, step, planner);

        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.enable().ok());
        ASSERT_TRUE(drv.disable().ok());
}

TEST(GenericStepDirDriver, IdentityIsGenericUnknown)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        GenericStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const auto id = drv.identity();
        EXPECT_STREQ(id.vendor, "Unknown");
        EXPECT_STREQ(id.model, "Generic STEP/DIR");
}

TEST(GenericStepDirDriver, IntentNonDefaultReportsUnsupported)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        GenericStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        EXPECT_EQ(drv.applyIntent(MotorIntent::Default), IntentSupport::Supported);
        EXPECT_EQ(drv.applyIntent(MotorIntent::Quiet), IntentSupport::Unsupported);
        EXPECT_EQ(drv.applyIntent(MotorIntent::HighTorque), IntentSupport::Unsupported);
}

TEST(GenericStepDirDriver, ArmMoveDelegatesToStepSignal)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        GenericStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.enable().ok());

        ASSERT_TRUE(drv.armMove(Direction::Forward, 5000, 40000, 160000, 160000).ok());
        EXPECT_EQ(step.armCalls_, 1u);
        EXPECT_EQ(step.lastMove_.totalSteps, 5000u);
        EXPECT_EQ(step.lastMove_.direction, Direction::Forward);
}

TEST(GenericStepDirDriver, StopForwardsModeToStepSignal)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        GenericStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.enable().ok());
        ASSERT_TRUE(drv.armMove(Direction::Forward, 1000, 10000, 40000, 40000).ok());

        ASSERT_TRUE(drv.stop(StopMode::Immediate).ok());
        EXPECT_EQ(step.lastStopMode_, StopMode::Immediate);
}

// =====================================================================
// YpmcStepDirDriver
// =====================================================================

TEST(YpmcStepDirDriver, IdentityIsRattmotorYpmc)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        YpmcStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const auto id = drv.identity();
        EXPECT_STREQ(id.vendor, "RATTMOTOR");
        EXPECT_STREQ(id.model, "YPMC + S2SVD15");
}

TEST(YpmcStepDirDriver, ArmMoveSucceedsWithoutSecondaryDir)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        YpmcConfig cfg;
        cfg.generic.enablePin = 23;
        // No secondaryDirPin → single motor.
        YpmcStepDirDriver drv(cfg, step, planner);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.enable().ok());

        ASSERT_TRUE(drv.armMove(Direction::Forward, 2000, 20000, 80000, 80000).ok());
        EXPECT_EQ(step.armCalls_, 1u);
        EXPECT_EQ(step.lastMove_.direction, Direction::Forward);
}

TEST(YpmcStepDirDriver, ArmMoveWithSecondaryDirPinDoesNotCrash)
{
        // The host stub for lib_hal::gpio is a no-op; we just verify
        // the YPMC driver's secondary-DIR write path doesn't trip an
        // assertion or fail the armMove.
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        YpmcConfig cfg;
        cfg.secondaryDirPin = 33;
        cfg.secondaryDirActiveHigh = true;
        cfg.secondaryDirInverted = false;
        YpmcStepDirDriver drv(cfg, step, planner);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.enable().ok());

        ASSERT_TRUE(drv.armMove(Direction::Forward, 1000, 20000, 80000, 80000).ok());
        ASSERT_TRUE(drv.armMove(Direction::Backward, 1000, 20000, 80000, 80000).ok());
}

TEST(YpmcStepDirDriver, ArmMoveWithBrakeDoesNotCrash)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        YpmcConfig cfg;
        cfg.brakePin = 27;
        cfg.brakeReleaseSettleMs = 0; // zero so the test doesn't sleep
        cfg.brakeEngageSettleMs = 0;
        YpmcStepDirDriver drv(cfg, step, planner);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.enable().ok());

        ASSERT_TRUE(drv.armMove(Direction::Forward, 500, 20000, 80000, 80000).ok());
        ASSERT_TRUE(drv.stop(StopMode::Immediate).ok());
}

TEST(YpmcStepDirDriver, IntentReportsUnsupportedExceptDefault)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        YpmcStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        EXPECT_EQ(drv.applyIntent(MotorIntent::Default), IntentSupport::Supported);
        EXPECT_EQ(drv.applyIntent(MotorIntent::Quiet), IntentSupport::Unsupported);
        EXPECT_EQ(drv.applyIntent(MotorIntent::AdaptiveCurrent), IntentSupport::Unsupported);
}

TEST(YpmcStepDirDriver, DiagnosticsExposeYpmcIdentity)
{
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        YpmcStepDirDriver drv({}, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        MotorDiagnostics d;
        drv.fillDriverDiagnostics(d);
        EXPECT_STREQ(d.identity.vendor, "RATTMOTOR");
        EXPECT_FALSE(d.stall_valid);
        EXPECT_FALSE(d.adaptive_current_valid);
}
