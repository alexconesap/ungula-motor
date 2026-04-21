// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <motor/homing/limit_switch_homing_strategy.h>

#include "mock_homeable_motor.h"

namespace {

    using motor::Direction;
    using motor::LimitSwitchHomingStrategy;
    using motor::MotionProfile;
    using motor::MotorFsmState;
    using motor::test::MockHomeableMotor;

    LimitSwitchHomingStrategy::Config makeConfig(bool finalApproach = true) {
        LimitSwitchHomingStrategy::Config cfg;
        cfg.homingDirection = Direction::BACKWARD;
        cfg.fastSpeedSps = 3000;
        cfg.fastAccelMs = 150U;
        cfg.slowSpeedSps = 400;
        cfg.slowAccelMs = 100U;
        cfg.backoffSteps = 300;
        cfg.finalApproach = finalApproach;
        return cfg;
    }

    TEST(LimitSwitchHomingStrategyTest, BeginClearsFaultsAndStartsFastApproach) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig());

        strategy.begin(mock);

        EXPECT_GE(mock.clearStallCount, 1);
        EXPECT_GE(mock.clearFaultCount, 1);
        EXPECT_EQ(mock.moveBackwardCount, 1);
        EXPECT_FALSE(strategy.succeeded());
    }

    TEST(LimitSwitchHomingStrategyTest, FullSequenceCompletesWithFinalApproach) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig(true));

        strategy.begin(mock);

        // First hit on the switch stops motion → strategy emergency-stops the
        // motor and launches backoff.
        mock.scriptedState = MotorFsmState::LimitReached;
        const int eStopsBefore = mock.emergencyStopCount;
        EXPECT_FALSE(strategy.tick(mock));
        EXPECT_GT(mock.emergencyStopCount, eStopsBefore);
        ASSERT_FALSE(mock.moveByCalls.empty());
        EXPECT_GT(mock.moveByCalls.back(), 0.0F);  // backoff is opposite sign.

        // Backoff finishes: TargetReached → slow re-approach.
        mock.scriptedState = MotorFsmState::TargetReached;
        EXPECT_FALSE(strategy.tick(mock));
        EXPECT_EQ(mock.moveBackwardCount, 2);

        // Slow approach re-hits the switch → done, success.
        mock.scriptedState = MotorFsmState::LimitReached;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_TRUE(strategy.succeeded());
    }

    TEST(LimitSwitchHomingStrategyTest, SingleHitModeSkipsBackoff) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig(false));

        strategy.begin(mock);
        mock.scriptedState = MotorFsmState::LimitReached;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_TRUE(strategy.succeeded());
        EXPECT_TRUE(mock.moveByCalls.empty());
    }

    TEST(LimitSwitchHomingStrategyTest, BackoffThatFailsToClearSwitchFails) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig(true));

        strategy.begin(mock);
        mock.scriptedState = MotorFsmState::LimitReached;
        EXPECT_FALSE(strategy.tick(mock));  // enters Backoff phase.

        // Backoff too small — still on the switch when backoff completes.
        mock.scriptedState = MotorFsmState::LimitReached;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_FALSE(strategy.succeeded());
    }

    TEST(LimitSwitchHomingStrategyTest, FaultDuringFastApproachFailsCleanly) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig());

        strategy.begin(mock);
        mock.scriptedState = MotorFsmState::Fault;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_FALSE(strategy.succeeded());
    }

    TEST(LimitSwitchHomingStrategyTest, FinishResetsPositionOnlyOnSuccess) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig(false));

        strategy.begin(mock);
        strategy.finish(mock, true);
        EXPECT_EQ(mock.resetPositionCount, 1);
    }

    TEST(LimitSwitchHomingStrategyTest, DoesNotForceAutoStopOnStall) {
        MockHomeableMotor mock;
        LimitSwitchHomingStrategy strategy(makeConfig());

        strategy.begin(mock);
        // Unlike the stall strategy, this one must leave autoStopOnStall alone.
        EXPECT_FALSE(mock.autoStopOnStall);
    }

}  // namespace
