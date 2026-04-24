// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <motor/homing/stall_homing_strategy.h>

#include "mock_homeable_motor.h"

namespace {

    using motor::Direction;
    using motor::MotionProfile;
    using motor::MotorFsmState;
    using motor::StallHomingStrategy;
    using motor::test::MockHomeableMotor;

    StallHomingStrategy::Config makeConfig(bool finalApproach = true) {
        StallHomingStrategy::Config cfg;
        cfg.homingDirection = Direction::BACKWARD;
        cfg.fastSpeedSps = 2000;
        cfg.fastAccelMs = 200U;
        cfg.slowSpeedSps = 500;
        cfg.slowAccelMs = 100U;
        cfg.backoffSteps = 200;
        cfg.finalApproach = finalApproach;
        return cfg;
    }

    TEST(StallHomingStrategyTest, BeginEnforcesAutoStopAndClearsFaults) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig());

        strategy.begin(mock);

        EXPECT_TRUE(mock.autoStopOnStall);
        EXPECT_GE(mock.clearStallCount, 1);
        EXPECT_GE(mock.clearFaultCount, 1);
        // Backward homing → first move is moveBackward().
        EXPECT_EQ(mock.moveBackwardCount, 1);
        EXPECT_EQ(mock.moveForwardCount, 0);
        EXPECT_FALSE(strategy.succeeded());
    }

    TEST(StallHomingStrategyTest, FullSequenceCompletesWithFinalApproach) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig(true));

        strategy.begin(mock);

        // --- FastApproach: nothing until the motor stalls ---
        mock.scriptedState = MotorFsmState::RunningBackward;
        EXPECT_FALSE(strategy.tick(mock));

        // Stall → strategy should clear it and command a backoff moveBy(+steps).
        mock.scriptedState = MotorFsmState::Stall;
        const int stallAcksBefore = mock.clearStallCount;
        EXPECT_FALSE(strategy.tick(mock));
        EXPECT_GT(mock.clearStallCount, stallAcksBefore);
        ASSERT_FALSE(mock.moveByCalls.empty());
        // Backoff is opposite to homingDirection (BACKWARD), so delta is +200.
        EXPECT_GT(mock.moveByCalls.back(), 0.0F);

        // --- Backoff complete: TargetReached triggers slow approach ---
        mock.scriptedState = MotorFsmState::TargetReached;
        const int eStopsBefore = mock.emergencyStopCount;
        EXPECT_FALSE(strategy.tick(mock));
        EXPECT_GT(mock.emergencyStopCount, eStopsBefore);
        // A second moveBackward (slow re-approach) must have been issued.
        EXPECT_EQ(mock.moveBackwardCount, 2);

        // --- Slow approach: second stall means done ---
        mock.scriptedState = MotorFsmState::Stall;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_TRUE(strategy.succeeded());
    }

    TEST(StallHomingStrategyTest, SingleStallModeSkipsBackoff) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig(false));

        strategy.begin(mock);
        EXPECT_EQ(mock.moveBackwardCount, 1);

        mock.scriptedState = MotorFsmState::Stall;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_TRUE(strategy.succeeded());
        // No backoff → exactly one motion command was issued.
        EXPECT_TRUE(mock.moveByCalls.empty());
        EXPECT_EQ(mock.moveBackwardCount, 1);
    }

    TEST(StallHomingStrategyTest, FaultDuringFastApproachFailsCleanly) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig());

        strategy.begin(mock);
        mock.scriptedState = MotorFsmState::Fault;
        EXPECT_TRUE(strategy.tick(mock));
        EXPECT_FALSE(strategy.succeeded());
    }

    TEST(StallHomingStrategyTest, FinishResetsPositionOnlyOnSuccess) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig(false));

        strategy.begin(mock);
        strategy.finish(mock, true);
        EXPECT_EQ(mock.resetPositionCount, 1);

        MockHomeableMotor mock2;
        StallHomingStrategy strategy2(makeConfig(false));
        strategy2.begin(mock2);
        strategy2.finish(mock2, false);
        EXPECT_EQ(mock2.resetPositionCount, 0);
    }

    TEST(StallHomingStrategyTest, ForwardHomingIssuesMoveForward) {
        auto cfg = makeConfig();
        cfg.homingDirection = Direction::FORWARD;
        MockHomeableMotor mock;
        StallHomingStrategy strategy(cfg);

        strategy.begin(mock);
        EXPECT_EQ(mock.moveForwardCount, 1);
        EXPECT_EQ(mock.moveBackwardCount, 0);
    }

    TEST(StallHomingStrategyTest, WritesHomingProfileBeforeEveryMove) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig(false));

        strategy.begin(mock);
        ASSERT_FALSE(mock.activeProfiles.empty());
        EXPECT_EQ(mock.activeProfiles.front(), MotionProfile::HOMING);
        ASSERT_FALSE(mock.profileWrites.empty());
        EXPECT_EQ(mock.profileWrites.front().profile, MotionProfile::HOMING);
        EXPECT_EQ(mock.profileWrites.front().speedSps, 2000);
    }

    // Boot-time seed: a stall-based strategy has no "am I at home" signal at
    // rest — the driver is "not stalling" whether the axis is on the stop or
    // not. Must always report false so LocalMotor::begin() won't wrongly seed
    // isHomed across a reboot.
    TEST(StallHomingStrategyTest, IsAtHomeReferenceAlwaysFalse) {
        MockHomeableMotor mock;
        StallHomingStrategy strategy(makeConfig());

        mock.scriptedLimitBackward = true;  // irrelevant for stall homing.
        mock.scriptedLimitForward = true;
        EXPECT_FALSE(strategy.isAtHomeReference(mock));
    }

}  // namespace
