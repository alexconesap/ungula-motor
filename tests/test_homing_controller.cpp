// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/homing/i_homing_axis.h"
#include "ungula/motor/homing/i_homing_strategy.h"
#include "ungula/motor/homing/limit_switch_homing_strategy.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

/// In-memory IHomingAxis. Simulates the axis with two knobs:
///   - `homeActive` — whether the home sensor reads as pressed.
///   - `motionIdle` — whether a previously-issued command has settled.
///
/// Tests drive the simulated axis by toggling these and calling
/// `controller.tick()` repeatedly.
class FakeHomingAxis : public IHomingAxis {
    public:
        Status commandMove(Distance delta, Velocity feed) override
        {
                ++commandMoveCount;
                lastDelta = delta;
                lastFeed = feed;
                motionIdle = false;
                if (failNextCommand) {
                        failNextCommand = false;
                        motionIdle = true;
                        return Status::Err(ErrorCode::InvalidState);
                }
                return Status::Ok();
        }
        Status commandJog(Direction d, Velocity feed) override
        {
                ++commandJogCount;
                lastJogDir = d;
                lastFeed = feed;
                motionIdle = false;
                if (failNextCommand) {
                        failNextCommand = false;
                        motionIdle = true;
                        return Status::Err(ErrorCode::InvalidState);
                }
                return Status::Ok();
        }
        Status stopMove() override
        {
                ++stopMoveCount;
                motionIdle = true;
                return Status::Ok();
        }
        bool isHomeActive() const override
        {
                return homeActive;
        }
        bool isMotionIdle() const override
        {
                return motionIdle;
        }
        Status resetPosition(Position p) override
        {
                ++resetCount;
                positionAfterReset = p;
                return Status::Ok();
        }

        // Test knobs
        bool homeActive = false;
        bool motionIdle = true;
        bool failNextCommand = false;

        // Counters / inspection
        uint32_t commandMoveCount = 0;
        uint32_t commandJogCount = 0;
        uint32_t stopMoveCount = 0;
        uint32_t resetCount = 0;
        Distance lastDelta = 0;
        Direction lastJogDir = Direction::Forward;
        Velocity lastFeed = 0;
        Position positionAfterReset = 999;
};

LimitSwitchHomingStrategy::Config defaultStrategyCfg()
{
        LimitSwitchHomingStrategy::Config c;
        c.approachDirection = Direction::Backward;
        c.fastFeedSps = 2000;
        c.slowFeedSps = 200;
        c.backoffSteps = 100;
        c.homePositionSteps = 0;
        return c;
}

TEST(HomingControllerTest, FullCycleSucceeds)
{
        LimitSwitchHomingStrategy strat(defaultStrategyCfg());
        HomingController ctrl;
        ctrl.setStrategy(&strat, /*timeoutMs=*/0);

        FakeHomingAxis axis;
        ASSERT_TRUE(ctrl.begin(axis, /*nowMs=*/0).ok());
        EXPECT_EQ(ctrl.phase(), HomingPhase::FastApproach);
        EXPECT_EQ(axis.commandJogCount, 1u);
        EXPECT_EQ(axis.lastJogDir, Direction::Backward);
        EXPECT_EQ(axis.lastFeed, 2000);

        // Motion is in flight; tick should be a no-op.
        ctrl.tick(axis, 1);
        EXPECT_EQ(ctrl.phase(), HomingPhase::FastApproach);

        // The Axis (in real life) detects home active mid-motion and
        // stops; we simulate that here.
        axis.homeActive = true;
        axis.motionIdle = true;

        ctrl.tick(axis, 2);
        EXPECT_EQ(ctrl.phase(), HomingPhase::Backoff);
        EXPECT_EQ(axis.commandMoveCount, 1u);
        // Backoff direction is opposite the approach direction. Approach
        // is Backward → backoff delta should be +100 (forward = positive).
        EXPECT_EQ(axis.lastDelta, 100);

        // Backoff completes; switch should now be off.
        axis.homeActive = false;
        axis.motionIdle = true;

        ctrl.tick(axis, 3);
        EXPECT_EQ(ctrl.phase(), HomingPhase::SlowApproach);
        EXPECT_EQ(axis.commandJogCount, 2u);
        EXPECT_EQ(axis.lastFeed, 200);

        // Slow approach hits the switch.
        axis.homeActive = true;
        axis.motionIdle = true;

        ctrl.tick(axis, 4);
        EXPECT_EQ(ctrl.phase(), HomingPhase::Complete);
        EXPECT_TRUE(ctrl.succeeded());
        EXPECT_EQ(axis.resetCount, 1u);
        EXPECT_EQ(axis.positionAfterReset, 0);
}

TEST(HomingControllerTest, FastApproachWithoutSwitchFails)
{
        LimitSwitchHomingStrategy strat(defaultStrategyCfg());
        HomingController ctrl;
        ctrl.setStrategy(&strat, 0);

        FakeHomingAxis axis;
        ASSERT_TRUE(ctrl.begin(axis, 0).ok());

        // Motion completes naturally (safety bound) without home switch
        // ever firing.
        axis.motionIdle = true;
        axis.homeActive = false;

        ctrl.tick(axis, 1);
        EXPECT_EQ(ctrl.phase(), HomingPhase::Failed);
        EXPECT_FALSE(ctrl.succeeded());
        EXPECT_EQ(ctrl.failureReason(), StopReason::HomingFailed);
}

TEST(HomingControllerTest, BackoffStuckOnSwitchFails)
{
        LimitSwitchHomingStrategy strat(defaultStrategyCfg());
        HomingController ctrl;
        ctrl.setStrategy(&strat, 0);

        FakeHomingAxis axis;
        axis.homeActive = true; // already on switch at start
        ASSERT_TRUE(ctrl.begin(axis, 0).ok());
        EXPECT_EQ(ctrl.phase(), HomingPhase::Backoff);

        // Simulate backoff finishing but switch still pressed (mechanics
        // didn't move us off).
        axis.motionIdle = true;
        axis.homeActive = true;

        ctrl.tick(axis, 1);
        EXPECT_EQ(ctrl.phase(), HomingPhase::Failed);
}

TEST(HomingControllerTest, TimeoutTriggersAbort)
{
        LimitSwitchHomingStrategy strat(defaultStrategyCfg());
        HomingController ctrl;
        ctrl.setStrategy(&strat, /*timeoutMs=*/100);

        FakeHomingAxis axis;
        ASSERT_TRUE(ctrl.begin(axis, /*nowMs=*/0).ok());

        // Time advances without the switch ever firing.
        ctrl.tick(axis, 50);
        EXPECT_TRUE(ctrl.isActive());

        ctrl.tick(axis, 150);
        EXPECT_FALSE(ctrl.isActive());
        EXPECT_EQ(ctrl.phase(), HomingPhase::Failed);
        EXPECT_EQ(axis.stopMoveCount, 1u);
}

TEST(HomingControllerTest, BeginRejectedWithoutStrategy)
{
        HomingController ctrl;
        FakeHomingAxis axis;
        EXPECT_EQ(ctrl.begin(axis, 0).error(), ErrorCode::Unsupported);
}

TEST(HomingControllerTest, AbortStopsAndLatchesReason)
{
        LimitSwitchHomingStrategy strat(defaultStrategyCfg());
        HomingController ctrl;
        ctrl.setStrategy(&strat, 0);

        FakeHomingAxis axis;
        ASSERT_TRUE(ctrl.begin(axis, 0).ok());
        ctrl.abort(axis, StopReason::EmergencyStop);
        EXPECT_EQ(ctrl.phase(), HomingPhase::Failed);
        EXPECT_EQ(ctrl.failureReason(), StopReason::EmergencyStop);
        EXPECT_EQ(axis.stopMoveCount, 1u);
}

} // namespace
