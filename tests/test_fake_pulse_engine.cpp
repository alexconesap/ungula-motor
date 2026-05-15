// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
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

TEST(FakePulseEngineTest, LifecycleHonoursOrder)
{
        FakePulseEngine eng;
        EXPECT_EQ(eng.loadMove(makeMove(Direction::Forward, 10)).error(),
                  ErrorCode::NotInitialized);

        EXPECT_TRUE(eng.begin(PulseMode::Internal).ok());
        EXPECT_EQ(eng.begin(PulseMode::Internal).error(), ErrorCode::AlreadyInitialized);

        EXPECT_EQ(eng.start().error(), ErrorCode::InvalidState);

        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 10)).ok());
        ASSERT_TRUE(eng.start().ok());
        EXPECT_TRUE(eng.isRunning());
        EXPECT_EQ(eng.start().error(), ErrorCode::MotionInProgress);
}

TEST(FakePulseEngineTest, TickStepsAdvancesPosition)
{
        FakePulseEngine eng;
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(eng.start().ok());

        eng.tickSteps(40);
        EXPECT_EQ(eng.commandedPositionSteps(), 40);
        EXPECT_TRUE(eng.isRunning());

        eng.tickSteps(60);
        EXPECT_EQ(eng.commandedPositionSteps(), 100);
        EXPECT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.status().finishedReason, StopReason::TargetReached);
}

TEST(FakePulseEngineTest, RunMoveCompletesEntireMove)
{
        FakePulseEngine eng;
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Backward, 25)).ok());
        ASSERT_TRUE(eng.start().ok());
        eng.runMove();
        EXPECT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.commandedPositionSteps(), -25);
        EXPECT_EQ(eng.status().finishedReason, StopReason::TargetReached);
}

TEST(FakePulseEngineTest, StopMidFlightLatchesUserStop)
{
        FakePulseEngine eng;
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 50)).ok());
        ASSERT_TRUE(eng.start().ok());
        eng.tickSteps(10);
        ASSERT_TRUE(eng.stop(StopMode::Immediate).ok());
        EXPECT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.status().finishedReason, StopReason::UserStop);
}

TEST(FakePulseEngineTest, EmergencyStopLatchesFault)
{
        FakePulseEngine eng;
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 50)).ok());
        ASSERT_TRUE(eng.start().ok());
        eng.tickSteps(5);
        ASSERT_TRUE(eng.emergencyStop().ok());
        const auto st = eng.status();
        EXPECT_FALSE(st.running);
        EXPECT_TRUE(st.faulted);
        EXPECT_EQ(st.finishedReason, StopReason::EmergencyStop);
}

TEST(FakePulseEngineTest, InjectFaultLatchesDriverFault)
{
        FakePulseEngine eng;
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 50)).ok());
        ASSERT_TRUE(eng.start().ok());
        eng.injectFault();
        const auto st = eng.status();
        EXPECT_FALSE(st.running);
        EXPECT_TRUE(st.faulted);
        EXPECT_EQ(st.finishedReason, StopReason::DriverFault);
}

} // namespace
