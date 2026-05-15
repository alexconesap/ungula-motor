// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/hal/timer/drivers/hwtimer_fake.h"
#include "ungula/hal/timer/i_hwtimer.h"

#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/planning/planned_move.h"
#include "ungula/motor/pulse/hal_pulse_engine.h"
#include "ungula/motor/pulse/i_pulse_engine.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;
using ungula::hal::timer::HwTimerStatus;
using ungula::hal::timer::drivers::HwTimerFake;

constexpr uint8_t STEP_PIN = 18;
constexpr uint8_t DIR_PIN = 19;
constexpr uint32_t RES_HZ = 1'000'000;

HalPulseEngine::Config makeConfig()
{
        HalPulseEngine::Config cfg;
        cfg.stepPin = STEP_PIN;
        cfg.dirPin = DIR_PIN;
        cfg.dirActiveHigh = true;
        cfg.dirSetupUs = 0; // tests don't need to wait
        cfg.timerResolutionHz = RES_HZ;
        cfg.timerMinTicks = 5;
        return cfg;
}

/// Build a deterministic single-segment move so we can count fires exactly.
PlannedMove makeMove(Direction dir, uint32_t steps, uint32_t halfPeriodTicks = 100)
{
        PlannedMove m;
        m.direction = dir;
        m.totalSteps = steps;
        m.segmentCount = 1;
        m.segments[0].stepCount = steps;
        m.segments[0].halfPeriodTicks = halfPeriodTicks;
        return m;
}

/// Build a two-segment move to verify segment-boundary advancement.
PlannedMove makeTwoSegmentMove()
{
        PlannedMove m;
        m.direction = Direction::Forward;
        m.totalSteps = 8;
        m.segmentCount = 2;
        m.segments[0].stepCount = 3;
        m.segments[0].halfPeriodTicks = 200;
        m.segments[1].stepCount = 5;
        m.segments[1].halfPeriodTicks = 100;
        return m;
}

// =====================================================================
// begin()
// =====================================================================

TEST(HalPulseEngineTest, BeginRejectsBadPin)
{
        HwTimerFake timer;
        auto cfg = makeConfig();
        cfg.stepPin = GPIO_NONE;
        HalPulseEngine eng(timer, cfg);
        EXPECT_EQ(eng.begin(PulseMode::Internal).error(), ErrorCode::InvalidConfig);
}

TEST(HalPulseEngineTest, BeginRejectsExternalMode)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        EXPECT_EQ(eng.begin(PulseMode::External).error(), ErrorCode::Unsupported);
}

TEST(HalPulseEngineTest, BeginIsIdempotentlyRejected)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        EXPECT_TRUE(eng.begin(PulseMode::Internal).ok());
        EXPECT_EQ(eng.begin(PulseMode::Internal).error(), ErrorCode::AlreadyInitialized);
}

TEST(HalPulseEngineTest, BeginBringsTimerUpAndRegistersCallback)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        EXPECT_TRUE(eng.begin(PulseMode::Internal).ok());
        EXPECT_EQ(timer.beginCallCount, 1u);
        EXPECT_EQ(timer.setCallbackCallCount, 1u);
        EXPECT_FALSE(timer.isArmed()); // begin alone does not arm
}

// =====================================================================
// loadMove()
// =====================================================================

TEST(HalPulseEngineTest, LoadMoveRequiresBegin)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        const auto m = makeMove(Direction::Forward, 10);
        EXPECT_EQ(eng.loadMove(m).error(), ErrorCode::NotInitialized);
}

TEST(HalPulseEngineTest, LoadMoveRejectsEmpty)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        PlannedMove empty;
        EXPECT_EQ(eng.loadMove(empty).error(), ErrorCode::InvalidConfig);
}

TEST(HalPulseEngineTest, LoadMoveRejectsZeroStepCountSegment)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        PlannedMove m = makeMove(Direction::Forward, 10);
        m.segments[0].stepCount = 0; // poisoned
        EXPECT_EQ(eng.loadMove(m).error(), ErrorCode::InvalidConfig);
}

TEST(HalPulseEngineTest, LoadMoveRejectsZeroHalfPeriodSegment)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        PlannedMove m = makeMove(Direction::Forward, 10);
        m.segments[0].halfPeriodTicks = 0;
        EXPECT_EQ(eng.loadMove(m).error(), ErrorCode::InvalidConfig);
}

TEST(HalPulseEngineTest, LoadMoveRejectsHalfPeriodBelowTimerMinTicks)
{
        HwTimerFake timer;
        auto cfg = makeConfig();
        cfg.timerMinTicks = 50;
        HalPulseEngine eng(timer, cfg);
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());

        PlannedMove m = makeMove(Direction::Forward, 10, /*halfPeriod=*/20);
        EXPECT_EQ(eng.loadMove(m).error(), ErrorCode::InvalidConfig)
            << "must reject any halfPeriod < cfg.timerMinTicks";
}

TEST(HalPulseEngineTest, LoadMoveRejectsSegmentSumMismatch)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        PlannedMove m;
        m.direction = Direction::Forward;
        m.totalSteps = 100;
        m.segmentCount = 1;
        m.segments[0].stepCount = 99; // sum != totalSteps
        m.segments[0].halfPeriodTicks = 100;
        EXPECT_EQ(eng.loadMove(m).error(), ErrorCode::InvalidConfig);
}

TEST(HalPulseEngineTest, LoadMoveRejectedWhileRunning)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(eng.start().ok());
        EXPECT_EQ(eng.loadMove(makeMove(Direction::Forward, 50)).error(),
                  ErrorCode::MotionInProgress);
}

// =====================================================================
// start()
// =====================================================================

TEST(HalPulseEngineTest, StartRequiresLoadedMove)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        EXPECT_EQ(eng.start().error(), ErrorCode::InvalidState);
}

TEST(HalPulseEngineTest, StartArmsTimerWithFirstSegmentHalfPeriod)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());

        auto m = makeMove(Direction::Forward, 4, /*halfPeriod=*/250);
        ASSERT_TRUE(eng.loadMove(m).ok());
        ASSERT_TRUE(eng.start().ok());

        EXPECT_TRUE(timer.isArmed());
        EXPECT_EQ(timer.lastArmedTicks(), 250u);
        EXPECT_EQ(timer.startCallCount, 1u);
        EXPECT_TRUE(eng.isRunning());
}

// =====================================================================
// Full-move execution
// =====================================================================

TEST(HalPulseEngineTest, ForwardMoveAdvancesPositionByTotalSteps)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());

        const uint32_t steps = 20;
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, steps)).ok());
        ASSERT_TRUE(eng.start().ok());

        // Drive enough ISR fires to cover the full move. The engine emits
        // 2*N fires per N steps — N rising edges + N falling edges (the
        // last falling edge is what triggers completion, so the final HIGH
        // pulse gets a full half-period of width). Pump a wide upper bound;
        // the fake drops extras silently after disarm.
        timer.fireMany(2 * steps + 10);

        EXPECT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.commandedPositionSteps(), static_cast<int32_t>(steps));

        const auto st = eng.status();
        EXPECT_EQ(st.emittedSteps, steps);
        EXPECT_EQ(st.finishedReason, StopReason::TargetReached);
        EXPECT_FALSE(st.faulted);

        // Disarm must have happened — timer should now drop any further fires.
        EXPECT_FALSE(timer.isArmed());
        EXPECT_GT(timer.firesDroppedWhileDisarmed, 0u);
}

TEST(HalPulseEngineTest, BackwardMoveDecrementsPosition)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());

        const uint32_t steps = 15;
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Backward, steps)).ok());
        ASSERT_TRUE(eng.start().ok());
        timer.fireMany(2 * steps + 10);

        EXPECT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.commandedPositionSteps(), -static_cast<int32_t>(steps));
}

TEST(HalPulseEngineTest, AdvancesSegmentsAtBoundaries)
{
        // 8 steps total: 3 at halfPeriod=200, 5 at halfPeriod=100.
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());

        ASSERT_TRUE(eng.loadMove(makeTwoSegmentMove()).ok());
        ASSERT_TRUE(eng.start().ok());

        // start() arms with the first segment's halfPeriod.
        EXPECT_EQ(timer.lastArmedTicks(), 200u);
        EXPECT_EQ(eng.status().segmentIndex, 0u);

        // 4 fires = 2 full step pulses (rising-falling × 2). Still inside
        // segment 0. The 3rd step (segment-0 boundary) lands on the 5th
        // fire — a *rising* edge that simultaneously increments stepsInSeg
        // to 3 and advances to segment 1, re-arming with seg[1].halfPeriod.
        timer.fireMany(4);
        EXPECT_EQ(eng.commandedPositionSteps(), 2);
        EXPECT_EQ(eng.status().segmentIndex, 0u);
        EXPECT_EQ(timer.lastArmedTicks(), 200u);

        // One more fire — the 5th — completes segment 0 and arms segment 1.
        timer.fireMany(1);
        EXPECT_EQ(eng.commandedPositionSteps(), 3);
        EXPECT_EQ(eng.status().segmentIndex, 1u);
        EXPECT_EQ(timer.lastArmedTicks(), 100u);

        // Drain the remaining 5 steps of segment 1.
        timer.fireMany(100);
        EXPECT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.commandedPositionSteps(), 8);
        EXPECT_EQ(eng.status().finishedReason, StopReason::TargetReached);
}

// =====================================================================
// stop() / emergencyStop()
// =====================================================================

TEST(HalPulseEngineTest, StopMidFlightHaltsAndLatchesUserStop)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(eng.start().ok());

        timer.fireMany(10); // 5 steps emitted

        EXPECT_TRUE(eng.isRunning());
        EXPECT_EQ(eng.commandedPositionSteps(), 5);

        EXPECT_TRUE(eng.stop(StopMode::Immediate).ok());
        EXPECT_FALSE(eng.isRunning());
        EXPECT_FALSE(timer.isArmed());

        const auto st = eng.status();
        EXPECT_EQ(st.finishedReason, StopReason::UserStop);
        EXPECT_FALSE(st.faulted);
}

TEST(HalPulseEngineTest, EmergencyStopLatchesFault)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 100)).ok());
        ASSERT_TRUE(eng.start().ok());

        timer.fireMany(4); // 2 steps
        EXPECT_TRUE(eng.emergencyStop().ok());

        const auto st = eng.status();
        EXPECT_FALSE(st.running);
        EXPECT_TRUE(st.faulted);
        EXPECT_EQ(st.finishedReason, StopReason::EmergencyStop);
}

TEST(HalPulseEngineTest, StopIsIdempotentWhenIdle)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        EXPECT_TRUE(eng.stop(StopMode::Immediate).ok()); // already idle
}

// =====================================================================
// resetPosition / running guards
// =====================================================================

TEST(HalPulseEngineTest, ResetPositionRejectedWhileRunning)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 10)).ok());
        ASSERT_TRUE(eng.start().ok());
        EXPECT_EQ(eng.resetPosition(0).error(), ErrorCode::MotionInProgress);
}

TEST(HalPulseEngineTest, ResetPositionWorksAfterStop)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 10)).ok());
        ASSERT_TRUE(eng.start().ok());
        timer.fireMany(2 * 10 + 5); // run to completion
        ASSERT_FALSE(eng.isRunning());
        EXPECT_TRUE(eng.resetPosition(42).ok());
        EXPECT_EQ(eng.commandedPositionSteps(), 42);
}

// =====================================================================
// Repeat-move (regression for the 1.5.6 lifecycle bug in lib_hal)
// =====================================================================

TEST(HalPulseEngineTest, ConsecutiveMovesSucceedWithoutBackendError)
{
        HwTimerFake timer;
        HalPulseEngine eng(timer, makeConfig());
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());

        // Move 1
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 5)).ok());
        ASSERT_TRUE(eng.start().ok());
        timer.fireMany(2 * 5 + 5);
        ASSERT_FALSE(eng.isRunning());
        EXPECT_EQ(eng.commandedPositionSteps(), 5);

        // Move 2 — must succeed against an already-completed timer.
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 7)).ok());
        EXPECT_TRUE(eng.start().ok()) << "second move should not return BackendError";
        timer.fireMany(2 * 7 + 5);
        EXPECT_EQ(eng.commandedPositionSteps(), 12);

        // Move 3, opposite direction.
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Backward, 4)).ok());
        EXPECT_TRUE(eng.start().ok());
        timer.fireMany(2 * 4 + 5);
        EXPECT_EQ(eng.commandedPositionSteps(), 8);
}

// =====================================================================
// Tandem DIR (secondaryDirPin)
// =====================================================================
// Host GPIO is a no-op stub — these are smoke tests: the new code path
// must not crash the engine, and configuration plumbing must accept the
// tandem fields without rejection. A bench / ESP32 build is the real
// verifier for both DIR pins flipping inside the dirSetupUs window.

TEST(HalPulseEngineTest, TandemDirPinConfiguresAndRuns)
{
        HwTimerFake timer;
        auto cfg = makeConfig();
        cfg.secondaryDirPin = 21;
        cfg.secondaryDirActiveHigh = true;
        cfg.secondaryDirInverted = false;
        HalPulseEngine eng(timer, cfg);
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 4)).ok());
        ASSERT_TRUE(eng.start().ok());
        timer.fireMany(2 * 4 + 5);
        EXPECT_EQ(eng.commandedPositionSteps(), 4);
}

TEST(HalPulseEngineTest, TandemDirPinInvertedBranchExercised)
{
        HwTimerFake timer;
        auto cfg = makeConfig();
        cfg.secondaryDirPin = 21;
        cfg.secondaryDirActiveHigh = true;
        cfg.secondaryDirInverted = true; // face-to-face mounting case
        HalPulseEngine eng(timer, cfg);
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Backward, 3)).ok());
        ASSERT_TRUE(eng.start().ok());
        timer.fireMany(2 * 3 + 5);
        EXPECT_EQ(eng.commandedPositionSteps(), -3);
}

TEST(HalPulseEngineTest, NoSecondaryDirPinLeavesBehaviourUnchanged)
{
        // Regression: the new code branch must be a no-op when
        // secondaryDirPin = GPIO_NONE (single-motor wiring).
        HwTimerFake timer;
        auto cfg = makeConfig();
        cfg.secondaryDirPin = GPIO_NONE;
        HalPulseEngine eng(timer, cfg);
        ASSERT_TRUE(eng.begin(PulseMode::Internal).ok());
        ASSERT_TRUE(eng.loadMove(makeMove(Direction::Forward, 4)).ok());
        ASSERT_TRUE(eng.start().ok());
        timer.fireMany(2 * 4 + 5);
        EXPECT_EQ(eng.commandedPositionSteps(), 4);
}

} // namespace
