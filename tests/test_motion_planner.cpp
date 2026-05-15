// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/axis_types.h"
#include "ungula/motor/planning/motion_planner.h"
#include "ungula/motor/planning/planned_move.h"

namespace
{

using namespace ungula::motor;

constexpr uint32_t RES_HZ = 1'000'000; // 1 MHz tick → 1 µs

TrajectoryLimits sensibleLimits()
{
        TrajectoryLimits l;
        l.maxVelocitySps = 5000;
        l.accelSpsPerSec = 20000;
        l.decelSpsPerSec = 20000;
        l.minPulseHighUs = 2;
        l.minPulseLowUs = 2;
        l.maxStepRateSps = 200000;
        return l;
}

uint32_t sumSteps(const PlannedMove &m)
{
        uint32_t sum = 0;
        for (uint8_t i = 0; i < m.segmentCount; ++i)
                sum += m.segments[i].stepCount;
        return sum;
}

// =====================================================================
// Edge cases
// =====================================================================

TEST(MotionPlannerTest, ZeroDeltaProducesEmptyMove)
{
        MotionPlanner p;
        const auto m = p.planBy(0, sensibleLimits(), RES_HZ);
        EXPECT_EQ(m.totalSteps, 0u);
        EXPECT_EQ(m.segmentCount, 0u);
}

TEST(MotionPlannerTest, PathologicalLimitsProduceEmptyMove)
{
        MotionPlanner p;
        TrajectoryLimits l = sensibleLimits();
        l.maxVelocitySps = 0;
        EXPECT_EQ(p.planBy(100, l, RES_HZ).segmentCount, 0u);

        l = sensibleLimits();
        l.accelSpsPerSec = 0;
        EXPECT_EQ(p.planBy(100, l, RES_HZ).segmentCount, 0u);

        l = sensibleLimits();
        l.decelSpsPerSec = 0;
        EXPECT_EQ(p.planBy(100, l, RES_HZ).segmentCount, 0u);
}

TEST(MotionPlannerTest, NegativeDeltaSetsBackwardDirection)
{
        MotionPlanner p;
        const auto m = p.planBy(-1000, sensibleLimits(), RES_HZ);
        EXPECT_EQ(m.direction, Direction::Backward);
        EXPECT_EQ(m.totalSteps, 1000u);
}

TEST(MotionPlannerTest, PositiveDeltaSetsForwardDirection)
{
        MotionPlanner p;
        const auto m = p.planBy(1000, sensibleLimits(), RES_HZ);
        EXPECT_EQ(m.direction, Direction::Forward);
        EXPECT_EQ(m.totalSteps, 1000u);
}

TEST(MotionPlannerTest, SingleStepUsesConstantVelocityFallback)
{
        MotionPlanner p;
        const auto m = p.planBy(1, sensibleLimits(), RES_HZ);
        EXPECT_EQ(m.segmentCount, 1u);
        EXPECT_EQ(m.segments[0].stepCount, 1u);
        EXPECT_GT(m.segments[0].halfPeriodTicks, 0u);
}

TEST(MotionPlannerTest, VeryShortMoveUsesConstantVelocityFallback)
{
        MotionPlanner p;
        const auto m = p.planBy(3, sensibleLimits(), RES_HZ);
        // Under 4 steps → single-segment fallback.
        EXPECT_EQ(m.segmentCount, 1u);
        EXPECT_EQ(m.segments[0].stepCount, 3u);
}

// =====================================================================
// Trapezoidal profile (long move with cruise)
// =====================================================================

TEST(MotionPlannerTest, TrapezoidalReachesPeakVelocity)
{
        MotionPlanner p;
        TrajectoryLimits l = sensibleLimits();
        // Big move to ensure accel + decel fits with room for cruise.
        const auto m = p.planBy(10'000, l, RES_HZ);

        EXPECT_GT(m.segmentCount, 3u); // at least accel + cruise + decel
        EXPECT_LE(m.segmentCount, MAX_PLANNED_SEGMENTS);
        EXPECT_EQ(m.direction, Direction::Forward);
        EXPECT_EQ(sumSteps(m), m.totalSteps);

        // Confirm the cruise segment exists by finding a "long" segment in
        // the middle whose half-period corresponds (approximately) to the
        // peak velocity (maxV = 5000 SPS → halfPeriod = 1MHz / 10000 = 100).
        // Cruise should be the segment with the smallest halfPeriod and the
        // largest stepCount.
        uint8_t cruiseIdx = 0;
        uint32_t minHp = UINT32_MAX;
        for (uint8_t i = 0; i < m.segmentCount; ++i) {
                if (m.segments[i].halfPeriodTicks < minHp) {
                        minHp = m.segments[i].halfPeriodTicks;
                        cruiseIdx = i;
                }
        }
        // Cruise should run at peak velocity ~ 5000 SPS → halfPeriod ~ 100 ticks
        EXPECT_LE(minHp, 110u);
        EXPECT_GE(minHp, 90u);

        // Cruise should be far enough into the move to be neither first
        // nor last (i.e. a real cruise, not a degenerate triangular).
        EXPECT_GT(cruiseIdx, 0u);
        EXPECT_LT(cruiseIdx, static_cast<uint8_t>(m.segmentCount - 1));
}

TEST(MotionPlannerTest, TrapezoidalRespectsMaxStepRate)
{
        MotionPlanner p;
        TrajectoryLimits l = sensibleLimits();
        l.maxVelocitySps = 100'000; // ask for 100 kSPS
        l.maxStepRateSps = 5'000; // but cap at 5 kSPS
        const auto m = p.planBy(20'000, l, RES_HZ);
        EXPECT_EQ(sumSteps(m), m.totalSteps);

        // Half-period for 5 kSPS at 1MHz = 100. Cruise should NOT go below
        // (i.e. cruise should not exceed 5 kSPS).
        uint32_t minHp = UINT32_MAX;
        for (uint8_t i = 0; i < m.segmentCount; ++i) {
                if (m.segments[i].halfPeriodTicks < minHp)
                        minHp = m.segments[i].halfPeriodTicks;
        }
        EXPECT_GE(minHp, 95u); // allow small rounding under
}

// =====================================================================
// Triangular profile (short move, peak velocity < maxV)
// =====================================================================

TEST(MotionPlannerTest, TriangularDoesNotReachMaxVelocity)
{
        MotionPlanner p;
        TrajectoryLimits l = sensibleLimits();
        // maxV = 5000 SPS, accel/decel = 20000 SPS². To reach maxV needs
        // 5000²/(2*20000) = 625 steps for accel + 625 for decel = 1250.
        // Pick a smaller move so the planner has to use a triangular profile.
        const auto m = p.planBy(500, l, RES_HZ);

        EXPECT_GE(m.segmentCount, 2u);
        EXPECT_LE(m.segmentCount, MAX_PLANNED_SEGMENTS);
        EXPECT_EQ(sumSteps(m), m.totalSteps);

        // Peak speed should be lower than 5 kSPS → minHp should be larger
        // than the trapezoidal peak (100 ticks).
        uint32_t minHp = UINT32_MAX;
        for (uint8_t i = 0; i < m.segmentCount; ++i) {
                if (m.segments[i].halfPeriodTicks < minHp)
                        minHp = m.segments[i].halfPeriodTicks;
        }
        EXPECT_GT(minHp, 100u);
}

// =====================================================================
// Step-count conservation
// =====================================================================

TEST(MotionPlannerTest, SegmentStepsSumToTotalSteps)
{
        MotionPlanner p;
        const TrajectoryLimits l = sensibleLimits();
        for (Distance d : { 7, 50, 250, 1000, 5000, 20'000, 50'000 }) {
                const auto m = p.planBy(d, l, RES_HZ);
                EXPECT_EQ(sumSteps(m), m.totalSteps)
                    << "delta=" << d << "  segs=" << static_cast<int>(m.segmentCount);
                EXPECT_EQ(m.totalSteps, static_cast<uint32_t>(d));
        }
}

// =====================================================================
// EXACT step-count guarantee — the planner must move EXACTLY what was
// requested, not "approximately due to ramp rounding".
// =====================================================================

TEST(MotionPlannerTest, PreservesRequestedDistanceAcrossFullRange)
{
        MotionPlanner p;
        const TrajectoryLimits l = sensibleLimits();
        // Cover triangular (small), boundary, and trapezoidal (large), plus
        // a few values right around the trapezoidal/triangular crossover
        // (~ 1250 steps with default limits).
        for (Distance d : { 1, 2, 3, 4, 5, 10, 100, 250, 500, 1000, 1248, 1250, 1252, 2000, 5000,
                            10'000, 50'000, 100'000 }) {
                const auto m = p.planBy(d, l, RES_HZ);
                EXPECT_EQ(m.totalSteps, static_cast<uint32_t>(d))
                    << "planBy(" << d << ") returned totalSteps=" << m.totalSteps;
                EXPECT_EQ(sumSteps(m), static_cast<uint32_t>(d))
                    << "sum of segments != requested for delta=" << d;
        }
}

TEST(MotionPlannerTest, ExactDistanceForBackwardMoves)
{
        MotionPlanner p;
        const TrajectoryLimits l = sensibleLimits();
        for (Distance d : { -1, -3, -100, -1248, -1250, -5000, -50'000 }) {
                const auto m = p.planBy(d, l, RES_HZ);
                const uint32_t expected = static_cast<uint32_t>(-d);
                EXPECT_EQ(m.totalSteps, expected);
                EXPECT_EQ(sumSteps(m), expected);
                EXPECT_EQ(m.direction, Direction::Backward);
        }
}

TEST(MotionPlannerTest, ExactDistanceUnderTightAccelLimits)
{
        MotionPlanner p;
        // High accel → small accelSteps → ramp-rounding error becomes a
        // larger fraction of the total. Worst case for the rebalancer.
        TrajectoryLimits l = sensibleLimits();
        l.accelSpsPerSec = 1'000'000;
        l.decelSpsPerSec = 1'000'000;
        for (Distance d : { 17, 41, 97, 211, 1009, 9999 }) {
                const auto m = p.planBy(d, l, RES_HZ);
                EXPECT_EQ(m.totalSteps, static_cast<uint32_t>(d));
                EXPECT_EQ(sumSteps(m), static_cast<uint32_t>(d));
        }
}

// =====================================================================
// Timer minTicks awareness — planner clamps half-period to the floor.
// =====================================================================

TEST(MotionPlannerTest, RespectsTimerMinTicks)
{
        MotionPlanner p;
        TrajectoryLimits l = sensibleLimits();
        l.minPulseHighUs = 0;
        l.minPulseLowUs = 0;
        // Force a high peak SPS that would otherwise produce a tiny
        // half-period.
        l.maxVelocitySps = 500'000;
        l.maxStepRateSps = 500'000;

        const uint32_t timerMin = 50; // floor = 50 ticks
        const auto m = p.planBy(20'000, l, RES_HZ, timerMin);

        for (uint8_t i = 0; i < m.segmentCount; ++i) {
                EXPECT_GE(m.segments[i].halfPeriodTicks, timerMin)
                    << "seg[" << static_cast<int>(i) << "] hp=" << m.segments[i].halfPeriodTicks;
        }
        // Distance preserved even with the cap.
        EXPECT_EQ(m.totalSteps, 20'000u);
        EXPECT_EQ(sumSteps(m), 20'000u);
}

TEST(MotionPlannerTest, PlanStopPreservesAnalyticalDistance)
{
        MotionPlanner p;
        const TrajectoryLimits l = sensibleLimits();
        // Stop from 4000 SPS at 20000 SPS/s decel → 4000^2/(2*20000) = 400 steps.
        const auto m = p.planStop(Direction::Forward, 4000, l, RES_HZ);
        EXPECT_GT(m.totalSteps, 0u);
        EXPECT_EQ(sumSteps(m), m.totalSteps);
        // Should be close to 400; rebalancer absorbs any ramp-rounding drift.
        EXPECT_GE(m.totalSteps, 380u);
        EXPECT_LE(m.totalSteps, 420u);
}

// =====================================================================
// PlanTo / PlanJog / PlanStop
// =====================================================================

TEST(MotionPlannerTest, PlanToProducesMatchingDeltaMove)
{
        MotionPlanner p;
        const auto m = p.planTo(100, 500, sensibleLimits(), RES_HZ);
        EXPECT_EQ(m.direction, Direction::Forward);
        EXPECT_EQ(m.totalSteps, 400u);

        const auto mBack = p.planTo(500, 100, sensibleLimits(), RES_HZ);
        EXPECT_EQ(mBack.direction, Direction::Backward);
        EXPECT_EQ(mBack.totalSteps, 400u);
}

TEST(MotionPlannerTest, PlanJogHonoursDirection)
{
        MotionPlanner p;
        const auto fwd = p.planJog(Direction::Forward, 10'000, sensibleLimits(), RES_HZ);
        EXPECT_EQ(fwd.direction, Direction::Forward);
        EXPECT_EQ(fwd.totalSteps, 10'000u);

        const auto back = p.planJog(Direction::Backward, 10'000, sensibleLimits(), RES_HZ);
        EXPECT_EQ(back.direction, Direction::Backward);
        EXPECT_EQ(back.totalSteps, 10'000u);
}

TEST(MotionPlannerTest, PlanStopFromZeroProducesEmptyMove)
{
        MotionPlanner p;
        const auto m = p.planStop(Direction::Forward, 0, sensibleLimits(), RES_HZ);
        EXPECT_EQ(m.totalSteps, 0u);
        EXPECT_EQ(m.segmentCount, 0u);
}

TEST(MotionPlannerTest, PlanStopProducesMonotonicDecel)
{
        MotionPlanner p;
        const auto m = p.planStop(Direction::Forward, 4000, sensibleLimits(), RES_HZ);
        EXPECT_GT(m.segmentCount, 0u);

        // Half-period should grow monotonically from segment to segment
        // (velocity decreases → period increases).
        for (uint8_t i = 1; i < m.segmentCount; ++i) {
                EXPECT_GE(m.segments[i].halfPeriodTicks, m.segments[i - 1].halfPeriodTicks)
                    << "non-monotonic decel at seg " << static_cast<int>(i);
        }
}

// =====================================================================
// Half-period minimum enforced from pulse-width constraint
// =====================================================================

TEST(MotionPlannerTest, MinPulseWidthClampsHalfPeriod)
{
        MotionPlanner p;
        TrajectoryLimits l = sensibleLimits();
        l.minPulseHighUs = 50; // 50 µs minimum → halfPeriod ≥ 50 ticks at 1MHz
        l.minPulseLowUs = 50;
        const auto m = p.planBy(10'000, l, RES_HZ);

        uint32_t minHp = UINT32_MAX;
        for (uint8_t i = 0; i < m.segmentCount; ++i) {
                if (m.segments[i].halfPeriodTicks < minHp)
                        minHp = m.segments[i].halfPeriodTicks;
        }
        EXPECT_GE(minHp, 50u);
}

} // namespace
