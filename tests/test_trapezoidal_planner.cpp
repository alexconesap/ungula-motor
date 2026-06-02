// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "ungula/motor/planners/trapezoidal_planner.h"

using namespace ungula::motor;

namespace
{

PlannerLimits standardLimits()
{
        PlannerLimits L;
        L.maxVelocitySps = 80000;
        L.accelSpsPerSec = 320000;
        L.decelSpsPerSec = 320000;
        L.hardStepRateCeilingSps = 200000;
        L.minPulseHighUs = 2;
        L.minPulseLowUs = 2;
        return L;
}

uint32_t sumSegments(const PlannedMove &m)
{
        uint32_t s = 0;
        for (uint8_t i = 0; i < m.segmentCount; ++i) {
                s += m.segments[i].stepCount;
        }
        return s;
}

} // namespace

TEST(TrapezoidalPlanner, ZeroDistanceReturnsEmptyMove)
{
        TrapezoidalPlanner p;
        const auto move = p.planMove(0, 0, standardLimits(), 1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 0u);
        EXPECT_EQ(move.segmentCount, 0u);
}

TEST(TrapezoidalPlanner, ShortMovePreservesStepCount)
{
        TrapezoidalPlanner p;
        const auto move = p.planMove(0, 3, standardLimits(), 1'000'000u, 5u);
        // < 4 steps falls back to a single constant-velocity segment.
        EXPECT_EQ(move.totalSteps, 3u);
        EXPECT_EQ(sumSegments(move), 3u);
        EXPECT_EQ(move.segmentCount, 1u);
        EXPECT_EQ(move.direction, Direction::Forward);
}

TEST(TrapezoidalPlanner, LongMoveSumsToExactStepCount)
{
        TrapezoidalPlanner p;
        const auto move = p.planMove(0, 10000, standardLimits(), 1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 10000u);
        EXPECT_EQ(sumSegments(move), 10000u);
        EXPECT_GE(move.segmentCount, 3u); // accel + cruise + decel at minimum
}

TEST(TrapezoidalPlanner, BackwardSignSetsDirection)
{
        TrapezoidalPlanner p;
        const auto move = p.planMove(0, -5000, standardLimits(), 1'000'000u, 5u);
        EXPECT_EQ(move.direction, Direction::Backward);
        EXPECT_EQ(move.totalSteps, 5000u);
}

TEST(TrapezoidalPlanner, JogPlanRespectsSafetyCap)
{
        TrapezoidalPlanner p;
        const auto move = p.planJog(Direction::Forward, 50000, standardLimits(),
                                    1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 50000u);
        EXPECT_EQ(move.direction, Direction::Forward);
        EXPECT_EQ(sumSegments(move), 50000u);
}

TEST(TrapezoidalPlanner, ZeroLimitsReturnEmptyMove)
{
        TrapezoidalPlanner p;
        PlannerLimits L;
        L.maxVelocitySps = 0;
        const auto move = p.planMove(0, 1000, L, 1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 0u);
        EXPECT_EQ(move.segmentCount, 0u);
}

// accel = 0 means "no ramp, jump to cruise instantly". The planner
// must emit a single cruise segment covering every requested step,
// not return empty. Use case: low-RPM jog where a ramp would be
// pointless overhead.
TEST(TrapezoidalPlanner, ZeroAccelEmitsSingleCruiseSegment)
{
        TrapezoidalPlanner p;
        PlannerLimits L = standardLimits();
        L.accelSpsPerSec = 0;   // no ramp up
        L.decelSpsPerSec = 0;   // no ramp down
        const auto move = p.planMove(0, 1000, L, 1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 1000u);
        EXPECT_EQ(move.segmentCount, 1u);
        EXPECT_EQ(sumSegments(move), 1000u);
        EXPECT_EQ(move.direction, Direction::Forward);
        EXPECT_GT(move.cruiseSps, 0u);
}

// accel = 0 should also work for a jog (indefinite move). The planner
// uses the same path so the no-ramp segment covers all of safetyCapSteps.
TEST(TrapezoidalPlanner, ZeroAccelJogEmitsSingleCruiseSegment)
{
        TrapezoidalPlanner p;
        PlannerLimits L = standardLimits();
        L.accelSpsPerSec = 0;
        L.decelSpsPerSec = 0;
        const auto move = p.planJog(Direction::Backward, 50000, L, 1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 50000u);
        EXPECT_EQ(move.segmentCount, 1u);
        EXPECT_EQ(move.direction, Direction::Backward);
}

// Asymmetric zero (only accel zero, decel non-zero): keep the
// behaviour predictable - no ramp anywhere. Ramping down without
// ramping up would need a hybrid plan the planner doesn't model.
TEST(TrapezoidalPlanner, AsymmetricZeroAccelStillNoRamp)
{
        TrapezoidalPlanner p;
        PlannerLimits L = standardLimits();
        L.accelSpsPerSec = 0;
        // decel left at standardLimits()
        const auto move = p.planMove(0, 1000, L, 1'000'000u, 5u);
        EXPECT_EQ(move.totalSteps, 1000u);
        EXPECT_EQ(move.segmentCount, 1u);
}
