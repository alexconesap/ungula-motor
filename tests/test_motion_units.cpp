// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/axis_types.h"
#include "ungula/motor/motion_units.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

/// Linear axis at the user's actual configuration: 1.8° motor, 16x
/// microstepping, GT2 20-tooth pulley (40 mm/rev). 3200 steps/rev →
/// 80 steps/mm. stepsPerDegree = 3200 / 360.
UnitScaling makeRealisticScaling()
{
        UnitScaling u;
        u.stepsPerMm = 80.0f;
        u.stepsPerDegree = 3200.0f / 360.0f; // ≈ 8.888...
        return u;
}

// ============================================================================
// Speed — linear units
// ============================================================================

TEST(MotionUnitsTest, StepsPerSecIsIdentity)
{
        UnitScaling u; // empty — stepsPerSec doesn't need scaling
        auto r = toStepsPerSec(Speed::stepsPerSec(1234.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 1234);
}

TEST(MotionUnitsTest, MmPerSecScalesByStepsPerMm)
{
        const auto u = makeRealisticScaling();
        // 100 mm/s × 80 steps/mm = 8000 steps/s
        auto r = toStepsPerSec(Speed::mmPerSec(100.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 8000);
}

TEST(MotionUnitsTest, MmPerMinDividesBy60)
{
        const auto u = makeRealisticScaling();
        // 6000 mm/min == 100 mm/s == 8000 steps/s
        auto r = toStepsPerSec(Speed::mmPerMin(6000.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 8000);
}

TEST(MotionUnitsTest, CmPerSecScalesBy10)
{
        const auto u = makeRealisticScaling();
        // 10 cm/s == 100 mm/s == 8000 steps/s
        auto r = toStepsPerSec(Speed::cmPerSec(10.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 8000);
}

TEST(MotionUnitsTest, InchesPerSecScalesByMm)
{
        const auto u = makeRealisticScaling();
        // 1 inch/s == 25.4 mm/s × 80 steps/mm = 2032 steps/s
        auto r = toStepsPerSec(Speed::inchesPerSec(1.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 2032);
}

// ============================================================================
// Speed — rotational units
// ============================================================================

TEST(MotionUnitsTest, RpmConvertsViaStepsPerRev)
{
        const auto u = makeRealisticScaling();
        // 60 rpm = 1 rps = 3200 steps/s on a 16x microstepped 1.8° motor
        auto r = toStepsPerSec(Speed::rpm(60.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 3200);
}

TEST(MotionUnitsTest, RpsConvertsViaStepsPerRev)
{
        const auto u = makeRealisticScaling();
        // 1 rev/s × 3200 steps/rev = 3200 steps/s
        auto r = toStepsPerSec(Speed::rps(1.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 3200);
}

TEST(MotionUnitsTest, DegreesPerSecScalesByStepsPerDegree)
{
        const auto u = makeRealisticScaling();
        // 360 deg/s × 8.888... steps/deg = 3200 steps/s
        auto r = toStepsPerSec(Speed::degreesPerSec(360.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 3200);
}

// ============================================================================
// Speed — failure paths
// ============================================================================

TEST(MotionUnitsTest, MmPerSecFailsWithoutLinearScaling)
{
        UnitScaling u; // stepsPerMm = 0
        auto r = toStepsPerSec(Speed::mmPerSec(100.0f), u);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(MotionUnitsTest, RpmFailsWithoutRotaryScaling)
{
        UnitScaling u;
        u.stepsPerMm = 80.0f; // linear OK, rotary missing
        auto r = toStepsPerSec(Speed::rpm(60.0f), u);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(MotionUnitsTest, NegativeSpeedRejected)
{
        const auto u = makeRealisticScaling();
        auto r = toStepsPerSec(Speed::mmPerSec(-50.0f), u);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

// ============================================================================
// Acceleration — same conversion factors as Speed, by design
// ============================================================================

TEST(MotionUnitsTest, AccelMmPerSecSquaredScalesByStepsPerMm)
{
        const auto u = makeRealisticScaling();
        // 500 mm/s² × 80 steps/mm = 40000 steps/s²
        auto r = toStepsPerSecSquared(Acceleration::mmPerSecSquared(500.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 40000u);
}

TEST(MotionUnitsTest, AccelRpmPerSecMatchesSpeedRpm)
{
        const auto u = makeRealisticScaling();
        // 60 rpm/s == "reach 60 rpm in 1 s" == 3200 steps/s²
        // (same factor as Speed::rpm(60) → 3200 steps/s)
        auto r = toStepsPerSecSquared(Acceleration::rpmPerSec(60.0f), u);
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 3200u);
}

TEST(MotionUnitsTest, AccelZeroRejected)
{
        const auto u = makeRealisticScaling();
        auto r = toStepsPerSecSquared(Acceleration::mmPerSecSquared(0.0f), u);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

// ============================================================================
// TrajectoryLimits helpers
// ============================================================================

TEST(MotionUnitsTest, ApplyMaxVelocityMutatesLimits)
{
        const auto u = makeRealisticScaling();
        TrajectoryLimits L;
        L.maxStepRateSps = 200000;

        auto s = applyMaxVelocity(L, Speed::mmPerSec(50.0f), u);
        ASSERT_TRUE(s.ok());
        EXPECT_EQ(L.maxVelocitySps, 4000);
}

TEST(MotionUnitsTest, ApplyMaxVelocityRejectsBadConfig)
{
        UnitScaling u; // no scaling
        TrajectoryLimits L;
        auto s = applyMaxVelocity(L, Speed::mmPerSec(50.0f), u);
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
        EXPECT_EQ(L.maxVelocitySps, 0); // unchanged on failure
}

TEST(MotionUnitsTest, ApplyRampProfileSetsBothSides)
{
        const auto u = makeRealisticScaling();
        TrajectoryLimits L;

        auto s = applyRampProfile(L, Acceleration::mmPerSecSquared(250.0f), u);
        ASSERT_TRUE(s.ok());
        EXPECT_EQ(L.accelSpsPerSec, 20000u);
        EXPECT_EQ(L.decelSpsPerSec, 20000u);
}

TEST(MotionUnitsTest, ApplyAccelDecelIndependently)
{
        const auto u = makeRealisticScaling();
        TrajectoryLimits L;

        ASSERT_TRUE(applyAcceleration(L, Acceleration::mmPerSecSquared(500.0f), u).ok());
        ASSERT_TRUE(applyDeceleration(L, Acceleration::mmPerSecSquared(250.0f), u).ok());

        EXPECT_EQ(L.accelSpsPerSec, 40000u);
        EXPECT_EQ(L.decelSpsPerSec, 20000u);
}

}  // namespace
