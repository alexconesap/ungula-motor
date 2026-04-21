// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <motor/motor_types.h>

namespace {

    using motor::CurrentCurve;
    using motor::currentMaForSps;

    CurrentCurve makeRisingCurve() {
        CurrentCurve c;
        c.minSps = 300;
        c.maxSps = 2000;
        c.minMa = 900;
        c.maxMa = 1300;
        return c;
    }

    CurrentCurve makeFallingCurve() {
        // High torque at low speed — vertical axis holding weight.
        CurrentCurve c;
        c.minSps = 200;
        c.maxSps = 3000;
        c.minMa = 1300;
        c.maxMa = 900;
        return c;
    }

    TEST(CurrentCurveTest, ClampsBelowMinSpsToMinMa) {
        auto c = makeRisingCurve();
        EXPECT_EQ(currentMaForSps(c, 0), 900);
        EXPECT_EQ(currentMaForSps(c, 100), 900);
        EXPECT_EQ(currentMaForSps(c, 300), 900);
    }

    TEST(CurrentCurveTest, ClampsAboveMaxSpsToMaxMa) {
        auto c = makeRisingCurve();
        EXPECT_EQ(currentMaForSps(c, 2000), 1300);
        EXPECT_EQ(currentMaForSps(c, 5000), 1300);
    }

    TEST(CurrentCurveTest, MidpointInterpolatesHalfway) {
        auto c = makeRisingCurve();
        // Midpoint SPS = 1150, expected mA = 900 + (400 * 850 / 1700) = 900 + 200 = 1100.
        EXPECT_EQ(currentMaForSps(c, 1150), 1100);
    }

    TEST(CurrentCurveTest, InterpolatesAtArbitraryPoint) {
        auto c = makeRisingCurve();
        // At sps=500: offset=200, span=1700, delta=400 → 900 + 400*200/1700 = 900 + 47 = 947.
        EXPECT_EQ(currentMaForSps(c, 500), 947);
    }

    TEST(CurrentCurveTest, FallingCurveReducesCurrentAtHigherSpeeds) {
        auto c = makeFallingCurve();
        EXPECT_EQ(currentMaForSps(c, 200), 1300);
        EXPECT_EQ(currentMaForSps(c, 3000), 900);
        // Midpoint ≈ 1600 SPS: offset=1400, span=2800, delta=-400 → 1300 - 200 = 1100.
        EXPECT_EQ(currentMaForSps(c, 1600), 1100);
    }

    TEST(CurrentCurveTest, NegativeSpsTreatedAsMagnitude) {
        auto c = makeRisingCurve();
        EXPECT_EQ(currentMaForSps(c, -1150), 1100);
        EXPECT_EQ(currentMaForSps(c, -5000), 1300);
    }

    TEST(CurrentCurveTest, DegenerateCurveReturnsMinMa) {
        CurrentCurve c;
        c.minSps = 1000;
        c.maxSps = 1000;
        c.minMa = 800;
        c.maxMa = 1400;
        EXPECT_EQ(currentMaForSps(c, 0), 800);
        EXPECT_EQ(currentMaForSps(c, 1000), 800);
        EXPECT_EQ(currentMaForSps(c, 5000), 800);
    }

    TEST(CurrentCurveTest, InvertedRangeReturnsMinMa) {
        // minSps > maxSps is nonsense; we treat it as degenerate to avoid
        // dividing by a negative span and returning garbage.
        CurrentCurve c;
        c.minSps = 3000;
        c.maxSps = 500;
        c.minMa = 1000;
        c.maxMa = 1500;
        EXPECT_EQ(currentMaForSps(c, 1000), 1000);
    }

    TEST(CurrentCurveTest, DefaultConstructedCurveIsZero) {
        // Default-constructed curve has zeros everywhere. Must not crash and
        // must return 0 so an unconfigured-but-enabled curve fails loud (no
        // current → driver won't move) instead of applying surprise values.
        CurrentCurve c;
        EXPECT_EQ(currentMaForSps(c, 0), 0);
        EXPECT_EQ(currentMaForSps(c, 5000), 0);
    }

}  // namespace
