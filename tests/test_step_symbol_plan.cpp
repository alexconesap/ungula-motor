// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "ungula/motor/motion_segment.h"
#include "ungula/motor/step_signal/step_symbol_plan.h"

using ungula::motor::MotionSegment;
using ungula::motor::PlannedMove;
using ungula::motor::rmtgen::MAX_CHUNK_TICKS;
using ungula::motor::rmtgen::MoveSymbolExpander;
using ungula::motor::rmtgen::StepSymbolCursor;
using ungula::motor::rmtgen::SymbolSlots;

namespace
{

// Drain a cursor for a single step into a flat list of (level, ticks)
// slots, asserting the per-slot invariants the RMT driver relies on:
// every duration in [1, 0x7FFF], never zero.
struct Slot {
        uint8_t level;
        uint16_t ticks;
};

std::vector<Slot> drain(uint32_t hp, uint32_t &symbolCount)
{
        StepSymbolCursor cur;
        cur.reset(hp);
        std::vector<Slot> slots;
        symbolCount = 0;
        // Generous guard so a logic bug can't hang the test.
        const uint32_t maxSymbols = 2u * (hp / MAX_CHUNK_TICKS + 2u);
        while (!cur.done()) {
                const SymbolSlots s = cur.next();
                ++symbolCount;
                EXPECT_GE(s.duration0, 1u);
                EXPECT_LE(s.duration0, MAX_CHUNK_TICKS);
                EXPECT_GE(s.duration1, 1u);
                EXPECT_LE(s.duration1, MAX_CHUNK_TICKS);
                slots.push_back({ s.level0, s.duration0 });
                slots.push_back({ s.level1, s.duration1 });
                if (symbolCount > maxSymbols) {
                        ADD_FAILURE() << "cursor failed to terminate for hp=" << hp;
                        break;
                }
        }
        return slots;
}

// Sum the durations spent at each level. For a faithful step the HIGH
// total and LOW total must both equal hp (modulo the documented 1-tick
// lone-chunk fudge, which the chosen hp values avoid).
void totals(const std::vector<Slot> &slots, uint64_t &high, uint64_t &low)
{
        high = 0;
        low = 0;
        for (const auto &s : slots) {
                if (s.level == 1u) {
                        high += s.ticks;
                } else {
                        low += s.ticks;
                }
        }
}

// HIGH must come strictly before LOW: once a LOW slot appears, no HIGH
// slot may follow. Guarantees a clean single rising then single falling
// edge per step rather than a glitchy pulse train.
bool highBeforeLow(const std::vector<Slot> &slots)
{
        bool sawLow = false;
        for (const auto &s : slots) {
                if (s.level == 0u) {
                        sawLow = true;
                } else if (sawLow) {
                        return false;
                }
        }
        return true;
}

} // namespace

TEST(StepSymbolPlan, FastPathSingleSymbol)
{
        // hp within one chunk → exactly one symbol, the classic {HIGH,LOW}.
        for (uint32_t hp : { 1u, 2u, 100u, 1476u, 14792u,
                             static_cast<uint32_t>(MAX_CHUNK_TICKS) }) {
                uint32_t symbols = 0;
                const auto slots = drain(hp, symbols);
                EXPECT_EQ(symbols, 1u) << "hp=" << hp;
                ASSERT_EQ(slots.size(), 2u);
                EXPECT_EQ(slots[0].level, 1u);
                EXPECT_EQ(slots[0].ticks, hp);
                EXPECT_EQ(slots[1].level, 0u);
                EXPECT_EQ(slots[1].ticks, hp);
        }
}

TEST(StepSymbolPlan, WidePathPreservesTotals)
{
        // The reported failure: ramp sub-segment near 121 sps at 10 MHz →
        // hp ≈ 41322, plus boundary and large values.
        for (uint32_t hp : { 0x8000u, 0x8001u, 0xFFFFu, 0x10000u, 41322u,
                             100000u, 1000000u, 5000000u }) {
                uint32_t symbols = 0;
                const auto slots = drain(hp, symbols);
                uint64_t high = 0;
                uint64_t low = 0;
                totals(slots, high, low);
                EXPECT_EQ(high, hp) << "HIGH total wrong for hp=" << hp;
                EXPECT_EQ(low, hp) << "LOW total wrong for hp=" << hp;
                EXPECT_TRUE(highBeforeLow(slots)) << "edge ordering wrong for hp=" << hp;
                EXPECT_GE(symbols, 2u) << "hp=" << hp;
        }
}

TEST(StepSymbolPlan, EveryHalfPeriodNearBoundaryIsExact)
{
        // Sweep across the 0x7FFF boundary where chunk counts change, to
        // catch off-by-one packing errors. Each run is hp long → totals
        // must be exact (even chunk count, no lone-chunk fudge).
        for (uint32_t hp = 0x7FF0u; hp <= 0x8010u; ++hp) {
                uint32_t symbols = 0;
                const auto slots = drain(hp, symbols);
                uint64_t high = 0;
                uint64_t low = 0;
                totals(slots, high, low);
                EXPECT_EQ(high, hp) << "HIGH total wrong for hp=" << hp;
                EXPECT_EQ(low, hp) << "LOW total wrong for hp=" << hp;
                EXPECT_TRUE(highBeforeLow(slots)) << "edge ordering wrong for hp=" << hp;
        }
}

TEST(StepSymbolPlan, DoneAfterDrain)
{
        StepSymbolCursor cur;
        cur.reset(0x12345u);
        while (!cur.done()) {
                (void)cur.next();
        }
        EXPECT_TRUE(cur.done());
}

namespace
{

// Fully expand a move and check, per step, that HIGH and LOW each total the
// segment's half-period and edges are ordered. Returns the number of
// completed steps.
uint32_t expandAndVerify(const PlannedMove &move)
{
        MoveSymbolExpander exp;
        exp.reset(move);

        uint32_t stepsDone = 0;
        std::vector<Slot> stepSlots;
        uint8_t segIdx = 0;
        uint32_t stepsInSeg = 0;
        // Advance segIdx to the first non-empty segment for the expected hp.
        auto skipEmpty = [&]() {
                while (segIdx < move.segmentCount && move.segments[segIdx].stepCount == 0u) {
                        ++segIdx;
                }
        };
        skipEmpty();

        uint64_t guard = 0;
        const uint64_t guardMax = static_cast<uint64_t>(move.totalSteps) * 200u + 1000u;
        while (!exp.done()) {
                bool stepCompleted = false;
                const SymbolSlots s = exp.next(stepCompleted);
                EXPECT_GE(s.duration0, 1u);
                EXPECT_LE(s.duration0, MAX_CHUNK_TICKS);
                EXPECT_GE(s.duration1, 1u);
                EXPECT_LE(s.duration1, MAX_CHUNK_TICKS);
                stepSlots.push_back({ s.level0, s.duration0 });
                stepSlots.push_back({ s.level1, s.duration1 });

                if (stepCompleted) {
                        if (segIdx >= move.segmentCount) {
                                ADD_FAILURE() << "more steps than segments declare";
                                break;
                        }
                        const uint32_t hp = move.segments[segIdx].halfPeriodTicks;
                        uint64_t high = 0;
                        uint64_t low = 0;
                        totals(stepSlots, high, low);
                        EXPECT_EQ(high, hp) << "HIGH total wrong, seg=" << +segIdx;
                        EXPECT_EQ(low, hp) << "LOW total wrong, seg=" << +segIdx;
                        EXPECT_TRUE(highBeforeLow(stepSlots)) << "edge order wrong, seg=" << +segIdx;
                        stepSlots.clear();
                        ++stepsDone;
                        if (++stepsInSeg >= move.segments[segIdx].stepCount) {
                                ++segIdx;
                                stepsInSeg = 0;
                                skipEmpty();
                        }
                }
                if (++guard > guardMax) {
                        ADD_FAILURE() << "expander failed to terminate";
                        break;
                }
        }
        return stepsDone;
}

MotionSegment seg(uint32_t steps, uint32_t hp)
{
        MotionSegment s;
        s.stepCount = steps;
        s.halfPeriodTicks = hp;
        return s;
}

} // namespace

TEST(MoveExpander, MixedFastAndWideMultiStepSegments)
{
        // Mirrors a real trapezoidal move at 10 MHz: slow (wide) accel sub-
        // segments, a fast cruise, slow (wide) decel — each with several steps.
        PlannedMove move;
        move.segments[0] = seg(3, 41322u);  // wide, multi-step accel toe
        move.segments[1] = seg(5, 8000u);   // mid ramp, fast
        move.segments[2] = seg(40, 1476u);  // fast cruise
        move.segments[3] = seg(5, 8000u);
        move.segments[4] = seg(3, 41322u);  // wide decel toe
        move.segmentCount = 5;
        move.totalSteps = 3 + 5 + 40 + 5 + 3;

        EXPECT_EQ(expandAndVerify(move), move.totalSteps);
}

TEST(MoveExpander, AllWideSingleSegment)
{
        // The regression that bit the first integration: a wide segment with
        // many steps must re-seed the cursor for every step, not just the first.
        PlannedMove move;
        move.segments[0] = seg(10, 0x20000u);
        move.segmentCount = 1;
        move.totalSteps = 10;
        EXPECT_EQ(expandAndVerify(move), 10u);
}

TEST(MoveExpander, SkipsEmptySegments)
{
        PlannedMove move;
        move.segments[0] = seg(0, 1476u); // stale/empty slot
        move.segments[1] = seg(4, 1476u);
        move.segments[2] = seg(0, 9999u); // empty
        move.segments[3] = seg(2, 0x10000u);
        move.segmentCount = 4;
        move.totalSteps = 6;
        EXPECT_EQ(expandAndVerify(move), 6u);
}

TEST(MoveExpander, EmptyMoveIsDoneImmediately)
{
        PlannedMove move;
        move.segmentCount = 0;
        move.totalSteps = 0;
        MoveSymbolExpander exp;
        exp.reset(move);
        EXPECT_TRUE(exp.done());
}
