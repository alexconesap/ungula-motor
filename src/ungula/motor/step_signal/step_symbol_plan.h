// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motion_segment.h"

// Pure, hardware-free helper that turns one step's half-period into a
// stream of RMT-symbol slots. Kept out of `rmt_step_signal.cpp` (which is
// ESP-IDF only) so it can be unit-tested on the host.
//
// The RMT symbol duration field is 15 bits, so a single (level, duration)
// slot can hold at most 0x7FFF ticks. When the generator runs at a high
// resolution (e.g. 10 MHz for the YPMC servo), the slow ends of an
// acceleration ramp need half-periods well above 0x7FFF — a single symbol
// can't represent them. This cursor splits one step (`hp` ticks HIGH then
// `hp` ticks LOW) into as many <= 0x7FFF chunks as needed and packs two
// chunks per symbol.
//
// Both the HIGH run and the LOW run are exactly `hp` ticks, so they chunk
// into the same count and the total chunk count is always even — every
// symbol gets two non-zero slots. The lone-chunk split below is defensive
// only.

namespace ungula::motor::rmtgen
{

// Largest value an RMT 15-bit duration slot can hold.
constexpr uint16_t MAX_CHUNK_TICKS = 0x7FFFu;

// One RMT symbol expressed without any ESP-IDF type dependency. Mirrors
// the two (level, duration) slots of `rmt_symbol_word_t`.
struct SymbolSlots {
        uint16_t duration0 = 0;
        uint16_t duration1 = 0;
        uint8_t level0 = 0;
        uint8_t level1 = 0;
};

// Emits the symbol stream for ONE step: `hp` ticks HIGH followed by `hp`
// ticks LOW. Resumable — call next() until done(). Every emitted duration
// is in [1, MAX_CHUNK_TICKS] and no slot is ever zero, which the RMT
// driver requires (a zero duration is an end-marker).
//
// For the common case hp <= MAX_CHUNK_TICKS the very first next() returns
// the classic {HIGH:hp, LOW:hp} symbol and done() is then true — identical
// to the pre-split single-symbol-per-step path.
class StepSymbolCursor
{
public:
        void reset(uint32_t hp)
        {
                highLeft_ = hp;
                lowLeft_ = hp;
        }

        bool done() const { return highLeft_ == 0u && lowLeft_ == 0u; }

        // Precondition: !done(). Builds the next symbol, consuming up to
        // two chunks (HIGH run first, then LOW run).
        SymbolSlots next()
        {
                SymbolSlots s;
                takeSlot(s.level0, s.duration0);
                if (done()) {
                        // Only one chunk remained for the whole step. Split it
                        // so slot 1 stays non-zero. Unreachable while the HIGH
                        // and LOW runs are equal length (even chunk count), but
                        // kept as a safety net for the ISR consumer.
                        splitLoneChunk(s);
                } else {
                        takeSlot(s.level1, s.duration1);
                }
                return s;
        }

private:
        void takeSlot(uint8_t &level, uint16_t &duration)
        {
                if (highLeft_ > 0u) {
                        level = 1u;
                        const uint32_t c = (highLeft_ < MAX_CHUNK_TICKS) ? highLeft_ : MAX_CHUNK_TICKS;
                        duration = static_cast<uint16_t>(c);
                        highLeft_ -= c;
                } else {
                        level = 0u;
                        const uint32_t c = (lowLeft_ < MAX_CHUNK_TICKS) ? lowLeft_ : MAX_CHUNK_TICKS;
                        duration = static_cast<uint16_t>(c);
                        lowLeft_ -= c;
                }
        }

        static void splitLoneChunk(SymbolSlots &s)
        {
                s.level1 = s.level0;
                if (s.duration0 >= 2u) {
                        const uint16_t half = static_cast<uint16_t>(s.duration0 / 2u);
                        s.duration1 = static_cast<uint16_t>(s.duration0 - half);
                        s.duration0 = half;
                } else {
                        // 1-tick remainder: duplicate it (one extra tick, ~0.1 us
                        // at 10 MHz) rather than emit a zero-duration slot.
                        s.duration1 = 1u;
                }
        }

        uint32_t highLeft_ = 0u;
        uint32_t lowLeft_ = 0u;
};

// Walks a whole PlannedMove and yields its RMT symbol stream one symbol at
// a time (pull model, for the streaming RMT encoder which refills the
// channel memory across many callbacks). Each segment contributes
// `stepCount` steps; each step is `halfPeriodTicks` HIGH then LOW, expanded
// through StepSymbolCursor so any half-period — including the slow ends of
// a ramp that overflow a single 15-bit slot — is handled.
//
// The encoder holds the last symbol while the RMT memory block is full and
// re-pushes it, so this class is pull-only: it advances exactly one symbol
// per next() and never looks ahead.
class MoveSymbolExpander
{
public:
        void reset(const PlannedMove &move)
        {
                move_ = &move;
                segment_ = 0;
                stepsInSegment_ = 0;
                stepSeeded_ = false;
                done_ = (move.segmentCount == 0 || move.totalSteps == 0);
                skipEmptySegments();
        }

        bool done() const { return done_; }

        // Build the next symbol. Precondition: !done(). `stepCompleted` is
        // set true when this symbol is the LAST of its step (the caller bumps
        // its position counter once the symbol is actually transmitted).
        SymbolSlots next(bool &stepCompleted)
        {
                if (!stepSeeded_) {
                        cursor_.reset(move_->segments[segment_].halfPeriodTicks);
                        stepSeeded_ = true;
                }
                const SymbolSlots s = cursor_.next();
                stepCompleted = cursor_.done();
                if (stepCompleted) {
                        stepSeeded_ = false;
                        ++stepsInSegment_;
                        if (stepsInSegment_ >= move_->segments[segment_].stepCount) {
                                ++segment_;
                                stepsInSegment_ = 0;
                                skipEmptySegments();
                        }
                }
                return s;
        }

private:
        // Step past any zero-step segments and flag done at the end of the move.
        void skipEmptySegments()
        {
                while (segment_ < move_->segmentCount &&
                       move_->segments[segment_].stepCount == 0u) {
                        ++segment_;
                }
                if (segment_ >= move_->segmentCount) {
                        done_ = true;
                }
        }

        const PlannedMove *move_ = nullptr;
        uint8_t segment_ = 0;
        uint32_t stepsInSegment_ = 0;
        StepSymbolCursor cursor_{};
        bool stepSeeded_ = false;
        bool done_ = true;
};

} // namespace ungula::motor::rmtgen
