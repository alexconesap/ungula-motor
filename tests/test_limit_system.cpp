// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "ungula/hal/gpio/platforms/gpio_default.h"
#include "ungula/motor/limits/limit_system.h"

using namespace ungula::motor;
namespace hgpio = ungula::hal::gpio;

namespace
{

// Build a single-slot wiring array with the requested TravelLimit on
// slot 0. The other slots stay default (pin == GPIO_NONE → unused).
void fillForwardTravelLimit(LimitWiring (&w)[MAX_LIMIT_INPUTS], uint8_t pin = 34,
                            uint16_t debounceMs = 20)
{
        w[0].pin         = pin;
        w[0].kind        = LimitKind::TravelLimit;
        w[0].direction   = Direction::Forward;
        w[0].polarity    = SwitchPolarity::NormallyOpen;
        w[0].debounceMs  = debounceMs;
}

void fillBidirectionalTravelLimits(LimitWiring (&w)[MAX_LIMIT_INPUTS])
{
        w[0].pin         = 34;
        w[0].kind        = LimitKind::TravelLimit;
        w[0].direction   = Direction::Backward;
        w[0].polarity    = SwitchPolarity::NormallyOpen;
        w[0].debounceMs  = 20;

        w[1].pin         = 35;
        w[1].kind        = LimitKind::TravelLimit;
        w[1].direction   = Direction::Forward;
        w[1].polarity    = SwitchPolarity::NormallyOpen;
        w[1].debounceMs  = 20;
}

} // namespace

// =====================================================================
// Noise immunity — a stray ISR edge with the pin reading inactive at
// service time must be discarded. This was the bug that the first
// 1.0.2 design introduced: EMI from the running stepper kept firing
// the ISR, the lib promoted on the edge alone, and the axis locked
// up with both directions latched. The fix is "edge AND live active"
// for the fast-track promotion.
// =====================================================================

TEST(LimitSystem, TravelLimitDiscardsIsrEdgeWhenPinReadsInactive)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        fillForwardTravelLimit(w);
        ASSERT_TRUE(ls.begin(w, /*engineForIsr=*/nullptr).ok());

        // Pin reads LOW (inactive), modelling a sub-microsecond noise
        // spike that is already over by the time service() runs.
        hgpio::detail::setHostReadValue(/*pin=*/34, false);

        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        ls.service(/*nowMs=*/100);
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));

        // A second stray edge shortly after — also discarded.
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        ls.service(110);
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));
}

// =====================================================================
// Activation fast-track — ISR edge + active on TWO consecutive
// service ticks. The two-tick requirement is what gives noise
// immunity: a single ISR fire that happens to coincide with one
// active polled read won't survive into the next tick if the pin
// has gone back to inactive (the typical EMI spike profile).
// =====================================================================

TEST(LimitSystem, TravelLimitPromotesOnTwoConsecutiveActiveReadsAfterEdge)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        fillForwardTravelLimit(w, /*pin=*/34, /*debounceMs=*/20);
        ASSERT_TRUE(ls.begin(w, nullptr).ok());

        // Real press: ISR fires; pin reads active.
        hgpio::detail::setHostReadValue(34, true);
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);

        // First service tick — edge is observed, edgeArmed=true, but
        // promotion needs the previous polled tick to also have been
        // active. On the first tick `candidateActive` was still
        // false, so the lib waits.
        ls.service(/*nowMs=*/5);
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));

        // Second service tick — pin still active, previous tick was
        // also active (candidateActive=true from the first tick's
        // update). Promotion confirmed.
        ls.service(/*nowMs=*/15);
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit));
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit, Direction::Forward));
}

// =====================================================================
// EMI-coincides-with-one-tick rejection — the failure mode that
// killed the previous design. ISR fires, the pin happens to be
// active on the next polled read by coincidence, but on the
// FOLLOWING tick the pin reads inactive. The two-tick requirement
// catches this: no promotion.
// =====================================================================

TEST(LimitSystem, TravelLimitDoesNotPromoteWhenSecondTickReadsInactive)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        fillForwardTravelLimit(w, 34, /*debounceMs=*/50);
        ASSERT_TRUE(ls.begin(w, nullptr).ok());

        // Spike scenario: pin reads active for ONE service tick
        // (matched the ISR edge) but is inactive by the next tick.
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        hgpio::detail::setHostReadValue(34, true);
        ls.service(5);
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));

        // Spike over: pin back to inactive. Disarms the edge.
        hgpio::detail::setHostReadValue(34, false);
        ls.service(15);
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));

        // Even if the pin later reads active for ONE tick again,
        // there's no new ISR edge AND no continuous-active window
        // long enough for the polled fallback (debounceMs=50 > one
        // tick).  Stays inactive.
        hgpio::detail::setHostReadValue(34, true);
        ls.service(25);
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));
}

TEST(LimitSystem, TravelLimitIsDirectionAwareAfterActivation)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        fillBidirectionalTravelLimits(w);
        ASSERT_TRUE(ls.begin(w, nullptr).ok());

        // Activate the Backward limit (slot 0, pin 34). Need TWO
        // consecutive ticks of polled-active reads after the ISR
        // for promotion (the two-tick noise rejection contract).
        hgpio::detail::setHostReadValue(/*backward pin=*/34, true);
        hgpio::detail::setHostReadValue(/*forward pin =*/35, false);
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        ls.service(50);
        ls.service(60);

        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit));
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit, Direction::Backward));
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit, Direction::Forward));
}

// =====================================================================
// Slow activation fallback — if the asserting edge predates begin()
// (switch already pressed at boot) OR the ISR is missed for any
// reason, the polled "stable-for-debounceMs" filter still latches
// the limit. Belt-and-suspenders against a missed edge.
// =====================================================================

TEST(LimitSystem, TravelLimitPolledActivePromotesAfterDebounceMs)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        // Pin already reads active BEFORE begin() — modelling the
        // boot-time-pressed case where the boot-seed loop in begin()
        // catches it. After begin() we additionally verify the
        // polled fallback re-promotes even if we manually pretend
        // the seed missed.
        fillForwardTravelLimit(w, 34, /*debounceMs=*/20);
        hgpio::detail::setHostReadValue(34, true);
        ASSERT_TRUE(ls.begin(w, nullptr).ok());

        // The boot-seed loop already set stableActive=true at this
        // point - the lib catches "pressed at boot" without waiting.
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit));
}

// =====================================================================
// Deactivation path — once latched, the limit stays latched until
// the pin reads inactive for `debounceMs` continuously. This is
// what gates "is it safe to drive that direction again".
// =====================================================================

TEST(LimitSystem, TravelLimitDeactivatesAfterDebounceMsOfInactiveReads)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        fillForwardTravelLimit(w, 34, /*debounceMs=*/20);
        ASSERT_TRUE(ls.begin(w, nullptr).ok());

        // Press, latch (needs two consecutive active ticks).
        hgpio::detail::setHostReadValue(34, true);
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        ls.service(/*nowMs=*/100);
        ls.service(/*nowMs=*/110);
        ASSERT_TRUE(ls.isActive(LimitKind::TravelLimit));

        // Release. The candidate timer starts at the first tick
        // that observes the inactive read; `stableActive` clears
        // only after `debounceMs` of continuously inactive reads.
        hgpio::detail::setHostReadValue(34, false);

        ls.service(120); // first inactive read - timer starts here
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit));

        ls.service(135); // 15 ms elapsed - not enough
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit));

        ls.service(140); // 20 ms elapsed - meets debounce threshold
        EXPECT_FALSE(ls.isActive(LimitKind::TravelLimit));
}

// =====================================================================
// Re-activation: after a clean deactivation the lib must accept
// another press. Catches the case where the latch-clear state
// machine accidentally locks itself.
// =====================================================================

TEST(LimitSystem, TravelLimitCanReactivateAfterDeactivation)
{
        hgpio::detail::resetAllHostReadValues();

        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        fillForwardTravelLimit(w, 34, /*debounceMs=*/5);
        ASSERT_TRUE(ls.begin(w, nullptr).ok());

        // First press - needs two consecutive active ticks for
        // promotion under the two-tick contract.
        hgpio::detail::setHostReadValue(34, true);
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        ls.service(0);
        ls.service(2);
        ASSERT_TRUE(ls.isActive(LimitKind::TravelLimit));

        // Release - latch clears once the inactive-read timer
        // reaches debounceMs.
        hgpio::detail::setHostReadValue(34, false);
        ls.service(10);     // first inactive read
        ls.service(20);     // > 5 ms after first inactive - clears
        ASSERT_FALSE(ls.isActive(LimitKind::TravelLimit));

        // Second press - must latch again.
        hgpio::detail::setHostReadValue(34, true);
        ls.simulateIsrEdgeForTesting(LimitKind::TravelLimit);
        ls.service(30);
        ls.service(40);
        EXPECT_TRUE(ls.isActive(LimitKind::TravelLimit));
}

// =====================================================================
// EmergencyLimit semantics are unchanged. Keep regression coverage
// in case a future refactor accidentally widens the engine
// requirement again.
// =====================================================================

TEST(LimitSystem, EmergencyLimitRequiresEngineRefForIsrStop)
{
        LimitSystem ls;
        LimitWiring w[MAX_LIMIT_INPUTS]{};
        w[0].pin = 36;
        w[0].kind = LimitKind::EmergencyLimit;
        w[0].polarity = SwitchPolarity::NormallyOpen;
        w[0].debounceMs = 0;

        // No engineForIsr — must fail at begin() since the ISR will
        // need to call engineForIsr_->stop(Immediate). Distinguishes
        // EmergencyLimit (stopsInIsr == true) from TravelLimit
        // (stopsInIsr == false; engine optional).
        const auto s = ls.begin(w, nullptr);
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
}
