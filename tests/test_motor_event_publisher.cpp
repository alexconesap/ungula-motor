// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

// Coverage for the fixed-capacity event broadcaster. No heap, no
// platform — pure host code.

#include <gtest/gtest.h>

#include <motor/motor_event.h>
#include <motor/motor_event_publisher.h>

namespace {

    using motor::MotorEvent;
    using motor::MotorEventPublisher;
    using motor::MotorEventType;
    using motor::MotorFsmState;

    class CountingListener : public motor::IMotorEventListener {
        public:
            int hits = 0;
            MotorEvent last{};
            void onMotorEvent(const MotorEvent& event) override {
                ++hits;
                last = event;
            }
    };

    MotorEvent make_event(MotorEventType type, int32_t pos) {
        MotorEvent e{};
        e.type = type;
        e.previousState = MotorFsmState::Idle;
        e.newState = MotorFsmState::Starting;
        e.positionSteps = pos;
        e.timestampMs = 12345;
        return e;
    }

    TEST(MotorEventPublisherTest, EmptyPublishIsNoOp) {
        MotorEventPublisher<4> pub;
        EXPECT_EQ(pub.listenerCount(), 0);
        pub.publish(make_event(MotorEventType::Started, 0));
        // No listeners → no observable side effect.
    }

    TEST(MotorEventPublisherTest, SubscribeAddsAndDeduplicates) {
        MotorEventPublisher<4> pub;
        CountingListener a;
        CountingListener b;

        EXPECT_TRUE(pub.subscribe(&a));
        EXPECT_TRUE(pub.subscribe(&b));
        EXPECT_EQ(pub.listenerCount(), 2);

        // Same pointer twice → idempotent (returns true, count unchanged).
        EXPECT_TRUE(pub.subscribe(&a));
        EXPECT_EQ(pub.listenerCount(), 2);
    }

    TEST(MotorEventPublisherTest, SubscribeRejectsNullptr) {
        MotorEventPublisher<4> pub;
        EXPECT_FALSE(pub.subscribe(nullptr));
        EXPECT_EQ(pub.listenerCount(), 0);
    }

    TEST(MotorEventPublisherTest, SubscribeRejectsWhenFull) {
        MotorEventPublisher<2> pub;
        CountingListener a, b, c;
        EXPECT_TRUE(pub.subscribe(&a));
        EXPECT_TRUE(pub.subscribe(&b));
        EXPECT_FALSE(pub.subscribe(&c));  // capacity hit
        EXPECT_EQ(pub.listenerCount(), 2);
    }

    TEST(MotorEventPublisherTest, PublishReachesEverySubscriber) {
        MotorEventPublisher<4> pub;
        CountingListener a, b, c;
        pub.subscribe(&a);
        pub.subscribe(&b);
        pub.subscribe(&c);

        pub.publish(make_event(MotorEventType::TargetReached, 100));
        EXPECT_EQ(a.hits, 1);
        EXPECT_EQ(b.hits, 1);
        EXPECT_EQ(c.hits, 1);
        EXPECT_EQ(a.last.positionSteps, 100);
        EXPECT_EQ(b.last.type, MotorEventType::TargetReached);
    }

    TEST(MotorEventPublisherTest, UnsubscribeRemovesAndShifts) {
        MotorEventPublisher<4> pub;
        CountingListener a, b, c;
        pub.subscribe(&a);
        pub.subscribe(&b);
        pub.subscribe(&c);

        // Remove the middle one — the other two should still receive.
        EXPECT_TRUE(pub.unsubscribe(&b));
        EXPECT_EQ(pub.listenerCount(), 2);

        pub.publish(make_event(MotorEventType::Stopped, 0));
        EXPECT_EQ(a.hits, 1);
        EXPECT_EQ(b.hits, 0);  // unsubscribed
        EXPECT_EQ(c.hits, 1);
    }

    TEST(MotorEventPublisherTest, UnsubscribeUnknownIsFalse) {
        MotorEventPublisher<4> pub;
        CountingListener a;
        EXPECT_FALSE(pub.unsubscribe(&a));  // not registered yet
        pub.subscribe(&a);
        EXPECT_TRUE(pub.unsubscribe(&a));
        EXPECT_FALSE(pub.unsubscribe(&a));  // already gone
    }

    TEST(MotorEventPublisherTest, ResubscribeAfterUnsubscribeWorks) {
        MotorEventPublisher<4> pub;
        CountingListener a;
        pub.subscribe(&a);
        pub.unsubscribe(&a);
        EXPECT_EQ(pub.listenerCount(), 0);

        EXPECT_TRUE(pub.subscribe(&a));
        EXPECT_EQ(pub.listenerCount(), 1);
        pub.publish(make_event(MotorEventType::Started, 0));
        EXPECT_EQ(a.hits, 1);
    }

    TEST(MotorEventPublisherTest, MultiplePublishesAccumulate) {
        MotorEventPublisher<2> pub;
        CountingListener a;
        pub.subscribe(&a);

        for (int i = 0; i < 7; ++i) {
            pub.publish(make_event(MotorEventType::StateChanged, i));
        }
        EXPECT_EQ(a.hits, 7);
        EXPECT_EQ(a.last.positionSteps, 6);
    }

}  // namespace
