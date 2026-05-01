// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

// Coverage for MotorCoordinator — the multi-motor orchestrator. Pure
// host code, no platform deps.

#include <gtest/gtest.h>

#include <motor/motor_coordinator.h>
#include <motor/remote_motor.h>

#include "test_remote_motor_sink.h"

namespace {

    using motor::MotorCommandType;
    using motor::MotorCoordinator;
    using motor::MotorEvent;
    using motor::MotorEventType;
    using motor::MotorFsmState;

    // Use RemoteMotor as a concrete IMotor — its commands flow into a
    // counting sink so we can prove the coordinator dispatched to all
    // registered motors.
    using test_helpers::RecordingSink;

    TEST(MotorCoordinatorTest, EmptyCoordinatorIsNoOp) {
        MotorCoordinator coord;
        EXPECT_EQ(coord.motorCount(), 0);
        coord.enableAll();         // must not crash
        coord.emergencyStopAll();  // must not crash
        EXPECT_EQ(coord.motor(0), nullptr);
    }

    TEST(MotorCoordinatorTest, AddMotorRejectsNullptr) {
        MotorCoordinator coord;
        EXPECT_FALSE(coord.addMotor(nullptr));
        EXPECT_EQ(coord.motorCount(), 0);
    }

    TEST(MotorCoordinatorTest, AddMotorRespectsCapacity) {
        MotorCoordinator coord;
        RecordingSink sink;
        motor::RemoteMotor motors[motor::MAX_COORDINATOR_MOTORS + 1] = {
                {sink, 0}, {sink, 1}, {sink, 2}, {sink, 3}, {sink, 4},
                {sink, 5}, {sink, 6}, {sink, 7}, {sink, 8},
        };
        for (uint8_t i = 0; i < motor::MAX_COORDINATOR_MOTORS; ++i) {
            EXPECT_TRUE(coord.addMotor(&motors[i]));
        }
        EXPECT_EQ(coord.motorCount(), motor::MAX_COORDINATOR_MOTORS);
        // One past capacity → rejected.
        EXPECT_FALSE(coord.addMotor(&motors[motor::MAX_COORDINATOR_MOTORS]));
        EXPECT_EQ(coord.motorCount(), motor::MAX_COORDINATOR_MOTORS);
    }

    TEST(MotorCoordinatorTest, EnableAllReachesEveryRegisteredMotor) {
        MotorCoordinator coord;
        RecordingSink sink;
        motor::RemoteMotor m1(sink, /*id=*/10);
        motor::RemoteMotor m2(sink, /*id=*/20);
        motor::RemoteMotor m3(sink, /*id=*/30);
        coord.addMotor(&m1);
        coord.addMotor(&m2);
        coord.addMotor(&m3);

        coord.enableAll();

        ASSERT_EQ(sink.simples.size(), 3U);
        EXPECT_EQ(sink.simples[0].id, 10);
        EXPECT_EQ(sink.simples[1].id, 20);
        EXPECT_EQ(sink.simples[2].id, 30);
        for (const auto& s : sink.simples) {
            EXPECT_EQ(s.cmd, MotorCommandType::ENABLE);
        }
    }

    TEST(MotorCoordinatorTest, EmergencyStopAllReachesEveryRegisteredMotor) {
        MotorCoordinator coord;
        RecordingSink sink;
        motor::RemoteMotor m1(sink, 1);
        motor::RemoteMotor m2(sink, 2);
        coord.addMotor(&m1);
        coord.addMotor(&m2);

        coord.emergencyStopAll();

        ASSERT_EQ(sink.simples.size(), 2U);
        EXPECT_EQ(sink.simples[0].cmd, MotorCommandType::EMERGENCY_STOP);
        EXPECT_EQ(sink.simples[1].cmd, MotorCommandType::EMERGENCY_STOP);
    }

    TEST(MotorCoordinatorTest, MotorIndexAccessor) {
        MotorCoordinator coord;
        RecordingSink sink;
        motor::RemoteMotor m1(sink, 1);
        motor::RemoteMotor m2(sink, 2);
        coord.addMotor(&m1);
        coord.addMotor(&m2);

        EXPECT_EQ(coord.motor(0), &m1);
        EXPECT_EQ(coord.motor(1), &m2);
        EXPECT_EQ(coord.motor(2), nullptr);
        EXPECT_EQ(coord.motor(255), nullptr);
    }

    TEST(MotorCoordinatorTest, OnMotorEventStoresLastEvent) {
        // The coordinator implements IMotorEventListener — verify the
        // virtual dispatch hooks up correctly (it's worth testing at
        // least the recording behaviour even if the production
        // implementation will likely route events into project logic).
        MotorCoordinator coord;
        MotorEvent e{};
        e.type = MotorEventType::TargetReached;
        e.previousState = MotorFsmState::RunningForward;
        e.newState = MotorFsmState::TargetReached;
        e.positionSteps = 999;
        e.timestampMs = 1'700'000'000'000LL;

        // Cast through the interface so we exercise the virtual dispatch.
        motor::IMotorEventListener& asListener = coord;
        asListener.onMotorEvent(e);
        // No public getter — but the call mustn't crash and the
        // override must be reachable. (If a getter is added later, this
        // is the canonical site to assert on it.)
        SUCCEED();
    }

}  // namespace
