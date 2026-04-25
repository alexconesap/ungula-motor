// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

// Coverage for RemoteMotor — proves every IMotor command translates to
// the correct typed message on the IMotorCommandSink, and that
// updateState() drives both `state()` and `isMoving()` correctly.

#include <gtest/gtest.h>

#include <motor/remote_motor.h>

#include "test_remote_motor_sink.h"

namespace {

    using motor::DistanceUnit;
    using motor::MotionProfileSpec;
    using motor::MotorCommandType;
    using motor::MotorFsmState;
    using motor::RemoteMotor;
    using test_helpers::RecordingSink;

    // ---- Initial state ----

    TEST(RemoteMotorTest, StartsDisabledZeroPositionNotMoving) {
        RecordingSink sink;
        RemoteMotor m(sink, /*motorId=*/3);
        EXPECT_EQ(m.state(), MotorFsmState::Disabled);
        EXPECT_EQ(m.positionSteps(), 0);
        EXPECT_FALSE(m.isMoving());
    }

    // ---- Simple commands carry the motorId ----

    TEST(RemoteMotorTest, SimpleCommandsSendCorrectType) {
        RecordingSink sink;
        RemoteMotor m(sink, /*motorId=*/7);

        m.enable();
        m.disable();
        m.moveForward();
        m.moveBackward();
        m.stop();
        m.emergencyStop();

        ASSERT_EQ(sink.simples.size(), 6U);
        EXPECT_EQ(sink.simples[0].cmd, MotorCommandType::ENABLE);
        EXPECT_EQ(sink.simples[1].cmd, MotorCommandType::DISABLE);
        EXPECT_EQ(sink.simples[2].cmd, MotorCommandType::MOVE_FORWARD);
        EXPECT_EQ(sink.simples[3].cmd, MotorCommandType::MOVE_BACKWARD);
        EXPECT_EQ(sink.simples[4].cmd, MotorCommandType::STOP);
        EXPECT_EQ(sink.simples[5].cmd, MotorCommandType::EMERGENCY_STOP);

        for (const auto& s : sink.simples) {
            EXPECT_EQ(s.id, 7);
        }
    }

    // ---- Move commands carry the typed parameters ----

    TEST(RemoteMotorTest, MoveToCarriesValueAndUnit) {
        RecordingSink sink;
        RemoteMotor m(sink, /*motorId=*/2);

        m.moveTo(10.5F, DistanceUnit::MM);

        ASSERT_EQ(sink.moves.size(), 1U);
        EXPECT_EQ(sink.moves[0].id, 2);
        EXPECT_EQ(sink.moves[0].cmd, MotorCommandType::MOVE_TO);
        EXPECT_FLOAT_EQ(sink.moves[0].params.value, 10.5F);
        EXPECT_EQ(sink.moves[0].params.unit, DistanceUnit::MM);
    }

    TEST(RemoteMotorTest, MoveByCarriesNegativeDelta) {
        RecordingSink sink;
        RemoteMotor m(sink, /*motorId=*/2);

        m.moveBy(-25.0F, DistanceUnit::DEGREES);

        ASSERT_EQ(sink.moves.size(), 1U);
        EXPECT_EQ(sink.moves[0].cmd, MotorCommandType::MOVE_BY);
        EXPECT_FLOAT_EQ(sink.moves[0].params.value, -25.0F);
        EXPECT_EQ(sink.moves[0].params.unit, DistanceUnit::DEGREES);
    }

    TEST(RemoteMotorTest, ExecuteProfileSendsTheSpec) {
        RecordingSink sink;
        RemoteMotor m(sink, /*motorId=*/9);

        MotionProfileSpec spec{};
        spec.startTimeMs = 1'700'000'000'000LL;   // post-int64 widening
        spec.targetPosition = 12345;
        spec.maxVelocitySps = 4000;
        m.executeProfile(spec);

        ASSERT_EQ(sink.profiles.size(), 1U);
        EXPECT_EQ(sink.profiles[0].id, 9);
        EXPECT_EQ(sink.profiles[0].profile.startTimeMs, 1'700'000'000'000LL);
        EXPECT_EQ(sink.profiles[0].profile.targetPosition, 12345);
        EXPECT_EQ(sink.profiles[0].profile.maxVelocitySps, 4000);
    }

    // ---- updateState() drives state() / isMoving() / positionSteps() ----

    TEST(RemoteMotorTest, UpdateStateRefreshesCachedReads) {
        RecordingSink sink;
        RemoteMotor m(sink, 0);
        m.updateState(MotorFsmState::RunningForward, 555);
        EXPECT_EQ(m.state(), MotorFsmState::RunningForward);
        EXPECT_EQ(m.positionSteps(), 555);
        EXPECT_TRUE(m.isMoving());

        m.updateState(MotorFsmState::Idle, 555);
        EXPECT_FALSE(m.isMoving());
    }

    TEST(RemoteMotorTest, IsMovingMatchesAllRunningStates) {
        RecordingSink sink;
        RemoteMotor m(sink, 0);
        const MotorFsmState moving[] = {
                MotorFsmState::WaitingStart, MotorFsmState::Starting,
                MotorFsmState::RunningForward, MotorFsmState::RunningBackward,
                MotorFsmState::Decelerating,
        };
        for (auto s : moving) {
            m.updateState(s, 0);
            EXPECT_TRUE(m.isMoving()) << "state " << static_cast<int>(s);
        }
        const MotorFsmState still[] = {
                MotorFsmState::Disabled,      MotorFsmState::Idle,
                MotorFsmState::TargetReached, MotorFsmState::LimitReached,
                MotorFsmState::Stall,         MotorFsmState::Fault,
        };
        for (auto s : still) {
            m.updateState(s, 0);
            EXPECT_FALSE(m.isMoving()) << "state " << static_cast<int>(s);
        }
    }

    // ---- Black-box flags: documented as conservative defaults for proxies ----

    TEST(RemoteMotorTest, BlackBoxFlagsAreFalseUntilProtocolGrows) {
        // Until the wire protocol carries them, the proxy is honest about
        // not knowing.
        RecordingSink sink;
        RemoteMotor m(sink, 0);
        EXPECT_FALSE(m.wasLimitHit());
        EXPECT_FALSE(m.isHoming());
        EXPECT_FALSE(m.isHomed());
    }

}  // namespace
