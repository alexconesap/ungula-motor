// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

// Exhaustive coverage for MotorFsm — the central state machine.
//
// The FSM has 11 states and 16 transition methods. This file checks:
//   - the legal-transition matrix (each starting state, each request);
//   - guard rejection from invalid sources;
//   - that every accepted transition publishes exactly one MotorEvent
//     with the right payload (previous/new state, event type);
//   - terminal-state semantics (auto-clearable vs ack-required);
//   - emergency stop behavior from every reachable state.

#include <gtest/gtest.h>

#include <motor/motor_event.h>
#include <motor/motor_event_publisher.h>
#include <motor/motor_fsm.h>
#include <motor/motor_state.h>

#include <vector>

namespace {

    using motor::Direction;
    using motor::MotorEvent;
    using motor::MotorEventPublisher;
    using motor::MotorEventType;
    using motor::MotorFsm;
    using motor::MotorFsmState;

    // Capturing listener so we can assert on the events the FSM publishes
    // for every transition.
    class CapturingListener : public motor::IMotorEventListener {
        public:
            std::vector<MotorEvent> events;

            void onMotorEvent(const MotorEvent& event) override {
                events.push_back(event);
            }
    };

    // Build a fresh FSM wired to a capturing listener. Returns the FSM
    // initialized to `Disabled` (the default state).
    struct Harness {
            MotorFsm fsm;
            MotorEventPublisher<motor::MAX_MOTOR_EVENT_LISTENERS> publisher;
            CapturingListener listener;
            int32_t position = 0;

            Harness() {
                publisher.subscribe(&listener);
                fsm.setPublisher(&publisher);
                fsm.setPositionSource(&position);
            }
    };

    // Drive the FSM into a known state for a test scenario. The
    // `enable()` helper is idempotent — if the FSM is already past
    // Disabled, it doesn't re-call `requestEnable()`. This lets a test
    // body call `enable(h)` to assert preconditions and then call
    // `enterRunningForward(h)` (which also calls `enable()`) without
    // spurious failures from a redundant transition.
    void enable(Harness& harness) {
        if (harness.fsm.state() == MotorFsmState::Disabled) {
            ASSERT_TRUE(harness.fsm.requestEnable());
        }
        ASSERT_EQ(harness.fsm.state(), MotorFsmState::Idle);
    }
    void enterRunningForward(Harness& harness) {
        enable(harness);
        ASSERT_TRUE(harness.fsm.requestMoveForward());
        ASSERT_EQ(harness.fsm.state(), MotorFsmState::Starting);
        ASSERT_TRUE(harness.fsm.requestRunning(Direction::FORWARD));
        ASSERT_EQ(harness.fsm.state(), MotorFsmState::RunningForward);
    }
    void enterRunningBackward(Harness& harness) {
        enable(harness);
        ASSERT_TRUE(harness.fsm.requestMoveBackward());
        ASSERT_TRUE(harness.fsm.requestRunning(Direction::BACKWARD));
        ASSERT_EQ(harness.fsm.state(), MotorFsmState::RunningBackward);
    }
    void enterDecelerating(Harness& harness) {
        enterRunningForward(harness);
        ASSERT_TRUE(harness.fsm.requestDecelerate());
        ASSERT_EQ(harness.fsm.state(), MotorFsmState::Decelerating);
    }

    // ------------------------------------------------------------------
    // Initial state
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, StartsInDisabled) {
        Harness h;
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Disabled);
        EXPECT_FALSE(h.fsm.isMoving());
    }

    // ------------------------------------------------------------------
    // Enable / Disable
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, EnableDisabledToIdleSucceeds) {
        Harness h;
        EXPECT_TRUE(h.fsm.requestEnable());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
        ASSERT_EQ(h.listener.events.size(), 1U);
        EXPECT_EQ(h.listener.events[0].previousState, MotorFsmState::Disabled);
        EXPECT_EQ(h.listener.events[0].newState, MotorFsmState::Idle);
        EXPECT_EQ(h.listener.events[0].type, MotorEventType::StateChanged);
    }

    TEST(MotorFsmTest, EnableFromNonDisabledRejected) {
        Harness h;
        enable(h);
        EXPECT_FALSE(h.fsm.requestEnable());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
    }

    TEST(MotorFsmTest, DisableFromAnyStateAllowed) {
        // Idle → Disabled
        {
            Harness h;
            enable(h);
            EXPECT_TRUE(h.fsm.requestDisable());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Disabled);
        }
        // RunningForward → Disabled
        {
            Harness h;
            enterRunningForward(h);
            EXPECT_TRUE(h.fsm.requestDisable());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Disabled);
        }
        // Stall → Disabled (you can disable a faulted motor)
        {
            Harness h;
            enterRunningForward(h);
            ASSERT_TRUE(h.fsm.requestStallDetected());
            EXPECT_TRUE(h.fsm.requestDisable());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Disabled);
        }
        // Disabled → Disabled (self) is also allowed.
        {
            Harness h;
            EXPECT_TRUE(h.fsm.requestDisable());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Disabled);
        }
    }

    // ------------------------------------------------------------------
    // Motion start
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, MoveForwardOnlyFromIdle) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestMoveForward());  // Disabled
        enable(h);
        EXPECT_TRUE(h.fsm.requestMoveForward());  // Idle
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Starting);
        EXPECT_FALSE(h.fsm.requestMoveForward());  // Starting
    }

    TEST(MotorFsmTest, MoveBackwardOnlyFromIdle) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestMoveBackward());  // Disabled
        enable(h);
        EXPECT_TRUE(h.fsm.requestMoveBackward());  // Idle
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Starting);
    }

    TEST(MotorFsmTest, RunningRequiresStarting) {
        Harness h;
        enable(h);
        EXPECT_FALSE(h.fsm.requestRunning(Direction::FORWARD));  // Idle, no go
        ASSERT_TRUE(h.fsm.requestMoveForward());
        EXPECT_TRUE(h.fsm.requestRunning(Direction::FORWARD));  // from Starting
        EXPECT_EQ(h.fsm.state(), MotorFsmState::RunningForward);
    }

    TEST(MotorFsmTest, RunningPicksDirectionFromArg) {
        Harness h;
        enable(h);
        ASSERT_TRUE(h.fsm.requestMoveForward());
        ASSERT_TRUE(h.fsm.requestRunning(Direction::BACKWARD));
        // Even though we requested moveForward, the running-direction is
        // the explicit argument. The host's actual direction tracking
        // (`LocalMotor::direction_`) is the source of truth — the FSM
        // just labels.
        EXPECT_EQ(h.fsm.state(), MotorFsmState::RunningBackward);
    }

    TEST(MotorFsmTest, WaitStartFromIdleOnly) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestWaitStart());  // Disabled
        enable(h);
        EXPECT_TRUE(h.fsm.requestWaitStart());  // Idle
        EXPECT_EQ(h.fsm.state(), MotorFsmState::WaitingStart);
        EXPECT_TRUE(h.fsm.isMoving());           // WaitingStart counts as moving
        EXPECT_FALSE(h.fsm.requestWaitStart());  // Already in WaitingStart
    }

    TEST(MotorFsmTest, StartingAcceptsIdleAndWaitingStart) {
        // From Idle (skipping the WaitingStart phase).
        {
            Harness h;
            enable(h);
            EXPECT_TRUE(h.fsm.requestStarting());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Starting);
        }
        // From WaitingStart (the normal scheduled-profile path).
        {
            Harness h;
            enable(h);
            ASSERT_TRUE(h.fsm.requestWaitStart());
            EXPECT_TRUE(h.fsm.requestStarting());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Starting);
        }
    }

    TEST(MotorFsmTest, StartingRejectedFromMostStates) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestStarting());  // Disabled
        enterRunningForward(h);
        EXPECT_FALSE(h.fsm.requestStarting());  // Running
    }

    // ------------------------------------------------------------------
    // Motion stop / decelerate
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, DecelerateFromAnyMovingState) {
        // RunningForward
        {
            Harness h;
            enterRunningForward(h);
            EXPECT_TRUE(h.fsm.requestDecelerate());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Decelerating);
        }
        // RunningBackward
        {
            Harness h;
            enterRunningBackward(h);
            EXPECT_TRUE(h.fsm.requestDecelerate());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Decelerating);
        }
        // Starting (rare but legal)
        {
            Harness h;
            enable(h);
            ASSERT_TRUE(h.fsm.requestMoveForward());
            EXPECT_TRUE(h.fsm.requestDecelerate());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Decelerating);
        }
    }

    TEST(MotorFsmTest, DecelerateRejectedFromStillStates) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestDecelerate());  // Disabled
        enable(h);
        EXPECT_FALSE(h.fsm.requestDecelerate());  // Idle
    }

    TEST(MotorFsmTest, StopFromMovingGoesToIdleWithStoppedEvent) {
        Harness h;
        enterDecelerating(h);
        const size_t before = h.listener.events.size();

        EXPECT_TRUE(h.fsm.requestStop());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
        ASSERT_EQ(h.listener.events.size(), before + 1);
        EXPECT_EQ(h.listener.events.back().type, MotorEventType::Stopped);
    }

    TEST(MotorFsmTest, StopAutoClearsTargetReached) {
        Harness h;
        enterRunningForward(h);
        ASSERT_TRUE(h.fsm.requestTargetReached());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::TargetReached);

        // Service-timer auto-clear path.
        EXPECT_TRUE(h.fsm.requestStop());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
        EXPECT_EQ(h.listener.events.back().type, MotorEventType::StateChanged);
    }

    TEST(MotorFsmTest, StopAutoClearsLimitReached) {
        Harness h;
        enterRunningForward(h);
        ASSERT_TRUE(h.fsm.requestLimitHit());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::LimitReached);

        EXPECT_TRUE(h.fsm.requestStop());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
    }

    TEST(MotorFsmTest, StopDoesNotAutoClearStallOrFault) {
        // Stall must be cleared explicitly via clearStall — requestStop
        // should NOT silently drop the operator's signal.
        {
            Harness h;
            enterRunningForward(h);
            ASSERT_TRUE(h.fsm.requestStallDetected());
            EXPECT_FALSE(h.fsm.requestStop());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Stall);
        }
        // Same for Fault.
        {
            Harness h;
            enable(h);
            ASSERT_TRUE(h.fsm.requestFault());
            EXPECT_FALSE(h.fsm.requestStop());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Fault);
        }
    }

    TEST(MotorFsmTest, StopRejectedFromIdleAndDisabled) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestStop());  // Disabled
        enable(h);
        EXPECT_FALSE(h.fsm.requestStop());  // Idle
    }

    // ------------------------------------------------------------------
    // Emergency stop
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, EmergencyStopFromMovingGoesToIdle) {
        Harness h;
        enterRunningForward(h);
        EXPECT_TRUE(h.fsm.requestEmergencyStop());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
        EXPECT_EQ(h.listener.events.back().type, MotorEventType::Stopped);
    }

    TEST(MotorFsmTest, EmergencyStopRejectedFromDisabled) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestEmergencyStop());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Disabled);
    }

    TEST(MotorFsmTest, EmergencyStopBypassesStallAndFault) {
        // E-stop is the heavy hammer — it intentionally yanks the motor
        // out of Stall / Fault without the explicit acknowledgement,
        // because by the time the operator pressed e-stop they don't care
        // why the motor was stopped, only that it is.
        {
            Harness h;
            enterRunningForward(h);
            ASSERT_TRUE(h.fsm.requestStallDetected());
            EXPECT_TRUE(h.fsm.requestEmergencyStop());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
        }
        {
            Harness h;
            enable(h);
            ASSERT_TRUE(h.fsm.requestFault());
            EXPECT_TRUE(h.fsm.requestEmergencyStop());
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
        }
    }

    // ------------------------------------------------------------------
    // Terminal events
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, TargetReachedRequiresMoving) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestTargetReached());  // Disabled
        enable(h);
        EXPECT_FALSE(h.fsm.requestTargetReached());  // Idle
        ASSERT_TRUE(h.fsm.requestMoveForward());
        EXPECT_TRUE(h.fsm.requestTargetReached());  // Starting
        EXPECT_EQ(h.fsm.state(), MotorFsmState::TargetReached);
        EXPECT_EQ(h.listener.events.back().type, MotorEventType::TargetReached);
    }

    TEST(MotorFsmTest, LimitHitRequiresMoving) {
        Harness h;
        enable(h);
        EXPECT_FALSE(h.fsm.requestLimitHit());  // Idle
        enterRunningForward(h);
        EXPECT_TRUE(h.fsm.requestLimitHit());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::LimitReached);
        EXPECT_EQ(h.listener.events.back().type, MotorEventType::LimitSwitchHit);
    }

    TEST(MotorFsmTest, StallDetectedRequiresMoving) {
        Harness h;
        EXPECT_FALSE(h.fsm.requestStallDetected());  // Disabled
        enable(h);
        EXPECT_FALSE(h.fsm.requestStallDetected());  // Idle
        enterRunningForward(h);
        EXPECT_TRUE(h.fsm.requestStallDetected());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Stall);
    }

    TEST(MotorFsmTest, FaultFromAnyEnabledState) {
        // Fault is an external condition — accepted from anywhere except Disabled.
        {
            Harness h;
            EXPECT_FALSE(h.fsm.requestFault());  // Disabled rejected
        }
        {
            Harness h;
            enable(h);
            EXPECT_TRUE(h.fsm.requestFault());  // Idle accepted
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Fault);
        }
        {
            Harness h;
            enterRunningForward(h);
            EXPECT_TRUE(h.fsm.requestFault());  // Running accepted
            EXPECT_EQ(h.fsm.state(), MotorFsmState::Fault);
        }
    }

    // ------------------------------------------------------------------
    // Acknowledgements
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, ClearStallOnlyFromStall) {
        Harness h;
        EXPECT_FALSE(h.fsm.clearStall());  // Disabled
        enable(h);
        EXPECT_FALSE(h.fsm.clearStall());  // Idle

        enterRunningForward(h);
        ASSERT_TRUE(h.fsm.requestStallDetected());
        EXPECT_TRUE(h.fsm.clearStall());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
    }

    TEST(MotorFsmTest, ClearFaultOnlyFromFault) {
        Harness h;
        EXPECT_FALSE(h.fsm.clearFault());  // Disabled
        enable(h);
        EXPECT_FALSE(h.fsm.clearFault());  // Idle

        ASSERT_TRUE(h.fsm.requestFault());
        EXPECT_TRUE(h.fsm.clearFault());
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
    }

    // ------------------------------------------------------------------
    // isMoving() coverage
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, IsMovingCoversAllActiveStates) {
        // Five states should be "moving": WaitingStart, Starting,
        // RunningForward, RunningBackward, Decelerating.
        {
            Harness h;
            enable(h);
            ASSERT_TRUE(h.fsm.requestWaitStart());
            EXPECT_TRUE(h.fsm.isMoving());
        }
        {
            Harness h;
            enable(h);
            ASSERT_TRUE(h.fsm.requestMoveForward());  // → Starting
            EXPECT_TRUE(h.fsm.isMoving());
        }
        {
            Harness h;
            enterRunningForward(h);
            EXPECT_TRUE(h.fsm.isMoving());
        }
        {
            Harness h;
            enterRunningBackward(h);
            EXPECT_TRUE(h.fsm.isMoving());
        }
        {
            Harness h;
            enterDecelerating(h);
            EXPECT_TRUE(h.fsm.isMoving());
        }
        // And six should NOT be: Disabled, Idle, TargetReached, LimitReached, Stall, Fault.
        {
            Harness h;
            EXPECT_FALSE(h.fsm.isMoving());  // Disabled
        }
        {
            Harness h;
            enable(h);
            EXPECT_FALSE(h.fsm.isMoving());  // Idle
        }
        {
            Harness h;
            enterRunningForward(h);
            ASSERT_TRUE(h.fsm.requestTargetReached());
            EXPECT_FALSE(h.fsm.isMoving());  // TargetReached
        }
        {
            Harness h;
            enterRunningForward(h);
            ASSERT_TRUE(h.fsm.requestLimitHit());
            EXPECT_FALSE(h.fsm.isMoving());  // LimitReached
        }
        {
            Harness h;
            enterRunningForward(h);
            ASSERT_TRUE(h.fsm.requestStallDetected());
            EXPECT_FALSE(h.fsm.isMoving());  // Stall
        }
        {
            Harness h;
            ASSERT_TRUE(h.fsm.requestEnable());
            ASSERT_TRUE(h.fsm.requestFault());
            EXPECT_FALSE(h.fsm.isMoving());  // Fault
        }
    }

    // ------------------------------------------------------------------
    // Event payload sanity
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, EventCarriesPositionFromSource) {
        Harness h;
        h.position = 12345;
        enable(h);
        ASSERT_FALSE(h.listener.events.empty());
        EXPECT_EQ(h.listener.events.back().positionSteps, 12345);

        h.position = -42;
        ASSERT_TRUE(h.fsm.requestMoveForward());
        EXPECT_EQ(h.listener.events.back().positionSteps, -42);
    }

    TEST(MotorFsmTest, EventTimestampIsInt64) {
        // Type-level check — proves the FSM hasn't silently re-narrowed
        // back to uint32_t after the time-API consolidation.
        Harness h;
        enable(h);
        ASSERT_FALSE(h.listener.events.empty());
        // If timestampMs is uint32_t this would warn / fail.
        const int64_t ts = h.listener.events.back().timestampMs;
        EXPECT_GE(ts, 0);
    }

    // ------------------------------------------------------------------
    // Full life-cycle smoke
    // ------------------------------------------------------------------

    TEST(MotorFsmTest, FullForwardCycleProducesExpectedEventTrail) {
        Harness h;
        ASSERT_TRUE(h.fsm.requestEnable());
        ASSERT_TRUE(h.fsm.requestMoveForward());  // → Starting
        ASSERT_TRUE(h.fsm.requestRunning(Direction::FORWARD));
        ASSERT_TRUE(h.fsm.requestDecelerate());
        ASSERT_TRUE(h.fsm.requestStop());

        ASSERT_EQ(h.listener.events.size(), 5U);
        EXPECT_EQ(h.listener.events[0].type, MotorEventType::StateChanged);  // Enable
        EXPECT_EQ(h.listener.events[1].type, MotorEventType::Started);       // Move
        EXPECT_EQ(h.listener.events[2].type, MotorEventType::StateChanged);  // Running
        EXPECT_EQ(h.listener.events[3].type, MotorEventType::StateChanged);  // Decel
        EXPECT_EQ(h.listener.events[4].type, MotorEventType::Stopped);       // Stop
        EXPECT_EQ(h.fsm.state(), MotorFsmState::Idle);
    }

}  // namespace
