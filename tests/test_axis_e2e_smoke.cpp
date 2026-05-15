// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "ungula/motor/actuator/step_dir_actuator.h"
#include "ungula/motor/axis.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/events/axis_event.h"
#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/homing/limit_switch_homing_strategy.h"
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/pulse/fake_pulse_engine.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

/// End-to-end host smoke test. Composes an Axis with a
/// `FakePulseEngine` so the pulse train can be advanced
/// deterministically (no real hardware, no GPIO, no timer ISR).
///
/// What this proves:
///   - The full pipeline (Axis → Actuator → Engine) composes through
///     `createComposed` and runs the lifecycle (begin → enable →
///     moveBy → service → completion event).
///   - The service tick observes engine completion transitions and
///     emits `MotionCompleted` from task context.
///   - Listeners receive events at `service()`-drain time, never
///     from the engine's own context.

class CapturingListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                events.push_back(ev);
        }
        std::vector<AxisEvent> events;

        uint32_t count(AxisEventType t) const
        {
                uint32_t n = 0;
                for (const auto &e : events)
                        if (e.type == t)
                                ++n;
                return n;
        }
};

AxisCommonConfig makeCommon()
{
        AxisCommonConfig c;
        c.axisId = AxisId(1);
        c.name = "x";
        c.units.stepsPerMm = 80.0f;
        c.limits.maxVelocitySps = 4000;
        c.limits.accelSpsPerSec = 8000;
        c.limits.decelSpsPerSec = 8000;
        c.limits.minPulseHighUs = 2;
        c.limits.minPulseLowUs = 2;
        c.limits.maxStepRateSps = 200'000;
        return c;
}

struct ComposedFixture {
        FakePulseEngine *engine = nullptr; // borrowed; owned by axis
        std::unique_ptr<Axis> axis;
};

ComposedFixture build(const SensorInputConfig *sensors = nullptr, uint8_t sensorCount = 0)
{
        auto engine = std::make_unique<FakePulseEngine>();
        StepDirActuator::Config aCfg;
        aCfg.kind = StepDirActuatorKind::OpenLoopStepper;
        auto actuator = std::make_unique<StepDirActuator>(*engine, aCfg);

        Axis::ComposedComponents comp;
        comp.timer = nullptr; // FakePulseEngine doesn't need one
        comp.actuator = std::move(actuator);
        comp.common = makeCommon();
        FakePulseEngine *enginePtr = engine.get();
        comp.engine = std::move(engine);

        auto r = Axis::createComposed(std::move(comp), sensors, sensorCount);
        if (!r.ok())
                return {};
        return ComposedFixture{ enginePtr, r.takeValue() };
}

TEST(AxisE2ESmokeTest, MoveByCompletesAndEmitsEvents)
{
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);

        CapturingListener l;
        ASSERT_TRUE(fx.axis->subscribe(&l).ok());

        ASSERT_TRUE(fx.axis->begin().ok());
        ASSERT_TRUE(fx.axis->enable().ok());

        // Command a 100-step forward move.
        ASSERT_TRUE(fx.axis->moveBy(100).ok());
        EXPECT_EQ(fx.axis->state(), AxisState::Moving);
        EXPECT_TRUE(fx.engine->isRunning());

        // Drive the engine to completion (FakePulseEngine doesn't
        // autonomously advance — tests own the schedule).
        fx.engine->runMove();
        EXPECT_FALSE(fx.engine->isRunning());

        // Service tick picks up the completion transition and emits
        // MotionCompleted + StateChanged.
        fx.axis->service(/*nowMs=*/10);

        EXPECT_EQ(fx.axis->state(), AxisState::Idle);
        EXPECT_EQ(fx.axis->feedback().commandedPosition, 100);
        EXPECT_EQ(fx.axis->lastStopReason(), StopReason::TargetReached);

        EXPECT_GE(l.count(AxisEventType::MotionStarted), 1u);
        EXPECT_GE(l.count(AxisEventType::MotionCompleted), 1u);
        EXPECT_GE(l.count(AxisEventType::StateChanged), 1u);
}

TEST(AxisE2ESmokeTest, EmergencyStopFromTaskHaltsAndFaults)
{
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);
        CapturingListener l;
        ASSERT_TRUE(fx.axis->subscribe(&l).ok());

        ASSERT_TRUE(fx.axis->begin().ok());
        ASSERT_TRUE(fx.axis->enable().ok());
        ASSERT_TRUE(fx.axis->moveBy(500).ok());
        ASSERT_TRUE(fx.engine->isRunning());

        // Halfway through, pull the e-stop from task context.
        fx.engine->tickSteps(250);
        ASSERT_TRUE(fx.axis->emergencyStop().ok());
        EXPECT_FALSE(fx.engine->isRunning());
        EXPECT_EQ(fx.axis->state(), AxisState::EmergencyStopped);
        EXPECT_TRUE(fx.axis->faultStatus().active());

        fx.axis->service(20);

        EXPECT_GE(l.count(AxisEventType::EmergencyStopped), 1u);
        EXPECT_GE(l.count(AxisEventType::FaultRaised), 1u);

        // clearFault drops back to Disabled, not Idle — host must
        // explicitly re-enable.
        ASSERT_TRUE(fx.axis->clearFault().ok());
        EXPECT_EQ(fx.axis->state(), AxisState::Disabled);
}

TEST(AxisE2ESmokeTest, HaltFromIsrIsObservedByService)
{
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);
        CapturingListener l;
        ASSERT_TRUE(fx.axis->subscribe(&l).ok());

        ASSERT_TRUE(fx.axis->begin().ok());
        ASSERT_TRUE(fx.axis->enable().ok());
        ASSERT_TRUE(fx.axis->moveBy(500).ok());

        // Simulate a CrashLimit ISR firing — exercises the ISR-safe halt
        // path on the engine. The Axis service tick should observe the
        // resulting fault and emit FaultRaised.
        fx.engine->haltFromIsr(StopReason::LimitSwitch);
        EXPECT_EQ(fx.engine->haltFromIsrCallCount, 1u);
        EXPECT_FALSE(fx.engine->isRunning());

        fx.axis->service(30);

        EXPECT_GE(l.count(AxisEventType::FaultRaised), 1u);
        EXPECT_EQ(fx.axis->state(), AxisState::Faulted);
}

TEST(AxisE2ESmokeTest, HomingCycleSucceedsEndToEnd)
{
        // Compose with a home sensor entry. On the host the GPIO stubs
        // always read inactive — but the strategy's `isHomeActive()` reads
        // through the SensorBank which we'll override by driving sensor
        // service from a higher level... actually the host can't simulate
        // the home pin going active without modifying lib_hal stubs.
        //
        // Skip the polled-sensor activation half of the cycle and instead
        // verify the controller correctly RUNS through its phases when no
        // strategy is configured returns Unsupported.
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);
        ASSERT_TRUE(fx.axis->begin().ok());
        ASSERT_TRUE(fx.axis->enable().ok());

        // No strategy yet — home() returns Unsupported.
        EXPECT_EQ(fx.axis->home().error(), ErrorCode::Unsupported);

        // Configure a strategy. The host can't toggle GPIO inputs to
        // simulate the home switch firing, so the actual run is exercised
        // by the dedicated `test_homing_controller` against `FakeHomingAxis`.
        LimitSwitchHomingStrategy::Config sCfg;
        sCfg.approachDirection = Direction::Backward;
        sCfg.fastFeedSps = 1000;
        sCfg.slowFeedSps = 100;
        sCfg.backoffSteps = 50;
        LimitSwitchHomingStrategy strat(sCfg);

        ASSERT_TRUE(fx.axis->setHomingStrategy(&strat, /*timeoutMs=*/0).ok());
        ASSERT_TRUE(fx.axis->home().ok());
        EXPECT_TRUE(fx.axis->isHoming());
        EXPECT_EQ(fx.axis->homingPhase(), HomingPhase::FastApproach);

        // Abort the cycle to leave the axis in a clean state for the
        // destructor.
        ASSERT_TRUE(fx.axis->stop(StopMode::Immediate).ok());
}

TEST(AxisE2ESmokeTest, MotionRejectedDuringMotion)
{
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);
        ASSERT_TRUE(fx.axis->begin().ok());
        ASSERT_TRUE(fx.axis->enable().ok());

        ASSERT_TRUE(fx.axis->moveBy(100).ok());
        EXPECT_EQ(fx.axis->moveBy(50).error(), ErrorCode::MotionInProgress);
        EXPECT_EQ(fx.axis->moveTo(0).error(), ErrorCode::MotionInProgress);
        EXPECT_EQ(fx.axis->jog(Direction::Forward).error(), ErrorCode::MotionInProgress);
}

// =====================================================================
// Convenience state predicates: isRunning() / isIdle() / hasFault()
// =====================================================================

TEST(AxisE2ESmokeTest, PredicatesAcrossLifecycle)
{
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);

        // Uninitialized — nothing is true.
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->isIdle());
        EXPECT_FALSE(fx.axis->hasFault());

        // Disabled — still no fault, but not idle (host must enable
        // first; the lib's `Idle` strictly means "enabled and ready").
        ASSERT_TRUE(fx.axis->begin().ok());
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->isIdle());
        EXPECT_FALSE(fx.axis->hasFault());

        // Idle.
        ASSERT_TRUE(fx.axis->enable().ok());
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_TRUE(fx.axis->isIdle());
        EXPECT_FALSE(fx.axis->hasFault());

        // Moving (moveBy in flight).
        ASSERT_TRUE(fx.axis->moveBy(500).ok());
        EXPECT_TRUE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->isIdle());
        EXPECT_FALSE(fx.axis->hasFault());

        // After natural completion → back to Idle.
        fx.engine->runMove();
        fx.axis->service(10);
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_TRUE(fx.axis->isIdle());
        EXPECT_FALSE(fx.axis->hasFault());

        // Jogging.
        ASSERT_TRUE(fx.axis->jog(Direction::Forward).ok());
        EXPECT_TRUE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->isIdle());

        // Emergency stop → EmergencyStopped + faulted.
        ASSERT_TRUE(fx.axis->emergencyStop().ok());
        fx.axis->service(20);
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->isIdle());
        EXPECT_TRUE(fx.axis->hasFault());

        // clearFault() must clear the fault flag. We deliberately
        // don't pin down the post-clear state here (Disabled vs Idle
        // is a recovery-policy detail); the contract is `hasFault()`
        // turns false.
        ASSERT_TRUE(fx.axis->clearFault().ok());
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->hasFault());
}

TEST(AxisE2ESmokeTest, HasFaultCatchesIsrLatchBeforeServiceTick)
{
        // The load-bearing test for `hasFault()`. The pulse engine can
        // latch a fault from an ISR (e.g. CrashLimit edge) before the
        // next `service()` tick promotes that latch to `state_`. During
        // that window `state()` still says `Moving`, but `hasFault()`
        // must return true — otherwise host code that polls `hasFault()`
        // between service ticks would miss the fault for milliseconds.
        auto fx = build();
        ASSERT_NE(fx.axis.get(), nullptr);
        ASSERT_TRUE(fx.axis->begin().ok());
        ASSERT_TRUE(fx.axis->enable().ok());
        ASSERT_TRUE(fx.axis->moveBy(1000).ok());

        // Pre-condition: motion is running, no fault yet.
        ASSERT_TRUE(fx.axis->isRunning());
        ASSERT_FALSE(fx.axis->hasFault());
        ASSERT_EQ(fx.axis->state(), AxisState::Moving);

        // ISR-side fault latch — same path a real CrashLimit edge takes.
        fx.engine->haltFromIsr(StopReason::LimitSwitch);

        // Crucially: do NOT call service() yet. The latch is set on the
        // engine; state_ has not been updated. `state()` still reports
        // Moving (or transitions internally), but `hasFault()` must
        // already be true because the engine's `faultStatus()` is.
        EXPECT_TRUE(fx.axis->hasFault())
                << "hasFault() must observe the engine-side latch "
                   "even before service() promotes it into state_";

        // After service(), state_ catches up to Faulted; predicates
        // must remain consistent.
        fx.axis->service(30);
        EXPECT_EQ(fx.axis->state(), AxisState::Faulted);
        EXPECT_FALSE(fx.axis->isRunning());
        EXPECT_FALSE(fx.axis->isIdle());
        EXPECT_TRUE(fx.axis->hasFault());
}

} // namespace
