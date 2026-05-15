// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "ungula/motor/axis.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/events/axis_event.h"
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor;

class CapturingListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                events.push_back(ev);
        }
        std::vector<AxisEvent> events;
};

StepDirStepperAxisConfig makeCfg()
{
        StepDirStepperAxisConfig cfg;
        cfg.common.axisId = AxisId(7);
        cfg.common.limits.maxVelocitySps = 4000;
        cfg.common.limits.accelSpsPerSec = 8000;
        cfg.common.limits.decelSpsPerSec = 8000;
        cfg.common.limits.maxStepRateSps = 200'000;
        cfg.stepPin = StepPin{ 18 };
        cfg.dirPin = DirectionPin{ 19 };
        cfg.enablePin = EnablePin{ GPIO_NONE };
        return cfg;
}

uint32_t countEvents(const CapturingListener &l, AxisEventType t)
{
        uint32_t n = 0;
        for (const auto &e : l.events)
                if (e.type == t)
                        ++n;
        return n;
}

TEST(AxisEventsTest, StateChangedEmittedOnBeginEnableDisable)
{
        auto r = Axis::createStepDirStepper(makeCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();

        CapturingListener l;
        ASSERT_TRUE(axis->subscribe(&l).ok());

        ASSERT_TRUE(axis->begin().ok());
        axis->serviceEvents(); // drain Begin event
        EXPECT_GE(countEvents(l, AxisEventType::StateChanged), 1u);

        l.events.clear();
        ASSERT_TRUE(axis->enable().ok());
        axis->serviceEvents();
        EXPECT_EQ(countEvents(l, AxisEventType::StateChanged), 1u);

        l.events.clear();
        ASSERT_TRUE(axis->disable().ok());
        axis->serviceEvents();
        EXPECT_EQ(countEvents(l, AxisEventType::StateChanged), 1u);
}

TEST(AxisEventsTest, EmergencyStopEmitsFaultAndStateEvents)
{
        auto r = Axis::createStepDirStepper(makeCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());
        ASSERT_TRUE(axis->enable().ok());

        CapturingListener l;
        ASSERT_TRUE(axis->subscribe(&l).ok());

        ASSERT_TRUE(axis->emergencyStop().ok());
        axis->serviceEvents();

        EXPECT_GE(countEvents(l, AxisEventType::EmergencyStopped), 1u);
        EXPECT_GE(countEvents(l, AxisEventType::FaultRaised), 1u);
        EXPECT_GE(countEvents(l, AxisEventType::StateChanged), 1u);

        // FaultCode should be EmergencyStop on those events
        bool sawFaultCodeOnEstop = false;
        for (const auto &e : l.events) {
                if (e.type == AxisEventType::EmergencyStopped) {
                        EXPECT_EQ(e.faultCode, FaultCode::EmergencyStop);
                        sawFaultCodeOnEstop = true;
                }
        }
        EXPECT_TRUE(sawFaultCodeOnEstop);
}

TEST(AxisEventsTest, EventCarriesAxisId)
{
        auto r = Axis::createStepDirStepper(makeCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();

        CapturingListener l;
        ASSERT_TRUE(axis->subscribe(&l).ok());

        ASSERT_TRUE(axis->begin().ok());
        axis->serviceEvents();
        ASSERT_FALSE(l.events.empty());
        EXPECT_EQ(l.events.front().axisId.value, 7u);
}

TEST(AxisEventsTest, SequenceIsMonotonic)
{
        auto r = Axis::createStepDirStepper(makeCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();

        CapturingListener l;
        ASSERT_TRUE(axis->subscribe(&l).ok());

        ASSERT_TRUE(axis->begin().ok());
        ASSERT_TRUE(axis->enable().ok());
        ASSERT_TRUE(axis->disable().ok());
        axis->serviceEvents();

        ASSERT_GE(l.events.size(), 2u);
        for (size_t i = 1; i < l.events.size(); ++i) {
                EXPECT_GT(l.events[i].sequence, l.events[i - 1].sequence);
        }
}

TEST(AxisEventsTest, ServiceDoesNotCrashWhenIdle)
{
        auto r = Axis::createStepDirStepper(makeCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());

        // No motion, no homing — service() should be a clean no-op.
        axis->service(0);
        axis->service(10);
        axis->service(20);
}

TEST(AxisEventsTest, HomeReturnsUnsupportedWithoutStrategy)
{
        auto r = Axis::createStepDirStepper(makeCfg());
        ASSERT_TRUE(r.ok());
        auto axis = r.takeValue();
        ASSERT_TRUE(axis->begin().ok());
        ASSERT_TRUE(axis->enable().ok());

        EXPECT_EQ(axis->home().error(), ErrorCode::Unsupported);
        EXPECT_FALSE(axis->isHoming());
        EXPECT_EQ(axis->homingPhase(), HomingPhase::Idle);
}

} // namespace
