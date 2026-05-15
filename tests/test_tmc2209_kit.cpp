// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <memory>

#include "ungula/hal/uart/uart.h"

#include "ungula/motor/drivers/tmc2209/tmc2209_kit.h"

namespace
{

using namespace ungula::motor;
using namespace ungula::motor::tmc2209;

StepperKitConfig makeBaseCfg()
{
        StepperKitConfig c;
        c.common.axisId = AxisId(1);
        c.common.name = "kit";
        c.common.units.stepsPerMm = 100.0f;
        c.common.limits.maxVelocitySps = 4000;
        c.common.limits.accelSpsPerSec = 8000;
        c.common.limits.decelSpsPerSec = 8000;
        c.common.limits.maxStepRateSps = 100000;

        c.stepPin = StepPin{ 18 };
        c.dirPin = DirectionPin{ 19 };
        c.enablePin = EnablePin{ 20 };

        c.uartPort = 1;
        c.uartBaud = 115200;
        c.uartTxPin = 17;
        c.uartRxPin = 16;
        c.slaveAddress = 0;
        return c;
}

TEST(Tmc2209KitTest, FactoryRejectsMissingStepPin)
{
        auto cfg = makeBaseCfg();
        cfg.stepPin = StepPin{}; // GPIO_NONE
        auto r = makeStepperKit(cfg);
        ASSERT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(Tmc2209KitTest, FactoryRejectsMissingDirPin)
{
        auto cfg = makeBaseCfg();
        cfg.dirPin = DirectionPin{};
        auto r = makeStepperKit(cfg);
        ASSERT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(Tmc2209KitTest, FactoryRejectsBadSlaveAddress)
{
        auto cfg = makeBaseCfg();
        cfg.slaveAddress = 7; // > 3
        auto r = makeStepperKit(cfg);
        ASSERT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::InvalidConfig);
}

TEST(Tmc2209KitTest, ValidConfigProducesAxis)
{
        auto cfg = makeBaseCfg();
        auto r = makeStepperKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        ASSERT_NE(kit, nullptr);
        EXPECT_NE(kit->axis, nullptr);
        EXPECT_NE(kit->configurator, nullptr);
        EXPECT_NE(kit->transport, nullptr);
        EXPECT_TRUE(kit->ownsUart);
        EXPECT_NE(kit->uart, nullptr);
        // Optional helpers default off.
        EXPECT_EQ(kit->stallGuard, nullptr);
        EXPECT_EQ(kit->coolStep, nullptr);
}

TEST(Tmc2209KitTest, OptionalStallGuardConstructed)
{
        auto cfg = makeBaseCfg();
        cfg.useStallGuard = true;
        cfg.stall.sgThreshold = 30;
        auto r = makeStepperKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        EXPECT_NE(kit->stallGuard, nullptr);
}

TEST(Tmc2209KitTest, OptionalCoolStepConstructed)
{
        auto cfg = makeBaseCfg();
        cfg.useCoolStep = true;
        auto r = makeStepperKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        EXPECT_NE(kit->coolStep, nullptr);
}

TEST(Tmc2209KitTest, TandemFieldsPropagateToStoredCfg)
{
        auto cfg = makeBaseCfg();
        cfg.secondaryDirPin = DirectionPin{ 22 };
        cfg.secondaryDirInverted = true;
        cfg.secondaryEnablePin = EnablePin{ 23 };
        auto r = makeStepperKit(cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        EXPECT_EQ(kit->storedCfg.secondaryDirPin.value, 22);
        EXPECT_TRUE(kit->storedCfg.secondaryDirInverted);
        EXPECT_EQ(kit->storedCfg.secondaryEnablePin.value, 23);
}

TEST(Tmc2209KitTest, SharedUartFactoryDoesNotOwnUart)
{
        ungula::hal::uart::Uart bus(2);
        ASSERT_TRUE(bus.begin(115200, 17, 16));

        auto cfgA = makeBaseCfg();
        auto cfgB = makeBaseCfg();
        cfgB.common.axisId = AxisId(2);
        cfgB.stepPin = StepPin{ 25 };
        cfgB.dirPin = DirectionPin{ 26 };

        auto rA = makeStepperKitOnUart(bus, /*addr=*/0, cfgA);
        auto rB = makeStepperKitOnUart(bus, /*addr=*/1, cfgB);
        ASSERT_TRUE(rA.ok());
        ASSERT_TRUE(rB.ok());

        auto kitA = rA.takeValue();
        auto kitB = rB.takeValue();
        EXPECT_FALSE(kitA->ownsUart);
        EXPECT_FALSE(kitB->ownsUart);
        EXPECT_EQ(kitA->uart, nullptr);
        EXPECT_EQ(kitB->uart, nullptr);
        // Two distinct axes — N-motor projects must produce N
        // independent kits even on one bus.
        EXPECT_NE(kitA->axis.get(), kitB->axis.get());
}

TEST(Tmc2209KitTest, SharedUartHonoursExplicitSlaveAddress)
{
        ungula::hal::uart::Uart bus(2);
        ASSERT_TRUE(bus.begin(115200, 17, 16));

        auto cfg = makeBaseCfg();
        cfg.slaveAddress = 0; // overridden by explicit param
        auto r = makeStepperKitOnUart(bus, /*addr=*/3, cfg);
        ASSERT_TRUE(r.ok());
        auto kit = r.takeValue();
        EXPECT_EQ(kit->storedCfg.slaveAddress, 3);
}

} // namespace
