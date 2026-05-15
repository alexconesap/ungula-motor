// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/fake_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_coolstep.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor::tmc2209;
using namespace ungula::motor;

TEST(Tmc2209CoolStepTest, BeginWritesCoolconfFields)
{
        FakeTmcUart uart;
        Tmc2209CoolStep cs(uart);

        Tmc2209CoolStep::Config cfg;
        cfg.loadThresholdToIncrease = 5;
        cfg.loadThresholdToDecreaseMargin = 2;
        cfg.currentRampUp = Tmc2209CoolStep::StepWidth::Step2;
        cfg.currentRampDown = Tmc2209CoolStep::StepWidth::Step4;
        cfg.currentFloor = Tmc2209CoolStep::CurrentFloor::Quarter;
        cfg.tCoolThrs = 0; // skip TCOOLTHRS

        ASSERT_TRUE(cs.begin(cfg).ok());

        const uint32_t v = uart.registers[reg::COOLCONF];
        EXPECT_EQ((v & coolconf::SEMIN_MASK) >> coolconf::SEMIN_SHIFT, 5u);
        EXPECT_EQ((v & coolconf::SEUP_MASK) >> coolconf::SEUP_SHIFT, 1u);  // Step2
        EXPECT_EQ((v & coolconf::SEMAX_MASK) >> coolconf::SEMAX_SHIFT, 2u);
        EXPECT_EQ((v & coolconf::SEDN_MASK) >> coolconf::SEDN_SHIFT, 2u);  // Step4
        EXPECT_NE(v & coolconf::SEIMIN, 0u);  // Quarter
}

TEST(Tmc2209CoolStepTest, BeginWritesTCoolThrsWhenNonZero)
{
        FakeTmcUart uart;
        Tmc2209CoolStep cs(uart);

        Tmc2209CoolStep::Config cfg;
        cfg.tCoolThrs = 0x1234;
        ASSERT_TRUE(cs.begin(cfg).ok());
        EXPECT_EQ(uart.registers[reg::TCOOLTHRS], 0x1234u);
}

TEST(Tmc2209CoolStepTest, TCoolThrsClampedTo20Bits)
{
        FakeTmcUart uart;
        Tmc2209CoolStep cs(uart);

        Tmc2209CoolStep::Config cfg;
        cfg.tCoolThrs = 0xFFFFFFFF;
        ASSERT_TRUE(cs.begin(cfg).ok());
        EXPECT_EQ(uart.registers[reg::TCOOLTHRS], 0xFFFFFu);
}

TEST(Tmc2209CoolStepTest, BeginSkipsTCoolThrsWhenZero)
{
        FakeTmcUart uart;
        // Pre-seed as if StallGuard had already written the register.
        uart.seed(reg::TCOOLTHRS, 0xABCD);

        Tmc2209CoolStep cs(uart);
        Tmc2209CoolStep::Config cfg;
        cfg.tCoolThrs = 0; // explicitly skip
        ASSERT_TRUE(cs.begin(cfg).ok());

        // The seeded value must survive — CoolStep did not touch it.
        EXPECT_EQ(uart.registers[reg::TCOOLTHRS], 0xABCDu);
}

TEST(Tmc2209CoolStepTest, DisableClearsSeminOnly)
{
        FakeTmcUart uart;
        Tmc2209CoolStep cs(uart);

        Tmc2209CoolStep::Config cfg;
        cfg.loadThresholdToIncrease = 5;
        cfg.loadThresholdToDecreaseMargin = 2;
        ASSERT_TRUE(cs.begin(cfg).ok());

        ASSERT_TRUE(cs.disable().ok());

        const uint32_t v = uart.registers[reg::COOLCONF];
        EXPECT_EQ(v & coolconf::SEMIN_MASK, 0u);  // CoolStep disabled
        // SEMAX should still be there — disable() preserves other fields.
        EXPECT_NE(v & coolconf::SEMAX_MASK, 0u);
}

TEST(Tmc2209CoolStepTest, RejectsOutOfRange)
{
        FakeTmcUart uart;
        Tmc2209CoolStep cs(uart);

        Tmc2209CoolStep::Config cfg;
        cfg.loadThresholdToIncrease = 16;  // > 4 bits
        EXPECT_EQ(cs.begin(cfg).error(), ErrorCode::InvalidConfig);
}

}  // namespace
