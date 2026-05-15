// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/fake_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_stallguard.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor::tmc2209;
using namespace ungula::motor;

TEST(Tmc2209StallGuardTest, BeginWritesSgthrsAndTCoolThrs)
{
        FakeTmcUart uart;
        Tmc2209StallGuard sg(uart);

        Tmc2209StallGuard::Config cfg;
        cfg.sgThreshold = 42;
        cfg.tCoolThrs = 0x1000;
        ASSERT_TRUE(sg.begin(cfg).ok());

        EXPECT_EQ(uart.registers[reg::SGTHRS], 42u);
        EXPECT_EQ(uart.registers[reg::TCOOLTHRS], 0x1000u);
        EXPECT_EQ(sg.shadowSgThreshold(), 42u);
        EXPECT_EQ(sg.shadowTCoolThrs(), 0x1000u);
}

TEST(Tmc2209StallGuardTest, TCoolThrsClampedTo20Bits)
{
        FakeTmcUart uart;
        Tmc2209StallGuard sg(uart);
        ASSERT_TRUE(sg.setTCoolThrs(0xFFFFFFFF).ok());
        EXPECT_EQ(uart.registers[reg::TCOOLTHRS], 0xFFFFFu);
        EXPECT_EQ(sg.shadowTCoolThrs(), 0xFFFFFu);
}

TEST(Tmc2209StallGuardTest, ReadSgResultMasks10Bits)
{
        FakeTmcUart uart;
        // Pre-seed with bits above [9:0] set; readSgResult must mask them.
        uart.seed(reg::SG_RESULT, 0xABCD03FFu);

        Tmc2209StallGuard sg(uart);
        auto r = sg.readSgResult();
        ASSERT_TRUE(r.ok());
        EXPECT_EQ(r.takeValue(), 0x3FFu);
}

TEST(Tmc2209StallGuardTest, ReadSgResultPropagatesTransportError)
{
        FakeTmcUart uart;
        Tmc2209StallGuard sg(uart);

        uart.failNextRead = true;
        auto r = sg.readSgResult();
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.error(), ErrorCode::TransportError);
}

} // namespace
