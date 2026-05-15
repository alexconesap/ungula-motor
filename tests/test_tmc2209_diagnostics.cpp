// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/fake_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor::tmc2209;
using namespace ungula::motor;

TEST(Tmc2209DiagnosticsTest, RefreshReadsThreeRegisters)
{
        FakeTmcUart uart;
        uart.seed(reg::DRV_STATUS, drv_status::STST);
        uart.seed(reg::IOIN, 0x12345678);
        uart.seed(reg::GSTAT, gstat::RESET);

        Tmc2209Diagnostics diag(uart);
        ASSERT_TRUE(diag.refresh().ok());
        EXPECT_EQ(uart.readCallCount, 3u);

        const auto &s = diag.snapshot();
        EXPECT_EQ(s.drvStatus, drv_status::STST);
        EXPECT_EQ(s.ioin, 0x12345678u);
        EXPECT_EQ(s.gstat, gstat::RESET);
        EXPECT_TRUE(s.standstill());
        EXPECT_FALSE(s.driveFault());
}

TEST(Tmc2209DiagnosticsTest, DriveFaultMaskedCorrectly)
{
        FakeTmcUart uart;
        uart.seed(reg::DRV_STATUS, drv_status::OT | drv_status::S2GA);
        Tmc2209Diagnostics diag(uart);
        ASSERT_TRUE(diag.refresh().ok());

        EXPECT_TRUE(diag.snapshot().overTemperature());
        EXPECT_TRUE(diag.snapshot().shortToGround());
        EXPECT_TRUE(diag.snapshot().driveFault());
        EXPECT_FALSE(diag.snapshot().openLoad());
}

TEST(Tmc2209DiagnosticsTest, RefreshSurfacesTransportError)
{
        FakeTmcUart uart;
        Tmc2209Diagnostics diag(uart);

        uart.failNextRead = true;
        EXPECT_EQ(diag.refresh().error(), ErrorCode::TransportError);
}

TEST(Tmc2209DiagnosticsTest, DumpRegistersStopsAtFirstError)
{
        FakeTmcUart uart;
        uart.seed(reg::GCONF, 0x11);
        uart.seed(reg::CHOPCONF, 0x22);
        uart.seed(reg::DRV_STATUS, 0x33);

        Tmc2209Diagnostics diag(uart);

        const uint8_t regs[] = { reg::GCONF, reg::CHOPCONF, reg::DRV_STATUS };
        uint32_t values[3] = {};
        uint8_t good = 0;

        ASSERT_TRUE(diag.dumpRegisters(regs, values, 3, &good).ok());
        EXPECT_EQ(good, 3);
        EXPECT_EQ(values[0], 0x11u);
        EXPECT_EQ(values[1], 0x22u);
        EXPECT_EQ(values[2], 0x33u);
}

} // namespace
