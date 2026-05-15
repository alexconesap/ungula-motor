// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/fake_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_configurator.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"
#include "ungula/motor/result.h"

namespace
{

using namespace ungula::motor::tmc2209;
using namespace ungula::motor;

TEST(Tmc2209ConfiguratorTest, BeginWritesExpectedRegistersInOrder)
{
        FakeTmcUart uart;
        Tmc2209Configurator cfg(uart);

        Tmc2209Configurator::Config c;
        c.runCurrentMa = 800;
        c.holdCurrentMa = 400;
        c.senseResistorOhms = 0.11f;
        c.microsteps = Microsteps::Sixteenth;
        c.mode = ChopperMode::StealthChop;

        EXPECT_TRUE(cfg.begin(c).ok());

        // Four writes expected: GCONF, CHOPCONF, IHOLD_IRUN, GSTAT.
        EXPECT_EQ(uart.writeCallCount, 4u);

        // No reads in begin() — diagnostics are opt-in.
        EXPECT_EQ(uart.readCallCount, 0u);

        // PDN_DISABLE + MSTEP_REG_SELECT, no SpreadCycle for StealthChop.
        EXPECT_EQ(uart.registers[reg::GCONF] & gconf::EN_SPREADCYCLE, 0u);
        EXPECT_NE(uart.registers[reg::GCONF] & gconf::PDN_DISABLE, 0u);
        EXPECT_NE(uart.registers[reg::GCONF] & gconf::MSTEP_REG_SELECT, 0u);

        // MRES nibble = Sixteenth (4)
        const uint32_t mres = (uart.registers[reg::CHOPCONF] & chopconf::MRES_MASK) >>
                              chopconf::MRES_SHIFT;
        EXPECT_EQ(mres, 4u);
}

TEST(Tmc2209ConfiguratorTest, ClearGstatWritesNotReads)
{
        // Regression: the old driver did a READ of GSTAT in clearGstat,
        // which doesn't clear the flags. Verify we WRITE the W1C mask.
        FakeTmcUart uart;
        uart.registers[reg::GSTAT] = gstat::ALL_FLAGS; // pretend boot left flags set

        Tmc2209Configurator cfg(uart);
        ASSERT_TRUE(cfg.clearGstat().ok());

        EXPECT_EQ(uart.lastWrittenReg, reg::GSTAT);
        EXPECT_EQ(uart.lastWrittenValue, gstat::ALL_FLAGS);
        // GSTAT bits should be cleared (FakeTmcUart honours W1C).
        EXPECT_EQ(uart.registers[reg::GSTAT], 0u);
}

TEST(Tmc2209ConfiguratorTest, RejectsInvalidCurrents)
{
        FakeTmcUart uart;
        Tmc2209Configurator cfg(uart);

        Tmc2209Configurator::Config c;
        c.runCurrentMa = 0; // zero run current makes no sense
        EXPECT_EQ(cfg.begin(c).error(), ErrorCode::InvalidConfig);
        EXPECT_EQ(uart.writeCallCount, 0u); // nothing written on bad input

        Tmc2209Configurator::Config c2;
        c2.runCurrentMa = 500;
        c2.holdCurrentMa = 900; // hold > run is invalid
        EXPECT_EQ(cfg.begin(c2).error(), ErrorCode::InvalidConfig);

        Tmc2209Configurator::Config c3;
        c3.senseResistorOhms = 0.0f; // zero sense resistor
        EXPECT_EQ(cfg.begin(c3).error(), ErrorCode::InvalidConfig);
}

TEST(Tmc2209ConfiguratorTest, MilliampsToCsRoundTrip)
{
        // Sanity check the conversion against the datasheet formula.
        // 0.11 Ω sense resistor, low-sensitivity (Vfs = 0.325 V).
        // Datasheet: I_rms = (CS+1)/32 * 0.325 / 0.13 / √2
        // For CS=15 → I_rms ≈ 0.706 A → 706 mA.
        const uint16_t expectedMa =
            Tmc2209Configurator::csToMilliamps(15, 0.11f, false);
        EXPECT_GE(expectedMa, 690u);
        EXPECT_LE(expectedMa, 720u);

        // Round-trip: convert that mA value back to CS, should land at 15.
        const uint8_t cs = Tmc2209Configurator::milliampsToCs(expectedMa, 0.11f, false);
        EXPECT_EQ(cs, 15);
}

TEST(Tmc2209ConfiguratorTest, MilliampsToCsClampsAtMaxCurrent)
{
        // Ask for far more current than the chip can deliver at this
        // sense resistor — should clamp to CS=31, not wrap.
        const uint8_t cs = Tmc2209Configurator::milliampsToCs(5000, 0.11f, false);
        EXPECT_EQ(cs, 31);
}

TEST(Tmc2209ConfiguratorTest, HighSensitivityScalesDifferently)
{
        // For a given mA target, the high-sensitivity (Vfs=0.18V) range
        // produces a higher CS value than the default (Vfs=0.325V).
        const uint8_t csLow = Tmc2209Configurator::milliampsToCs(200, 0.11f, false);
        const uint8_t csHigh = Tmc2209Configurator::milliampsToCs(200, 0.11f, true);
        EXPECT_GT(csHigh, csLow);
}

TEST(Tmc2209ConfiguratorTest, SetMicrostepsPreservesChopconfFields)
{
        FakeTmcUart uart;
        Tmc2209Configurator cfg(uart);
        Tmc2209Configurator::Config c;
        c.microsteps = Microsteps::Sixteenth;
        c.toff = 5;
        c.interpolate = true;
        ASSERT_TRUE(cfg.begin(c).ok());

        const uint32_t before = uart.registers[reg::CHOPCONF];
        ASSERT_TRUE(cfg.setMicrosteps(Microsteps::TwoFiftySix).ok());
        const uint32_t after = uart.registers[reg::CHOPCONF];

        // Only the MRES nibble should have changed.
        EXPECT_EQ(before & ~chopconf::MRES_MASK, after & ~chopconf::MRES_MASK);
        const uint32_t mres = (after & chopconf::MRES_MASK) >> chopconf::MRES_SHIFT;
        EXPECT_EQ(mres, 0u); // 256 microsteps encodes as 0
}

TEST(Tmc2209ConfiguratorTest, SetChopperModeFlipsBitOnly)
{
        FakeTmcUart uart;
        Tmc2209Configurator cfg(uart);
        Tmc2209Configurator::Config c;
        c.mode = ChopperMode::StealthChop;
        ASSERT_TRUE(cfg.begin(c).ok());

        const uint32_t baseline = uart.registers[reg::GCONF];

        ASSERT_TRUE(cfg.setChopperMode(ChopperMode::SpreadCycle).ok());
        EXPECT_NE(uart.registers[reg::GCONF] & gconf::EN_SPREADCYCLE, 0u);
        EXPECT_EQ(uart.registers[reg::GCONF] & ~gconf::EN_SPREADCYCLE,
                  baseline & ~gconf::EN_SPREADCYCLE);

        ASSERT_TRUE(cfg.setChopperMode(ChopperMode::StealthChop).ok());
        EXPECT_EQ(uart.registers[reg::GCONF] & gconf::EN_SPREADCYCLE, 0u);
}

TEST(Tmc2209ConfiguratorTest, TransportErrorBubblesUp)
{
        FakeTmcUart uart;
        Tmc2209Configurator cfg(uart);

        uart.failNextWrite = true;
        EXPECT_EQ(cfg.begin({}).error(), ErrorCode::TransportError);
}

TEST(Tmc2209ConfiguratorTest, DoubleBeginRejected)
{
        FakeTmcUart uart;
        Tmc2209Configurator cfg(uart);
        ASSERT_TRUE(cfg.begin({}).ok());
        EXPECT_EQ(cfg.begin({}).error(), ErrorCode::AlreadyInitialized);
}

} // namespace
