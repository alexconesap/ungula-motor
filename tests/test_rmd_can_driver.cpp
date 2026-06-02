// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_can.h"
#include "ungula/canbus/devices/rmd/protocol.h"
#include "ungula/motor/drivers/rmd/rmd_can_driver.h"

using namespace ungula::motor;
using ungula::canbus::tests::FakeCan;
using ungula::motor::rmd::RmdCanDriver;
using ungula::motor::rmd::RmdConfig;

namespace canbus_rmd = ungula::canbus::rmd;

namespace
{

RmdConfig makeConfig(uint8_t motorId = 1, uint32_t spr = 36000)
{
        // 36000 steps/rev makes the SPS↔centideg conversion 1:1 —
        // simplifies hand-verifying wire bytes in tests. Production
        // hosts pick whatever stepsPerRev matches their setup.
        RmdConfig c;
        c.motorId = motorId;
        c.stepsPerRevolution = spr;
        c.commandTimeoutMs = 50;
        return c;
}

int32_t le32(const uint8_t *p)
{
        return static_cast<int32_t>(
            (static_cast<uint32_t>(p[0])) |
            (static_cast<uint32_t>(p[1]) << 8) |
            (static_cast<uint32_t>(p[2]) << 16) |
            (static_cast<uint32_t>(p[3]) << 24));
}

uint16_t le16(const uint8_t *p)
{
        return static_cast<uint16_t>(
            static_cast<uint16_t>(p[0]) |
            (static_cast<uint16_t>(p[1]) << 8));
}

// Pre-queue a stock 0x12 identity reply at RX_BASE + motorId so a
// subsequent `drv.begin()` finds the motor on the bus. Default
// firmware bytes are arbitrary non-zero values.
void queueStockIdentity(FakeCan &bus, uint8_t motorId,
                        uint8_t fwMajor = 0x03, uint8_t fwMinor = 0x05)
{
        ungula::hal::can::CanFrame f{};
        f.id         = canbus_rmd::RX_BASE + motorId;
        f.extendedId = false;
        f.dlc        = 8;
        f.data[0]    = canbus_rmd::CMD_READ_MODEL;
        f.data[1]    = fwMajor;
        f.data[2]    = fwMinor;
        bus.queueRxFrame(f);
}

} // namespace

// =====================================================================
// CAN ID derivation
// =====================================================================

TEST(RmdCanDriver, CanIdIsBaseOf0x140PlusMotorId)
{
        EXPECT_EQ(RmdCanDriver::canIdFor(1), 0x141u);
        EXPECT_EQ(RmdCanDriver::canIdFor(2), 0x142u);
        EXPECT_EQ(RmdCanDriver::canIdFor(32), 0x160u);
}

// =====================================================================
// Lifecycle
// =====================================================================

TEST(RmdCanDriver, BeginRejectsInvalidMotorId)
{
        FakeCan bus;
        RmdCanDriver drv(makeConfig(/*motorId=*/0), bus);
        const auto s = drv.begin();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
}

TEST(RmdCanDriver, BeginRejectsZeroStepsPerRev)
{
        FakeCan bus;
        RmdConfig c = makeConfig();
        c.stepsPerRevolution = 0;
        RmdCanDriver drv(c, bus);
        EXPECT_EQ(drv.begin().error(), ErrorCode::InvalidConfig);
}

// A motor that doesn't reply to 0x12 within the identity timeout is
// not on the bus (wrong motor id, dead drive, unwired CAN). begin()
// must refuse with a TransportError so the host fails fast.
TEST(RmdCanDriver, BeginFailsWhenNoIdentityReply)
{
        FakeCan bus; // no reply pre-queued
        RmdCanDriver drv(makeConfig(), bus);
        const auto s = drv.begin();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::TransportError);
}

TEST(RmdCanDriver, BeginSendsIdentityReadAndCachesFirmware)
{
        FakeCan bus;
        queueStockIdentity(bus, /*motorId=*/3, /*fwMajor=*/0x02, /*fwMinor=*/0x05);
        RmdCanDriver drv(makeConfig(/*motorId=*/3), bus);

        ASSERT_TRUE(drv.begin().ok());
        ASSERT_GE(bus.sent.size(), 1u);
        // First sent frame is the 0x12 identity read.
        EXPECT_EQ(bus.sent[0].id, 0x143u);
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_READ_MODEL);

        const auto id = drv.identity();
        EXPECT_STREQ(id.vendor, "MyActuator");
        EXPECT_STREQ(id.model, "RMD");
        EXPECT_EQ(id.firmwareMajor, 0x02);
        EXPECT_EQ(id.firmwareMinor, 0x05);
}

TEST(RmdCanDriver, EnableSendsMotorRunningCommand)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        ASSERT_TRUE(drv.enable().ok());
        ASSERT_EQ(bus.sent.size(), 1u);
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_WAKE_UP);
}

TEST(RmdCanDriver, DisableDefaultsToMotorStop081)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        ASSERT_TRUE(drv.disable().ok());
        ASSERT_EQ(bus.sent.size(), 1u);
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_STOP);
}

TEST(RmdCanDriver, DisableSendsShutdown080WhenReleaseBrakeOnDisable)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdConfig c = makeConfig();
        c.releaseBrakeOnDisable = true;
        RmdCanDriver drv(c, bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        ASSERT_TRUE(drv.disable().ok());
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_SHUTDOWN);
}

// =====================================================================
// Motion
// =====================================================================

TEST(RmdCanDriver, ArmJogSendsSpeedControlFrameWithSignForward)
{
        FakeCan bus;
        queueStockIdentity(bus, 2);
        RmdCanDriver drv(makeConfig(/*motorId=*/2), bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        // 100 SPS at 36000 SPR → 100 centideg/s → positive (forward).
        ASSERT_TRUE(drv.armJog(Direction::Forward, /*cruiseSps=*/100,
                               /*accelSps2=*/0).ok());
        ASSERT_EQ(bus.sent.size(), 1u);
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.id, 0x142u);
        EXPECT_EQ(f.data[0], canbus_rmd::CMD_SPEED_LOOP);
        EXPECT_EQ(le32(&f.data[4]), 100);
}

TEST(RmdCanDriver, ArmJogSpeedIsNegativeWhenBackward)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        ASSERT_TRUE(drv.armJog(Direction::Backward, 250, 0).ok());
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_SPEED_LOOP);
        EXPECT_EQ(le32(&bus.sent[0].data[4]), -250);
}

TEST(RmdCanDriver, ArmMoveSendsPositionControlFrame)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        // 18000 steps at 36000 SPR = 180 deg = 18000 centideg.
        ASSERT_TRUE(drv.armMove(Direction::Forward, 18000, /*cruiseSps=*/6000,
                                /*accelSps2=*/0, /*decelSps2=*/0).ok());

        ASSERT_EQ(bus.sent.size(), 1u);
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.data[0], canbus_rmd::CMD_POSITION_ABS);
        // Speed field is uint16 LE deg/s (centideg/s / 100).
        //   6000 SPS at 36000 SPR = 6000 centideg/s = 60 deg/s.
        EXPECT_EQ(le16(&f.data[2]), 60u);
        // Position is int32 LE centideg.
        EXPECT_EQ(le32(&f.data[4]), 18000);
        // Commanded position tracker advanced.
        EXPECT_EQ(drv.commandedPositionSteps(), 18000);
}

TEST(RmdCanDriver, ArmMoveBackwardSubtractsFromCommandedPosition)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.armMove(Direction::Forward, 18000, 6000, 0, 0).ok());
        ASSERT_EQ(drv.commandedPositionSteps(), 18000);

        ASSERT_TRUE(drv.armMove(Direction::Backward, 9000, 6000, 0, 0).ok());
        EXPECT_EQ(drv.commandedPositionSteps(), 9000);
}

// The 0xA4 absolute-position field is in CENTIDEG (0.01 deg), but the
// driver tracks `commandedPosition_` in STEPS. When the host's
// stepsPerRevolution doesn't happen to equal 36000 (= 1 step / centideg)
// the conversion factor matters: an earlier version added a centideg
// delta to a step counter, producing the right number ONLY when
// steps == centideg by accident. This regression test uses
// stepsPerRevolution = 3600 (10 centideg per step) so the bug would
// produce different bytes than the fix.
TEST(RmdCanDriver, ArmMovePositionFieldUsesCentidegUnitsAfterPriorMove)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        // SPR = 3600 -> 1 step = 360/3600 = 0.1 deg = 10 centideg.
        RmdCanDriver drv(makeConfig(/*motorId=*/1, /*spr=*/3600), bus);
        ASSERT_TRUE(drv.begin().ok());

        // First move: 100 steps forward. Position field should be 100
        // steps -> 1000 centideg.
        ASSERT_TRUE(drv.armMove(Direction::Forward, 100, 6000, 0, 0).ok());
        bus.sent.clear();

        // Second move: another 100 steps forward. New absolute target
        // should be 200 steps -> 2000 centideg. The pre-fix code would
        // have computed `commandedPosition_(steps=100) + mag(centideg=1000)`
        // = 1100, hitting the wire with a position 900 centideg short
        // of where it belongs.
        ASSERT_TRUE(drv.armMove(Direction::Forward, 100, 6000, 0, 0).ok());
        ASSERT_EQ(bus.sent.size(), 1u);
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.data[0], canbus_rmd::CMD_POSITION_ABS);
        EXPECT_EQ(le32(&f.data[4]), 2000);   // centideg, not 1100
        EXPECT_EQ(drv.commandedPositionSteps(), 200); // steps stay in steps
}

TEST(RmdCanDriver, StopDecelerateSendsMotorStop081)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        ASSERT_TRUE(drv.armJog(Direction::Forward, 1000, 0).ok());
        bus.sent.clear();

        ASSERT_TRUE(drv.stop(StopMode::Decelerate).ok());
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_STOP);
        const auto st = drv.motionStatus();
        EXPECT_FALSE(st.running);
        EXPECT_EQ(st.finishedReason, StopReason::UserStop);
}

TEST(RmdCanDriver, StopImmediateSendsShutdown080)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        bus.sent.clear();

        ASSERT_TRUE(drv.stop(StopMode::Immediate).ok());
        EXPECT_EQ(bus.sent[0].data[0], canbus_rmd::CMD_SHUTDOWN);
        EXPECT_EQ(drv.motionStatus().finishedReason, StopReason::EmergencyStop);
}

// =====================================================================
// Intent
// =====================================================================

TEST(RmdCanDriver, IntentNonDefaultReportsUnsupported)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());
        EXPECT_EQ(drv.applyIntent(MotorIntent::Default), IntentSupport::Supported);
        EXPECT_EQ(drv.applyIntent(MotorIntent::Quiet), IntentSupport::Unsupported);
        EXPECT_EQ(drv.applyIntent(MotorIntent::HighTorque), IntentSupport::Unsupported);
}

// =====================================================================
// SPS / step → centideg helpers
// =====================================================================

TEST(RmdCanDriver, SpsToCentidegMath)
{
        // 1000 SPS at 36000 SPR → 1000 × 36000 / 36000 = 1000 centideg/s.
        EXPECT_EQ(RmdCanDriver::spsToCentideg(1000, 36000), 1000);
        // 1000 SPS at 10000 SPR → 1000 × 36000 / 10000 = 3600 centideg/s.
        EXPECT_EQ(RmdCanDriver::spsToCentideg(1000, 10000), 3600);
        EXPECT_EQ(RmdCanDriver::spsToCentideg(1000, 0), 0);
}

TEST(RmdCanDriver, StepsToCentidegMath)
{
        EXPECT_EQ(RmdCanDriver::stepsToCentideg(36000, 36000), 36000);
        EXPECT_EQ(RmdCanDriver::stepsToCentideg(-9000, 36000), -9000);
}

// =====================================================================
// Diagnostics
// =====================================================================

TEST(RmdCanDriver, DiagnosticsCarryIdentity)
{
        FakeCan bus;
        queueStockIdentity(bus, 1);
        RmdCanDriver drv(makeConfig(), bus);
        ASSERT_TRUE(drv.begin().ok());

        MotorDiagnostics d;
        drv.fillDriverDiagnostics(d);
        EXPECT_STREQ(d.identity.vendor, "MyActuator");
        EXPECT_FALSE(d.stall_valid);
        EXPECT_FALSE(d.adaptive_current_valid);
}
