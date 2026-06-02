// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_step_signal.h"
#include "fakes/fake_tmc_transport.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_driver.h"
#include "ungula/motor/planners/trapezoidal_planner.h"

using namespace ungula::motor;
using ungula::motor::tests::FakeStepSignal;
using ungula::motor::tests::FakeTmcTransport;
using ungula::motor::tmc2209::MicrostepDepth;
using ungula::motor::tmc2209::Tmc2209Config;
using ungula::motor::tmc2209::Tmc2209Driver;

namespace
{

// Register addresses mirrored from the driver impl for assertions.
constexpr uint8_t kRegGconf     = 0x00;
constexpr uint8_t kRegGstat     = 0x01;
constexpr uint8_t kRegIoin      = 0x06;
constexpr uint8_t kRegIholdIrun = 0x10;
constexpr uint8_t kRegTcoolthrs = 0x14;
constexpr uint8_t kRegSgthrs    = 0x40;
constexpr uint8_t kRegSgResult  = 0x41;
constexpr uint8_t kRegPwmconf   = 0x70;
constexpr uint8_t kRegCoolconf  = 0x42;
constexpr uint8_t kRegChopconf  = 0x6C;

Tmc2209Config makeConfig()
{
        Tmc2209Config c;
        c.slaveAddress = 0;
        c.runCurrentMa = 800;
        c.holdCurrentMa = 200;
        c.senseResistorOhms = 0.11f;
        c.useHighSensitivity = false;
        c.microsteps = MicrostepDepth::x16;
        c.interpolate = true;
        c.diagPin = 0xFF; // none
        c.stallSensitivity = tmc2209::StallSensitivity::pct(50);
        c.enablePin = 0xFF; // none
        return c;
}

bool writeLogContains(const FakeTmcTransport &tr, uint8_t reg)
{
        for (const auto &e : tr.writeLog()) {
                if (e.reg == reg) {
                        return true;
                }
        }
        return false;
}

uint32_t writeLogLast(const FakeTmcTransport &tr, uint8_t reg)
{
        uint32_t v = 0;
        for (const auto &e : tr.writeLog()) {
                if (e.reg == reg) {
                        v = e.value;
                }
        }
        return v;
}

} // namespace

// =====================================================================
// Lifecycle: begin writes the canonical sequence
// =====================================================================

TEST(Tmc2209Driver, BeginWritesGconfChopconfIholdIrunAndClearsGstat)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);

        // IOIN read returns version byte 0x21 (production silicon).
        tr.setRegister(kRegIoin, 0x21000000);

        ASSERT_TRUE(drv.begin().ok());
        EXPECT_TRUE(writeLogContains(tr, kRegGconf));
        EXPECT_TRUE(writeLogContains(tr, kRegChopconf));
        EXPECT_TRUE(writeLogContains(tr, kRegIholdIrun));
        EXPECT_TRUE(writeLogContains(tr, kRegGstat));
        // PWMCONF must be explicitly written at begin() so StealthChop
        // doesn't inherit stale chopper coefficients from whatever
        // program ran before; without this the motor can run hot in
        // StealthChop because pwm_autoscale never restarts from a
        // known baseline.
        EXPECT_TRUE(writeLogContains(tr, kRegPwmconf));
        const uint32_t pwmconf = writeLogLast(tr, kRegPwmconf);
        EXPECT_NE(pwmconf & (1u << 18), 0u); // pwm_autoscale set
        EXPECT_NE(pwmconf & (1u << 19), 0u); // pwm_autograd set

        // CHOPCONF defaults (matching v2 / TMCStepper): TOFF=5, TBL=1
        // (= 24 clk blank time). These are exposed on the config for
        // host override; the default test pins them so a config change
        // can't silently shift behaviour.
        const uint32_t chopconf = writeLogLast(tr, kRegChopconf);
        EXPECT_EQ(chopconf & 0x0Fu, 5u);          // TOFF
        EXPECT_EQ((chopconf >> 15) & 0x03u, 1u);  // TBL = 1 (24 clk)

        // Identity should now reflect the IOIN read.
        const auto id = drv.identity();
        EXPECT_STREQ(id.vendor, "Trinamic");
        EXPECT_STREQ(id.model, "TMC2209");
        EXPECT_EQ(id.firmwareMajor, 0x21u);
}

TEST(Tmc2209Driver, BeginRejectsInvalidConfig)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto bad = makeConfig();
        bad.runCurrentMa = 0;
        Tmc2209Driver drv(bad, tr, step, planner);
        const auto s = drv.begin();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
}

// IOIN version byte of 0x00 means nothing on the bus replied (or
// replied all zeros) — the chip is not present at this slave
// address. begin() must refuse so the host fails fast.
TEST(Tmc2209Driver, BeginRejectsBlankIoinAsTransportError)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        tr.setRegister(kRegIoin, 0x00000000u); // override fake's default 0x21
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        const auto s = drv.begin();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::TransportError);
}

// IOIN version byte of 0xFF is the "floating bus" pattern — same
// failure semantics.
TEST(Tmc2209Driver, BeginRejectsAllOnesIoinAsTransportError)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        tr.setRegister(kRegIoin, 0xFFFFFFFFu);
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        const auto s = drv.begin();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::TransportError);
}

// =====================================================================
// GCONF: chopper mode follows intent
// =====================================================================

TEST(Tmc2209Driver, DefaultIntentSelectsStealthChop)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const uint32_t gconf = writeLogLast(tr, kRegGconf);
        EXPECT_EQ(gconf & (1u << 2), 0u); // EN_SPREADCYCLE not set
}

TEST(Tmc2209Driver, HighTorqueIntentSelectsSpreadCycle)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);

        // Apply intent BEFORE begin so the first GCONF write reflects it.
        drv.applyIntent(MotorIntent::HighTorque);
        ASSERT_TRUE(drv.begin().ok());

        const uint32_t gconf = writeLogLast(tr, kRegGconf);
        EXPECT_NE(gconf & (1u << 2), 0u); // EN_SPREADCYCLE set
}

TEST(Tmc2209Driver, ApplyIntentAfterBeginRewritesChopperMode)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const auto before = writeLogLast(tr, kRegGconf);
        EXPECT_EQ(before & (1u << 2), 0u); // default = StealthChop

        const auto support = drv.applyIntent(MotorIntent::HighTorque);
        EXPECT_EQ(support, IntentSupport::Supported);

        const auto after = writeLogLast(tr, kRegGconf);
        EXPECT_NE(after & (1u << 2), 0u); // SpreadCycle
}

TEST(Tmc2209Driver, QuietPlusHighTorqueIsPartiallySupported)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const auto support =
            drv.applyIntent(MotorIntent::Quiet | MotorIntent::HighTorque);
        EXPECT_EQ(support, IntentSupport::PartiallySupported);

        // HighTorque wins → SpreadCycle.
        EXPECT_NE(writeLogLast(tr, kRegGconf) & (1u << 2), 0u);
}

// =====================================================================
// Stall configuration (only when DIAG pin is set)
// =====================================================================

TEST(Tmc2209Driver, NoStallWritesWhenDiagPinUnset)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 0xFF;
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        EXPECT_FALSE(writeLogContains(tr, kRegSgthrs));
        EXPECT_FALSE(writeLogContains(tr, kRegTcoolthrs));
}

TEST(Tmc2209Driver, StallSensitivityMapsToSgthrs)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;                // any non-NONE pin
        cfg.stallSensitivity = tmc2209::StallSensitivity::pct(50);
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const uint32_t sgthrs = writeLogLast(tr, kRegSgthrs);
        // 50% of 255 = 127.5 -> 127 (integer division).
        EXPECT_EQ(sgthrs, 127u);
        EXPECT_TRUE(writeLogContains(tr, kRegTcoolthrs));
}

// Host can override the chopper baseline (TOFF, blank time) and the
// PWMCONF flags via the same Tmc2209Config struct. Verify the values
// land in the right register bits.
TEST(Tmc2209Driver, ChopperAndPwmconfHostOverrides)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.toff = 7;
        cfg.blankTimeClk = 36;       // TBL = 2
        cfg.pwmAutoscale = false;
        cfg.pwmAutograd = false;
        cfg.pwmFreq = 2;
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const uint32_t chopconf = writeLogLast(tr, kRegChopconf);
        EXPECT_EQ(chopconf & 0x0Fu, 7u);          // TOFF
        EXPECT_EQ((chopconf >> 15) & 0x03u, 2u);  // TBL = 2 (= 36 clk)

        const uint32_t pwmconf = writeLogLast(tr, kRegPwmconf);
        EXPECT_EQ(pwmconf & (1u << 18), 0u);      // pwm_autoscale off
        EXPECT_EQ(pwmconf & (1u << 19), 0u);      // pwm_autograd off
        EXPECT_EQ((pwmconf >> 16) & 0x03u, 2u);   // pwm_freq
}

// Invalid host values must be rejected at begin().
TEST(Tmc2209Driver, BeginRejectsInvalidChopperConfig)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        {
                auto cfg = makeConfig();
                cfg.toff = 16;            // out of 4-bit range
                Tmc2209Driver drv(cfg, tr, step, planner);
                EXPECT_EQ(drv.begin().error(), ErrorCode::InvalidConfig);
        }
        {
                auto cfg = makeConfig();
                cfg.blankTimeClk = 30;    // not 16/24/36/54
                Tmc2209Driver drv(cfg, tr, step, planner);
                EXPECT_EQ(drv.begin().error(), ErrorCode::InvalidConfig);
        }
        {
                auto cfg = makeConfig();
                cfg.pwmFreq = 4;          // 2-bit field, max is 3
                Tmc2209Driver drv(cfg, tr, step, planner);
                EXPECT_EQ(drv.begin().error(), ErrorCode::InvalidConfig);
        }
}

// RawSgthrs path: the host wants to write a specific byte to SGTHRS
// (e.g. picked from a chip dump) without going through the percent
// mapping. Whatever value the host gives must reach the chip
// verbatim.
TEST(Tmc2209Driver, RawSgthrsBypassesPercentMath)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;
        cfg.stallSensitivity = tmc2209::StallSensitivity::rawSgthrs(90);
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        const uint32_t sgthrs = writeLogLast(tr, kRegSgthrs);
        EXPECT_EQ(sgthrs, 90u); // verbatim, no pct conversion
}

// TMC2209 StallGuard4 only runs in StealthChop. If the host wires
// DIAG AND asks for HighTorque (SpreadCycle), the chip silently
// stops updating SG_RESULT and the stall path is dead. The lib
// should refuse the config at begin() rather than booting a setup
// that will never trip a stall. Matches the v2 stallguard's
// `verifyChopperMode = true` boot-time check.
TEST(Tmc2209Driver, BeginRefusesDiagWithHighTorqueIntent)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;
        Tmc2209Config copy = cfg;
        Tmc2209Driver drv(copy, tr, step, planner);
        // HighTorque -> SpreadCycle.
        EXPECT_EQ(drv.applyIntent(MotorIntent::HighTorque), IntentSupport::Supported);
        const auto s = drv.begin();
        EXPECT_FALSE(s.ok());
        EXPECT_EQ(s.error(), ErrorCode::InvalidConfig);
}

// Same constraint at runtime: if the host calls applyIntent with
// HighTorque after begin() while DIAG is wired, the lib refuses
// the chopper flip and reports Conflicted. The chip stays in
// StealthChop; stall detection keeps working.
TEST(Tmc2209Driver, ApplyIntentRefusesHighTorqueWhenDiagWired)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());
        tr.resetLogs();

        const auto result = drv.applyIntent(MotorIntent::HighTorque);
        EXPECT_EQ(result, IntentSupport::Conflicted);
        // No GCONF write should have happened.
        EXPECT_FALSE(writeLogContains(tr, kRegGconf));
}

// =====================================================================
// CoolStep: enabled by AdaptiveCurrent / Cool / EnergySaving intents
// =====================================================================

TEST(Tmc2209Driver, AdaptiveCurrentIntentEnablesCoolStep)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;
        Tmc2209Driver drv(cfg, tr, step, planner);
        drv.applyIntent(MotorIntent::AdaptiveCurrent);
        ASSERT_TRUE(drv.begin().ok());

        EXPECT_TRUE(writeLogContains(tr, kRegCoolconf));
        // SEMIN non-zero means CoolStep is enabled.
        EXPECT_NE(writeLogLast(tr, kRegCoolconf) & 0x0Fu, 0u);
}

TEST(Tmc2209Driver, AdaptiveCurrentIntentEnablesCoolStepWithoutDiagPin)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 0xFF;
        Tmc2209Driver drv(cfg, tr, step, planner);
        drv.applyIntent(MotorIntent::AdaptiveCurrent);
        ASSERT_TRUE(drv.begin().ok());

        EXPECT_TRUE(writeLogContains(tr, kRegCoolconf));
        EXPECT_NE(writeLogLast(tr, kRegCoolconf) & 0x0Fu, 0u);
        EXPECT_TRUE(writeLogContains(tr, kRegTcoolthrs));
        EXPECT_EQ(writeLogLast(tr, kRegTcoolthrs), 0xFFFFFu);
}

TEST(Tmc2209Driver, DefaultIntentLeavesCoolStepDisabled)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        // COOLCONF is still written but with SEMIN=0 (disabled).
        EXPECT_TRUE(writeLogContains(tr, kRegCoolconf));
        EXPECT_EQ(writeLogLast(tr, kRegCoolconf) & 0x0Fu, 0u);
}

// =====================================================================
// mA <-> CS round-trip
// =====================================================================

TEST(Tmc2209Driver, MilliampsToCsAtKnownPoints)
{
        // 0.11 Ω sense, low-sensitivity (Vfs = 0.325 V).
        // 800 mA RMS → CS ≈ round(0.8 * 32 * √2 * 0.13 / 0.325) - 1 ≈ 13.
        EXPECT_EQ(Tmc2209Driver::milliampsToCs(800, 0.11f, false), 13u);
        // Round-trip within ±5% of the original.
        const uint8_t cs = Tmc2209Driver::milliampsToCs(800, 0.11f, false);
        const uint16_t ma = Tmc2209Driver::csToMilliamps(cs, 0.11f, false);
        EXPECT_GE(ma, 760u);
        EXPECT_LE(ma, 840u);
}

TEST(Tmc2209Driver, MilliampsToCsClampsAtMax)
{
        // Insane current → clamps at CS=31.
        EXPECT_EQ(Tmc2209Driver::milliampsToCs(10000, 0.11f, false), 31u);
}

// =====================================================================
// Motion: arm forwards through planner + step signal generator
// =====================================================================

TEST(Tmc2209Driver, ArmMoveProducesPlannedMoveAndArmsStepSignal)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        ASSERT_TRUE(drv.armMove(Direction::Forward, 10000, 80000, 320000, 320000).ok());
        EXPECT_EQ(step.armCalls_, 1u);
        EXPECT_EQ(step.lastMove_.totalSteps, 10000u);
        EXPECT_EQ(step.lastMove_.direction, Direction::Forward);
}

TEST(Tmc2209Driver, ArmJogProducesIndefiniteMove)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        ASSERT_TRUE(drv.armJog(Direction::Backward, 10000, 40000).ok());
        EXPECT_EQ(step.armCalls_, 1u);
        EXPECT_EQ(step.lastMove_.direction, Direction::Backward);
        // 2.1 G steps cap. We just verify it's much larger than a normal
        // move would be.
        EXPECT_GT(step.lastMove_.totalSteps, 1'000'000u);
}

TEST(Tmc2209Driver, MotionStatusForwardsFromStepSignal)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        step.setStatus({ true, false, StopReason::None });
        auto st = drv.motionStatus();
        EXPECT_TRUE(st.running);

        step.setStatus({ false, false, StopReason::TargetReached });
        st = drv.motionStatus();
        EXPECT_FALSE(st.running);
        EXPECT_EQ(st.finishedReason, StopReason::TargetReached);
}

// =====================================================================
// Diagnostics: SG_RESULT -> load %
// =====================================================================

TEST(Tmc2209Driver, DiagnosticsExposeStallReadingWhenDiagPinSet)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        auto cfg = makeConfig();
        cfg.diagPin = 26;
        cfg.stallSensitivity = tmc2209::StallSensitivity::pct(50);
        Tmc2209Driver drv(cfg, tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        // SG_RESULT = 1023 (free-running) → load reading should be 0 %.
        tr.setRegister(kRegSgResult, 1023);
        MotorDiagnostics d;
        drv.fillDriverDiagnostics(d);
        EXPECT_TRUE(d.stall_valid);
        // 50% pct -> SGTHRS = 127. Diagnostic round-trips
        // (127 * 100 / 255) = 49.
        EXPECT_GE(d.stallSensitivityPct, 49u);
        EXPECT_LE(d.stallSensitivityPct, 50u);
        EXPECT_LE(d.stallReadingPct, 1u);

        // SG_RESULT = 0 (heavily loaded) → 100 %.
        tr.setRegister(kRegSgResult, 0);
        drv.fillDriverDiagnostics(d);
        EXPECT_EQ(d.stallReadingPct, 100u);
}

TEST(Tmc2209Driver, DiagnosticsLeaveStallInvalidWhenNoDiagPin)
{
        FakeTmcTransport tr;
        FakeStepSignal step;
        TrapezoidalPlanner planner;
        Tmc2209Driver drv(makeConfig(), tr, step, planner);
        ASSERT_TRUE(drv.begin().ok());

        MotorDiagnostics d;
        drv.fillDriverDiagnostics(d);
        EXPECT_FALSE(d.stall_valid);
        EXPECT_STREQ(d.identity.model, "TMC2209");
}
