// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_driver.h"

#include <cmath>
#include <cstddef>
#include <memory>

#include "ungula/hal/gpio/gpio.h"
#include "ungula/motor/planners/trapezoidal_planner.h"
#include "ungula/motor/step_signal/rmt_step_signal.h"

namespace ungula::motor::tmc2209
{

namespace gpio = ungula::hal::gpio;

// =====================================================================
// Wire constants — TMC datagram framing (private to this file)
// =====================================================================

namespace
{

        constexpr uint8_t kSync = 0x05;
        constexpr uint8_t kWriteFlag = 0x80;
        constexpr uint8_t kMaxSlave = 3;
        constexpr size_t kWriteFrame = 8; // SYNC + slave + reg + 4 data + CRC
        constexpr size_t kReadReq = 4; // SYNC + slave + reg + CRC
        constexpr size_t kReadReply = 8; // SYNC + 0xFF + reg + 4 data + CRC

        // Register addresses we touch. Other registers are not used by
        // this driver — adding them is straightforward, no need to
        // expose `reg::` to host code.
        constexpr uint8_t kRegGconf = 0x00;
        constexpr uint8_t kRegGstat = 0x01;
        constexpr uint8_t kRegIfcnt = 0x02;
        constexpr uint8_t kRegIoin = 0x06;
        constexpr uint8_t kRegIholdIrun = 0x10;
        constexpr uint8_t kRegTstep = 0x12;
        constexpr uint8_t kRegTcoolthrs = 0x14;
        constexpr uint8_t kRegSgthrs = 0x40;
        constexpr uint8_t kRegSgResult = 0x41;
        constexpr uint8_t kRegCoolconf = 0x42;
        constexpr uint8_t kRegChopconf = 0x6C;
        constexpr uint8_t kRegDrvStatus = 0x6F;
        constexpr uint8_t kRegPwmconf = 0x70;

        // GCONF bits.
        constexpr uint32_t kGconfEnSpreadCycle = 1u << 2;
        constexpr uint32_t kGconfPdnDisable = 1u << 6;
        constexpr uint32_t kGconfMstepRegSelect = 1u << 7;

        // GSTAT W1C bits.
        constexpr uint32_t kGstatAll = (1u << 0) | (1u << 1) | (1u << 2);

        // CHOPCONF bits / shifts.
        constexpr uint32_t kChopVsense = 1u << 17;
        constexpr uint8_t kChopMresShift = 24;
        constexpr uint32_t kChopIntpol = 1u << 28;

        // COOLCONF: minimal config — SEMIN=4, SEMAX margin=2, defaults
        // for SEUP/SEDN/SEIMIN (Half-current floor). The host shouldn't
        // need to micromanage these; they're a sensible starting point
        // that maps to MotorIntent::AdaptiveCurrent / Cool.
        constexpr uint8_t kCoolConfSeminDefault = 4;
        constexpr uint8_t kCoolConfSemaxMarginDef = 2;
        constexpr uint32_t kTcoolthrsAlwaysOn = 0xFFFFFu;

        // Current-conversion constants (TMC datasheet §9.1 / §5.5.1).
        constexpr float kVfsLow = 0.325f;
        constexpr float kVfsHigh = 0.180f;
        constexpr float kInternalSenseOhms = 0.02f;
        constexpr float kSqrt2 = 1.41421356f;
        constexpr uint8_t kMaxCs = 31;

        constexpr float vfs(bool useHighSensitivity)
        {
                return useHighSensitivity ? kVfsHigh : kVfsLow;
        }

        // CRC8 with polynomial 0x07, init 0, lsb-first. Trinamic
        // AN001 reference algorithm.
        uint8_t tmcCrc(const uint8_t *data, size_t length)
        {
                uint8_t c = 0;
                for (size_t i = 0; i < length; ++i) {
                        uint8_t b = data[i];
                        for (uint8_t j = 0; j < 8; ++j) {
                                const uint8_t mix = (c >> 7) ^ (b & 0x01);
                                c = static_cast<uint8_t>(c << 1);
                                if (mix) {
                                        c ^= 0x07;
                                }
                                b = static_cast<uint8_t>(b >> 1);
                        }
                }
                return c;
        }

        // Build IHOLD_IRUN word: bits [4:0]=IHOLD, [12:8]=IRUN,
        // [19:16]=IHOLDDELAY.
        constexpr uint32_t packIholdIrun(uint8_t ihold, uint8_t irun, uint8_t delay)
        {
                return (static_cast<uint32_t>(ihold & 0x1F)) |
                       ((static_cast<uint32_t>(irun & 0x1F)) << 8) |
                       ((static_cast<uint32_t>(delay & 0x0F)) << 16);
        }

        // Map host-facing blank time (16/24/36/54 clk) to the TBL
        // register's 2-bit value. Unrecognized values fall back to
        // 24 clk (TBL=1).
        constexpr uint8_t blankTimeToTbl(uint8_t clk)
        {
                switch (clk) {
                case 16:
                        return 0;
                case 24:
                        return 1;
                case 36:
                        return 2;
                case 54:
                        return 3;
                default:
                        return 1;
                }
        }

        // CHOPCONF: TOFF, blank time (TBL), microsteps, INTPOL, VSENSE
        // all come from the host config. HSTRT/HEND use conservative
        // fixed values (4/0).
        constexpr uint32_t buildChopconf(uint8_t toff, uint8_t blankTimeClk, MicrostepDepth msteps,
                                         bool intpol, bool useHighSensitivity)
        {
                uint32_t v = 0;
                v |= (static_cast<uint32_t>(toff) & 0x0F);
                v |= (4u & 0x07) << 4; // HSTRT
                v |= (0u & 0x0F) << 7; // HEND
                v |= (static_cast<uint32_t>(blankTimeToTbl(blankTimeClk)) & 0x03) << 15;
                if (useHighSensitivity) {
                        v |= kChopVsense;
                }
                v |= (static_cast<uint32_t>(static_cast<uint8_t>(msteps)) & 0x0F) << kChopMresShift;
                if (intpol) {
                        v |= kChopIntpol;
                }
                return v;
        }

        // GCONF: PDN_DISABLE (so UART can talk to the chip even with
        // STEP idle), MSTEP_REG_SELECT (MRES comes from CHOPCONF, not
        // from MS1/MS2 pins), and optionally EN_SPREADCYCLE.
        constexpr uint32_t buildGconf(bool spreadCycle)
        {
                uint32_t v = kGconfPdnDisable | kGconfMstepRegSelect;
                if (spreadCycle) {
                        v |= kGconfEnSpreadCycle;
                }
                return v;
        }

        // PWMCONF default for StealthChop autoscale. The TMC2209's
        // StealthChop chopper relies on PWMCONF being initialised
        // BEFORE motion starts; otherwise the chip can deliver way
        // more (or less) current than IRUN suggests until pwm_autoscale
        // converges - and on certain motor inductances it never
        // converges well. Leaving PWMCONF at whatever value the chip
        // happened to have at boot
        // is the most likely reason a StealthChop axis runs much
        // hotter than the same motor in SpreadCycle.
        //
        // PWM_OFS / PWM_GRAD start at the datasheet reset defaults
        // (36 / 14). When autoscale / autograd are enabled the chip
        // adapts them at runtime; when disabled the host is on the
        // hook for tuning. pwm_reg / pwm_lim / freewheel stay at the
        // values TMCStepper picks - safe across the whole NEMA range.
        constexpr uint32_t buildPwmconf(bool autoscale, bool autograd, uint8_t pwmFreq)
        {
                uint32_t v = 0;
                v |= (36u & 0xFFu); // PWM_OFS
                v |= (14u & 0xFFu) << 8; // PWM_GRAD
                v |= (static_cast<uint32_t>(pwmFreq) & 0x03u) << 16; // pwm_freq
                if (autoscale) {
                        v |= (1u << 18); // pwm_autoscale
                }
                if (autograd) {
                        v |= (1u << 19); // pwm_autograd
                }
                // freewheel [21:20] = 0
                v |= (4u & 0x0Fu) << 24; // pwm_reg
                v |= (12u & 0x0Fu) << 28; // pwm_lim
                return v;
        }

        constexpr uint32_t buildCoolconf(uint8_t semin, uint8_t semaxMargin)
        {
                // COOLCONF: SEMIN[3:0], SEUP[6:5]=0 (Step1), SEMAX[11:8],
                // SEDN[14:13]=1 (Step2), SEIMIN[15]=0 (Half).
                uint32_t v = 0;
                v |= (static_cast<uint32_t>(semin) & 0x0F);
                v |= (static_cast<uint32_t>(semaxMargin) & 0x0F) << 8;
                v |= (1u & 0x03) << 13; // SEDN Step2 (gentler ramp-down)
                return v;
        }

} // namespace

// =====================================================================
// Construction
// =====================================================================

Tmc2209Driver::Tmc2209Driver(Tmc2209Config cfg, IDeviceTransport &transport,
                             IStepSignalGenerator &stepSignal, IMotionPlanner &planner)
        : cfg_(cfg)
        , transport_(transport)
        , ownedStepSignal_(nullptr)
        , ownedPlanner_(nullptr)
        , stepSignal_(stepSignal)
        , planner_(planner)
{
}

Tmc2209Driver::Tmc2209Driver(Tmc2209Config cfg, IDeviceTransport &transport)
        : cfg_(cfg)
        , transport_(transport)
        , ownedStepSignal_(std::make_unique<RmtStepSignal>())
        , ownedPlanner_(std::make_unique<TrapezoidalPlanner>())
        , stepSignal_(*ownedStepSignal_)
        , planner_(*ownedPlanner_)
{
}

// =====================================================================
// mA <-> CS conversion (static helpers, exposed for tests / diagnostics)
// =====================================================================

uint8_t Tmc2209Driver::milliampsToCs(uint16_t rmsMilliamps, float senseResistorOhms,
                                     bool useHighSensitivity)
{
        if (rmsMilliamps == 0 || senseResistorOhms <= 0.0f) {
                return 0;
        }
        const float ampsRms = static_cast<float>(rmsMilliamps) / 1000.0f;
        const float denom = vfs(useHighSensitivity);
        const float num = ampsRms * 32.0f * kSqrt2 * (senseResistorOhms + kInternalSenseOhms);
        const float cs = std::round(num / denom) - 1.0f;
        if (cs < 0.0f) {
                return 0;
        }
        if (cs > static_cast<float>(kMaxCs)) {
                return kMaxCs;
        }
        return static_cast<uint8_t>(cs);
}

uint16_t Tmc2209Driver::csToMilliamps(uint8_t cs, float senseResistorOhms, bool useHighSensitivity)
{
        if (senseResistorOhms <= 0.0f) {
                return 0;
        }
        const float ampsRms = static_cast<float>(cs + 1) / 32.0f * vfs(useHighSensitivity) /
                              (senseResistorOhms + kInternalSenseOhms) / kSqrt2;
        return static_cast<uint16_t>(std::round(ampsRms * 1000.0f));
}

// =====================================================================
// Register-level I/O (private — TMC framing lives here)
// =====================================================================

Status Tmc2209Driver::writeRegister(uint8_t reg, uint32_t value)
{
        if (cfg_.slaveAddress > kMaxSlave) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        uint8_t frame[kWriteFrame];
        frame[0] = kSync;
        frame[1] = cfg_.slaveAddress;
        frame[2] = static_cast<uint8_t>(reg | kWriteFlag);
        frame[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        frame[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
        frame[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
        frame[6] = static_cast<uint8_t>(value & 0xFF);
        frame[7] = tmcCrc(frame, 7);
        return transport_.writeFrame(frame, kWriteFrame);
}

Result<uint32_t> Tmc2209Driver::readRegister(uint8_t reg)
{
        if (cfg_.slaveAddress > kMaxSlave) {
                return Result<uint32_t>::Err(ErrorCode::InvalidConfig);
        }
        uint8_t req[kReadReq];
        req[0] = kSync;
        req[1] = cfg_.slaveAddress;
        req[2] = static_cast<uint8_t>(reg & 0x7F);
        req[3] = tmcCrc(req, 3);

        const auto wr = transport_.writeFrame(req, kReadReq);
        if (!wr.ok()) {
                return Result<uint32_t>::Err(wr.error());
        }

        // Half-duplex echo: the chip's transmitter mirrors our own
        // request bytes back onto the RX line. Consume them before
        // reading the actual reply. We don't validate the echo — just
        // drain it.
        uint8_t echo[kReadReq];
        (void)transport_.readFrame(echo, kReadReq, /*timeoutMs=*/0);

        uint8_t reply[kReadReply];
        auto rd = transport_.readFrame(reply, kReadReply, /*timeoutMs=*/0);
        if (!rd.ok()) {
                return Result<uint32_t>::Err(rd.error());
        }
        if (rd.takeValue() != kReadReply) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }

        if (reply[0] != kSync || reply[2] != static_cast<uint8_t>(reg & 0x7F)) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }
        if (tmcCrc(reply, kReadReply - 1) != reply[kReadReply - 1]) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }

        const uint32_t value =
            (static_cast<uint32_t>(reply[3]) << 24) | (static_cast<uint32_t>(reply[4]) << 16) |
            (static_cast<uint32_t>(reply[5]) << 8) | static_cast<uint32_t>(reply[6]);
        return Result<uint32_t>::Ok(value);
}

// =====================================================================
// Programming helpers
// =====================================================================

Status Tmc2209Driver::writeGconf(bool spreadCycle)
{
        return writeRegister(kRegGconf, buildGconf(spreadCycle));
}

Status Tmc2209Driver::writeChopconf()
{
        return writeRegister(kRegChopconf,
                             buildChopconf(cfg_.toff, cfg_.blankTimeClk, cfg_.microsteps,
                                           cfg_.interpolate, cfg_.useHighSensitivity));
}

Status Tmc2209Driver::writeIholdIrun()
{
        const uint8_t runCs =
            milliampsToCs(cfg_.runCurrentMa, cfg_.senseResistorOhms, cfg_.useHighSensitivity);
        const uint8_t holdCs =
            milliampsToCs(cfg_.holdCurrentMa, cfg_.senseResistorOhms, cfg_.useHighSensitivity);
        // IHOLDDELAY = 1 (~21 ms ramp run→hold after motion ends).
        return writeRegister(kRegIholdIrun, packIholdIrun(holdCs, runCs, 1));
}

Status Tmc2209Driver::writeStallConfig()
{
        if (cfg_.diagPin == GPIO_NONE) {
                return Status::Ok(); // no DIAG wired
        }
        // Fail loud if the chopper is in SpreadCycle. TMC2209's
        // StallGuard4 is StealthChop-native; SG_RESULT stays near
        // zero and DIAG never asserts in SpreadCycle mode (datasheet
        // section 14 / 15.1).
        // We just wrote GCONF, so we can decide from the intent
        // rather than re-reading the chip - no extra UART traffic.
        // A silent "stall never fires" is much harder to debug later
        // than a clear boot-time failure here.
        if (intentSpreadCycle()) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Resolve the tagged sensitivity (Pct or RawSgthrs) to the
        // 8-bit register byte. Higher SGTHRS = chip trips at higher
        // SG_RESULT = more sensitive (catches lighter loads). Note
        // that the same chip in the same setup needs a calibrated
        // value - what works for one motor will fault another.
        const uint8_t sgthrs = cfg_.stallSensitivity.toSgthrsByte();
        cachedSgthrs_ = sgthrs;
        auto s = writeRegister(kRegSgthrs, sgthrs);
        if (!s.ok()) {
                return s;
        }
        // TCOOLTHRS = 0xFFFFF: keep StallGuard armed at any non-zero
        // velocity. The lib does its own arm-window debounce on the
        // task side.
        cachedTcoolthrs_ = kTcoolthrsAlwaysOn;
        return writeRegister(kRegTcoolthrs, kTcoolthrsAlwaysOn);
}

Status Tmc2209Driver::writePwmconf()
{
        // Always write a known PWMCONF, even in SpreadCycle. The
        // chip might be picked up in any state (warm boot, prior
        // program left arbitrary values), and a deterministic baseline
        // makes behaviour reproducible. In SpreadCycle the chopper
        // path mostly ignores PWMCONF; in StealthChop it depends on
        // it for current regulation.
        return writeRegister(kRegPwmconf,
                             buildPwmconf(cfg_.pwmAutoscale, cfg_.pwmAutograd, cfg_.pwmFreq));
}

Status Tmc2209Driver::writeCoolStepConfig(bool enabled)
{
        if (enabled && cachedTcoolthrs_ == 0u) {
                // CoolStep also needs TCOOLTHRS to be armed. When
                // DIAG/stall is disabled this register has not been
                // programmed yet, so initialise it once here.
                cachedTcoolthrs_ = kTcoolthrsAlwaysOn;
                const auto s = writeRegister(kRegTcoolthrs, kTcoolthrsAlwaysOn);
                if (!s.ok()) {
                        return s;
                }
        }
        const uint32_t v = enabled ? buildCoolconf(kCoolConfSeminDefault, kCoolConfSemaxMarginDef) :
                                     0u; // SEMIN=0 disables CoolStep
        cachedCoolconf_ = v;
        return writeRegister(kRegCoolconf, v);
}

Status Tmc2209Driver::clearGstat()
{
        // Real W1C — write 1s into bits to clear them. Reading GSTAT
        // does NOT clear it on this chip.
        return writeRegister(kRegGstat, kGstatAll);
}

Status Tmc2209Driver::readIdentityFromIoin()
{
        auto r = readRegister(kRegIoin);
        if (!r.ok()) {
                // Identity read failed — keep the static default
                // (vendor=Trinamic / model=TMC2209) and report 0 fw.
                identity_.firmwareMajor = 0;
                identity_.firmwareMinor = 0;
                identity_.rawId = 0;
                return r.ok() ? Status::Ok() : Status::Err(r.error());
        }
        const uint32_t raw = r.takeValue();
        identity_.vendor = "Trinamic";
        identity_.model = "TMC2209";
        identity_.firmwareMajor = static_cast<uint8_t>((raw >> 24) & 0xFFu);
        identity_.firmwareMinor = 0;
        identity_.rawId = raw;
        return Status::Ok();
}

Status Tmc2209Driver::setEnablePinLevel(bool enabled)
{
        if (cfg_.enablePin == GPIO_NONE) {
                return Status::Ok();
        }
        const bool level = cfg_.enableActiveLow ? !enabled : enabled;
        gpio::write(cfg_.enablePin, level);
        return Status::Ok();
}

// =====================================================================
// Intent helpers
// =====================================================================

bool Tmc2209Driver::intentSpreadCycle() const
{
        // HighTorque → SpreadCycle. Quiet → StealthChop (default). If
        // both flags are set HighTorque wins (cooler high-speed +
        // torque > silence).
        if (has(activeIntent_, MotorIntent::HighTorque)) {
                return true;
        }
        return false; // StealthChop by default and when Quiet is set
}

bool Tmc2209Driver::intentCoolStepOn() const
{
        return has(activeIntent_, MotorIntent::AdaptiveCurrent) ||
               has(activeIntent_, MotorIntent::Cool) ||
               has(activeIntent_, MotorIntent::EnergySaving);
}

// =====================================================================
// IMotorDriver — lifecycle
// =====================================================================

Status Tmc2209Driver::begin()
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (cfg_.runCurrentMa == 0 || cfg_.holdCurrentMa > cfg_.runCurrentMa ||
            cfg_.senseResistorOhms <= 0.0f) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Pct mode tolerates values up to 100; raw SGTHRS tolerates
        // the full 8-bit range. Anything outside is a config bug.
        if (cfg_.stallSensitivity.unit == StallSensitivity::Unit::Pct &&
            cfg_.stallSensitivity.value > 100u) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (cfg_.stallSensitivity.unit == StallSensitivity::Unit::RawSgthrs &&
            cfg_.stallSensitivity.value > 255u) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // TOFF range per datasheet 8.1 - 0 disables the chopper entirely
        // (no motion), 15 is the field max.
        if (cfg_.toff > 15u) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Blank time must match one of the four valid TBL settings.
        if (cfg_.blankTimeClk != 16 && cfg_.blankTimeClk != 24 && cfg_.blankTimeClk != 36 &&
            cfg_.blankTimeClk != 54) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // pwm_freq is a 2-bit field (0..3) per datasheet 5.4.5.
        if (cfg_.pwmFreq > 3u) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // EN pin first — pin configured but motor not yet enabled.
        if (cfg_.enablePin != GPIO_NONE) {
                if (!gpio::configOutput(cfg_.enablePin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                (void)setEnablePinLevel(false);
        }

        // Self-owns mode: bring up the internally-allocated step
        // signal generator using the pin / timing fields in cfg_.
        // In pluggable mode the host already called begin() on its
        // own generator, so we skip this.
        if (ownedStepSignal_) {
                if (cfg_.stepPin == GPIO_NONE || cfg_.dirPin == GPIO_NONE) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                const auto sg = ownedStepSignal_->begin(cfg_.stepPin, cfg_.dirPin,
                                                        cfg_.dirActiveHigh, cfg_.dirSetupUs,
                                                        cfg_.minPulseHighUs, cfg_.minPulseLowUs);
                if (!sg.ok()) {
                        return sg;
                }
        }

        // Bring up the transport (idempotent — multiple drivers
        // sharing one transport each call this).
        auto s = transport_.begin();
        if (!s.ok()) {
                return s;
        }

        // Order matters: GCONF first (MSTEP_REG_SELECT enabled before
        // we write MRES). Then CHOPCONF. Then IHOLD_IRUN. Then optional
        // stall / CoolStep. Then GSTAT clear.
        s = writeGconf(intentSpreadCycle());
        if (!s.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }
        s = writeChopconf();
        if (!s.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }
        // PWMCONF must be initialised before motion. Without an
        // explicit write the chip retains whatever PWMCONF was
        // present - StealthChop axes
        // would then run with random PWM coefficients and the chopper
        // can deliver way more current than IRUN suggests, heating
        // the motor. Reset defaults (autoscale + autograd) make the
        // chopper adapt to the actual motor at runtime.
        s = writePwmconf();
        if (!s.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }
        s = writeIholdIrun();
        if (!s.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }
        s = writeStallConfig();
        if (!s.ok()) {
                // Preserve the InvalidConfig that writeStallConfig
                // raises for "SpreadCycle + DIAG wired"; only map
                // unknown failures to TransportError.
                return (s.error() == ErrorCode::InvalidConfig) ?
                           s :
                           Status::Err(ErrorCode::TransportError);
        }
        s = writeCoolStepConfig(intentCoolStepOn());
        if (!s.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }
        s = clearGstat();
        if (!s.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }

        // Read identity once. Failure or an obviously-bogus version
        // byte (0x00 or 0xFF) means no chip is responding on this
        // slave address — refuse to come up so the host can detect
        // a wiring / power / address fault BEFORE motion is armed.
        // Production TMC2209 silicon reports 0x21 in IOIN[31:24];
        // 0x00 / 0xFF are the canonical "floating bus" patterns.
        const auto idStatus = readIdentityFromIoin();
        if (!idStatus.ok()) {
                return Status::Err(ErrorCode::TransportError);
        }
        if (identity_.firmwareMajor == 0x00u || identity_.firmwareMajor == 0xFFu) {
                return Status::Err(ErrorCode::TransportError);
        }

        begun_ = true;
        return Status::Ok();
}

Status Tmc2209Driver::enable()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        return setEnablePinLevel(true);
}

Status Tmc2209Driver::disable()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        return setEnablePinLevel(false);
}

Status Tmc2209Driver::clearFault()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        // Clear chip-side fault flags AND any latched state in the
        // step signal generator.
        (void)clearGstat();
        return stepSignal_.clearFault();
}

// =====================================================================
// IMotorDriver — motion
// =====================================================================

Status Tmc2209Driver::armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps,
                              uint32_t accelSps2, uint32_t decelSps2)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (targetSteps == 0 || cruiseSps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // accel / decel of zero are valid - the planner treats them
        // as "no ramp, instant cruise". cruiseSps must still be > 0.

        PlannerLimits L;
        L.maxVelocitySps = cruiseSps;
        L.accelSpsPerSec = accelSps2;
        L.decelSpsPerSec = decelSps2;
        L.hardStepRateCeilingSps = cruiseSps; // axis-side already clamped
        // Pulse widths come from the chip config so the planner
        // honors host-configured electrical timing.
        L.minPulseHighUs = cfg_.minPulseHighUs;
        L.minPulseLowUs  = cfg_.minPulseLowUs;

        const Position from = stepSignal_.commandedPosition();
        const Position to = (dir == Direction::Forward) ? from + static_cast<int32_t>(targetSteps) :
                                                          from - static_cast<int32_t>(targetSteps);
        lastPlannedMove_ = planner_.planMove(from, to, L, stepSignal_.timerResolutionHz(),
                                             stepSignal_.minTimerTicks());
        if (lastPlannedMove_.totalSteps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        return stepSignal_.armMove(lastPlannedMove_);
}

Status Tmc2209Driver::armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (cruiseSps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // accel of zero is a valid no-ramp request - the planner emits
        // a single cruise segment.
        PlannerLimits L;
        L.maxVelocitySps = cruiseSps;
        L.accelSpsPerSec = accelSps2;
        L.decelSpsPerSec = accelSps2;
        L.hardStepRateCeilingSps = cruiseSps;
        L.minPulseHighUs = cfg_.minPulseHighUs;
        L.minPulseLowUs  = cfg_.minPulseLowUs;

        // Practically-infinite jog cap (~2.1 G steps; at 200 kpps ≈ 3 h).
        constexpr uint32_t kJogIndefiniteSteps = 0x7FFFFFFFu;
        lastPlannedMove_ = planner_.planJog(dir, kJogIndefiniteSteps, L,
                                            stepSignal_.timerResolutionHz(),
                                            stepSignal_.minTimerTicks());
        if (lastPlannedMove_.totalSteps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        return stepSignal_.armMove(lastPlannedMove_);
}

Status Tmc2209Driver::stop(StopMode mode)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        return stepSignal_.stop(mode);
}

DriverMotionStatus Tmc2209Driver::motionStatus() const
{
        const auto s = stepSignal_.status();
        DriverMotionStatus out;
        out.running = s.running;
        out.faulted = s.faulted;
        out.finishedReason = s.finishedReason;
        return out;
}

Position Tmc2209Driver::commandedPositionSteps() const
{
        return stepSignal_.commandedPosition();
}

uint32_t Tmc2209Driver::commandedSpsNow() const
{
        return stepSignal_.commandedSpsNow();
}

Status Tmc2209Driver::resetPosition(Position newSteps)
{
        return stepSignal_.resetPosition(newSteps);
}

// =====================================================================
// IMotorDriver — identity / intent / diagnostics
// =====================================================================

DriverIdentity Tmc2209Driver::identity()
{
        return identity_;
}

IntentSupport Tmc2209Driver::applyIntent(MotorIntent intent)
{
        // SpreadCycle + DIAG wired = StallGuard goes silent on this
        // chip (datasheet 14 / 15.1). Refuse the request before we
        // touch the chip; leave `activeIntent_` unchanged.
        const bool wantSpreadProposed = has(intent, MotorIntent::HighTorque);
        if (wantSpreadProposed && cfg_.diagPin != GPIO_NONE && begun_) {
                return IntentSupport::Conflicted;
        }

        activeIntent_ = intent;
        // If the chip isn't up yet we just record; begin() will pick
        // it up. Otherwise re-write GCONF + COOLCONF.
        if (!begun_) {
                return IntentSupport::Supported;
        }

        const bool wantSpread = intentSpreadCycle();
        const bool wantCool = intentCoolStepOn();

        bool anyConflict = false;
        if (has(intent, MotorIntent::Quiet) && has(intent, MotorIntent::HighTorque)) {
                anyConflict = true;
        }

        auto s = writeGconf(wantSpread);
        if (!s.ok()) {
                return IntentSupport::Unsupported;
        }
        s = writeCoolStepConfig(wantCool);
        if (!s.ok()) {
                return IntentSupport::Unsupported;
        }

        // `Precision` maps to INTPOL which is part of CHOPCONF — re-
        // write CHOPCONF to honour the host's intent.
        s = writeChopconf();
        if (!s.ok()) {
                return IntentSupport::Unsupported;
        }

        if (anyConflict) {
                return IntentSupport::PartiallySupported;
        }
        return IntentSupport::Supported;
}

Result<Tmc2209Driver::StallSnapshot> Tmc2209Driver::readStallSnapshot()
{
        StallSnapshot s{};
        s.ok = false;
        if (!begun_) {
                return Result<StallSnapshot>::Err(ErrorCode::NotInitialized);
        }

        // Write-only registers - cached at write time. The chip
        // returns 0 if we try to read these back.
        s.sgthrsWritten    = cachedSgthrs_;
        s.tcoolthrsWritten = cachedTcoolthrs_;
        s.coolconfWritten  = cachedCoolconf_;

        // Pre-computed convenience values so the host doesn't repeat
        // chip-specific math at every dump site. Both follow datasheet
        // 14.1 / 14.2: the chip pulls DIAG HIGH when SG_RESULT falls
        // below SGTHRS * 2, and StallGuard4 / CoolStep arm when the
        // live TSTEP drops below the 20-bit TCOOLTHRS field.
        s.diagTripsBelowSgResult =
            static_cast<uint16_t>(static_cast<uint16_t>(s.sgthrsWritten) * 2u);
        s.tcoolthrsClkCycles = s.tcoolthrsWritten & 0xFFFFFu;

        auto pull = [&](uint8_t reg, uint32_t &out) -> bool {
                auto r = readRegister(reg);
                if (!r.ok()) {
                        return false;
                }
                out = r.takeValue();
                return true;
        };

        bool allOk = true;
        allOk &= pull(kRegSgResult,  s.sgResultRaw);
        allOk &= pull(kRegGconf,     s.gconf);
        allOk &= pull(kRegChopconf,  s.chopconf);
        allOk &= pull(kRegIoin,      s.ioin);
        allOk &= pull(kRegTstep,     s.tstep);
        allOk &= pull(kRegDrvStatus, s.drvStatus);
        allOk &= pull(kRegIfcnt,     s.ifcnt);

        // SG_RESULT is 10-bit at [9:0]; the upper bits are reserved
        // and read as 0 on this chip, but masking is the contract.
        s.sgResult = static_cast<uint16_t>(s.sgResultRaw & 0x3FFu);

        // DIAG is IOIN bit 4 on TMC2209 (datasheet table 5.5). The
        // bit 10 is reserved and always reads 0.
        s.diagLevelHigh = ((s.ioin >> 4) & 0x1u) != 0u;
        // StallGuard4 needs StealthChop (en_SpreadCycle = 0 in GCONF
        // bit 2). When this bit is set SG_RESULT stays near zero
        // regardless of load - the chip's load estimator only runs
        // in the StealthChop chopper path.
        s.stealthChopActive = ((s.gconf >> 2) & 0x1u) == 0u;
        s.ok = allOk;
        return Result<StallSnapshot>::Ok(s);
}

void Tmc2209Driver::fillDriverDiagnostics(MotorDiagnostics &out) const
{
        out.identity = identity_;
        out.totalStepsIssued = 0; // Step signal generator doesn't expose this yet.

        // SG_RESULT read — only if DIAG pin wired (otherwise the chip
        // hasn't been programmed for StallGuard so the read is
        // meaningless).
        if (cfg_.diagPin != GPIO_NONE && begun_) {
                // const_cast is acceptable here: we read a chip
                // register (no observable internal state change in the
                // driver), and `fillDriverDiagnostics` is the
                // conventional const observer that the const-ness of
                // the queries forces. The transport call itself is
                // task-context (blocking ~1.5 ms at 115200 baud) — the
                // host shouldn't poll this from a hot path; see the
                // doc on `IMotorDriver::fillDriverDiagnostics`.
                auto *self = const_cast<Tmc2209Driver *>(this);
                auto r = self->readRegister(kRegSgResult);
                if (r.ok()) {
                        const uint32_t raw = r.takeValue();
                        // SG_RESULT is 10-bit (0..1023). 0 = stalled,
                        // 1023 = free-running. Map to a "load %" where
                        // higher = more loaded.
                        const uint32_t sg10 = raw & 0x3FFu;
                        const uint32_t loadPct = (sg10 >= 1023u) ? 0u :
                                                                   100u - ((sg10 * 100u) / 1023u);
                        out.stall_valid = true;
                        // Diagnostics report the resolved sensitivity
                        // as a 0..100 percentage regardless of which
                        // unit the host configured, so serial / web
                        // UIs do not need to know about raw SGTHRS.
                        // For Pct input this is the original value;
                        // for RawSgthrs input it's the equivalent
                        // percentage (lossy by at most 1 LSB).
                        const uint8_t sgthrs = cfg_.stallSensitivity.toSgthrsByte();
                        out.stallSensitivityPct =
                            static_cast<uint8_t>((static_cast<uint32_t>(sgthrs) * 100u) / 255u);
                        out.stallReadingPct = static_cast<uint8_t>(loadPct);
                }
        }

        // Adaptive-current visibility: if CoolStep is on, we don't
        // know the live coil current without an additional read; mark
        // valid and report the configured IRUN as the upper bound.
        if (intentCoolStepOn() && begun_) {
                out.adaptive_current_valid = true;
                out.adaptiveCurrentMa = cfg_.runCurrentMa;
        }
}

} // namespace ungula::motor::tmc2209
