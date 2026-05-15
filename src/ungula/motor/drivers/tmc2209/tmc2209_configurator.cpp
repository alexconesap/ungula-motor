// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_configurator.h"

#include <cmath>

#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"

namespace ungula::motor::tmc2209
{

namespace
{

        constexpr uint8_t kMaxCurrent = 31;
        constexpr uint8_t kMaxIHoldDelay = 15;

        // TMC2209 current-scaling constants (datasheet §9.1, §5.5.1).
        // I_rms = (CS+1)/32 * V_fs / (R_sense + 0.02 Ω) * 1/√2
        // V_fs depends on the CHOPCONF.vsense bit:
        //   vsense = 0 → V_fs = 0.325 V (low-sensitivity, default)
        //   vsense = 1 → V_fs = 0.180 V (high-sensitivity, low-current)
        constexpr float kVfsLowSensitivity = 0.325f;
        constexpr float kVfsHighSensitivity = 0.180f;
        constexpr float kInternalSenseOhms = 0.02f;
        constexpr float kSqrt2 = 1.41421356f;

        float vfs(bool useHighSensitivity)
        {
                return useHighSensitivity ? kVfsHighSensitivity : kVfsLowSensitivity;
        }

        /// Pack IHOLD / IRUN / IHOLDDELAY into the wire-format word per
        /// datasheet §5.5.1: bits 4..0 = IHOLD, 12..8 = IRUN, 19..16 =
        /// IHOLDDELAY. All other bits reserved (zero).
        constexpr uint32_t packIholdIrun(uint8_t ihold, uint8_t irun, uint8_t delay)
        {
                return (static_cast<uint32_t>(ihold & 0x1F)) |
                       ((static_cast<uint32_t>(irun & 0x1F)) << 8) |
                       ((static_cast<uint32_t>(delay & 0x0F)) << 16);
        }

        /// CHOPCONF skeleton with sane defaults for stealthChop. TBL=2 (24 clk
        /// blank time), HSTRT=4, HEND=0. TOFF and MRES are caller-supplied.
        constexpr uint32_t buildChopconf(uint8_t toff, Microsteps msteps, bool intpol,
                                         bool useHighSensitivity)
        {
                uint32_t v = 0;
                v |= (static_cast<uint32_t>(toff) & 0x0F); // [3:0]
                v |= (static_cast<uint32_t>(4) & 0x07) << 4; // HSTRT
                v |= (static_cast<uint32_t>(0) & 0x0F) << 7; // HEND
                v |= (static_cast<uint32_t>(2) & 0x03) << 15; // TBL
                if (useHighSensitivity)
                        v |= chopconf::VSENSE;
                v |= (static_cast<uint32_t>(static_cast<uint8_t>(msteps)) & 0x0F)
                     << chopconf::MRES_SHIFT;
                if (intpol)
                        v |= chopconf::INTPOL;
                return v;
        }

        constexpr uint32_t buildGconf(ChopperMode mode)
        {
                // PDN_DISABLE so the UART can talk to the chip when the host
                // hasn't pulled PDN_UART low externally; MSTEP_REG_SELECT so
                // MRES comes from CHOPCONF (not the MS1/MS2 pins).
                uint32_t v = gconf::PDN_DISABLE | gconf::MSTEP_REG_SELECT;
                if (mode == ChopperMode::SpreadCycle)
                        v |= gconf::EN_SPREADCYCLE;
                return v;
        }

} // namespace

Tmc2209Configurator::Tmc2209Configurator(ITmcUart &uart)
        : uart_(uart)
{
}

uint8_t Tmc2209Configurator::milliampsToCs(uint16_t rmsMilliamps,
                                             float    senseResistorOhms,
                                             bool     useHighSensitivity)
{
        if (rmsMilliamps == 0 || senseResistorOhms <= 0.0f)
                return 0;
        // I_rms = (CS+1)/32 * V_fs / (R_sense + 0.02) / √2
        // Solve for CS: CS = round(I_rms * 32 * √2 * (R_sense + 0.02) / V_fs - 1)
        const float ampsRms = static_cast<float>(rmsMilliamps) / 1000.0f;
        const float denom = vfs(useHighSensitivity);
        const float num = ampsRms * 32.0f * kSqrt2 * (senseResistorOhms + kInternalSenseOhms);
        const float cs = std::round(num / denom) - 1.0f;
        if (cs < 0.0f)
                return 0;
        if (cs > static_cast<float>(kMaxCurrent))
                return kMaxCurrent;
        return static_cast<uint8_t>(cs);
}

uint16_t Tmc2209Configurator::csToMilliamps(uint8_t cs,
                                              float   senseResistorOhms,
                                              bool    useHighSensitivity)
{
        if (senseResistorOhms <= 0.0f)
                return 0;
        const float ampsRms = static_cast<float>(cs + 1) / 32.0f * vfs(useHighSensitivity) /
                              (senseResistorOhms + kInternalSenseOhms) / kSqrt2;
        return static_cast<uint16_t>(std::round(ampsRms * 1000.0f));
}

Status Tmc2209Configurator::begin(const Config &cfg)
{
        if (begun_)
                return Status::Err(ErrorCode::AlreadyInitialized);

        if (cfg.runCurrentMa == 0 || cfg.holdCurrentMa > cfg.runCurrentMa ||
            cfg.iHoldDelay > kMaxIHoldDelay || cfg.toff > 15 ||
            cfg.senseResistorOhms <= 0.0f) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Cache the sense-resistor math knobs so runtime `setCurrents`
        // can repeat the conversion without re-passing them.
        senseResistorOhms_ = cfg.senseResistorOhms;
        useHighSensitivity_ = cfg.useHighSensitivity;

        const uint8_t runCs = milliampsToCs(cfg.runCurrentMa, cfg.senseResistorOhms,
                                              cfg.useHighSensitivity);
        const uint8_t holdCs = milliampsToCs(cfg.holdCurrentMa, cfg.senseResistorOhms,
                                              cfg.useHighSensitivity);

        shadowGconf_ = buildGconf(cfg.mode);
        shadowChopconf_ = buildChopconf(cfg.toff, cfg.microsteps, cfg.interpolate,
                                          cfg.useHighSensitivity);
        shadowIholdIrun_ = packIholdIrun(holdCs, runCs, cfg.iHoldDelay);

        // Order matters: GCONF before CHOPCONF so MSTEP_REG_SELECT is
        // active when we write MRES. IHOLD_IRUN last so current ramps to
        // the configured run value only after the chopper is shaped.
        auto s = uart_.writeRegister(reg::GCONF, shadowGconf_);
        if (!s.ok())
                return s;

        s = uart_.writeRegister(reg::CHOPCONF, shadowChopconf_);
        if (!s.ok())
                return s;

        s = uart_.writeRegister(reg::IHOLD_IRUN, shadowIholdIrun_);
        if (!s.ok())
                return s;

        // Clear latched boot flags so the first DRV_STATUS read doesn't
        // surface stale RESET / UV_CP from power-up. W1C: write 1s into
        // the bits we want cleared.
        s = clearGstat();
        if (!s.ok())
                return s;

        begun_ = true;
        return Status::Ok();
}

Status Tmc2209Configurator::setCurrents(uint16_t runCurrentMa, uint16_t holdCurrentMa,
                                          uint8_t iHoldDelay)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (runCurrentMa == 0 || holdCurrentMa > runCurrentMa || iHoldDelay > kMaxIHoldDelay) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        const uint8_t runCs = milliampsToCs(runCurrentMa, senseResistorOhms_,
                                              useHighSensitivity_);
        const uint8_t holdCs = milliampsToCs(holdCurrentMa, senseResistorOhms_,
                                              useHighSensitivity_);
        shadowIholdIrun_ = packIholdIrun(holdCs, runCs, iHoldDelay);
        return uart_.writeRegister(reg::IHOLD_IRUN, shadowIholdIrun_);
}

Status Tmc2209Configurator::setMicrosteps(Microsteps msteps)
{
        if (static_cast<uint8_t>(msteps) > 8) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Replace just the MRES nibble; preserve TOFF / TBL / INTPOL etc.
        shadowChopconf_ &= ~chopconf::MRES_MASK;
        shadowChopconf_ |= (static_cast<uint32_t>(static_cast<uint8_t>(msteps)) & 0x0F)
                           << chopconf::MRES_SHIFT;
        return uart_.writeRegister(reg::CHOPCONF, shadowChopconf_);
}

Status Tmc2209Configurator::setChopperMode(ChopperMode mode)
{
        if (mode == ChopperMode::SpreadCycle) {
                shadowGconf_ |= gconf::EN_SPREADCYCLE;
        } else {
                shadowGconf_ &= ~gconf::EN_SPREADCYCLE;
        }
        return uart_.writeRegister(reg::GCONF, shadowGconf_);
}

Status Tmc2209Configurator::clearGstat()
{
        // Real W1C — write the bits you want cleared. Do NOT read GSTAT;
        // reading does NOT clear flags on the TMC2209.
        return uart_.writeRegister(reg::GSTAT, gstat::ALL_FLAGS);
}

} // namespace ungula::motor::tmc2209
