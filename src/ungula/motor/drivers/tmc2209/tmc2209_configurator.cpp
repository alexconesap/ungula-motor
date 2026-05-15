// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_configurator.h"

#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"

namespace ungula::motor::tmc2209
{

namespace
{

        constexpr uint8_t kMaxCurrent = 31;
        constexpr uint8_t kMaxIHoldDelay = 15;

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
        constexpr uint32_t buildChopconf(uint8_t toff, Microsteps msteps, bool intpol)
        {
                uint32_t v = 0;
                v |= (static_cast<uint32_t>(toff) & 0x0F); // [3:0]
                v |= (static_cast<uint32_t>(4) & 0x07) << 4; // HSTRT
                v |= (static_cast<uint32_t>(0) & 0x0F) << 7; // HEND
                v |= (static_cast<uint32_t>(2) & 0x03) << 15; // TBL
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

Status Tmc2209Configurator::begin(const Config &cfg)
{
        if (begun_)
                return Status::Err(ErrorCode::AlreadyInitialized);

        if (cfg.runCurrent > kMaxCurrent || cfg.holdCurrent > kMaxCurrent ||
            cfg.iHoldDelay > kMaxIHoldDelay || cfg.toff > 15) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        shadowGconf_ = buildGconf(cfg.mode);
        shadowChopconf_ = buildChopconf(cfg.toff, cfg.microsteps, cfg.interpolate);
        shadowIholdIrun_ = packIholdIrun(cfg.holdCurrent, cfg.runCurrent, cfg.iHoldDelay);

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

Status Tmc2209Configurator::setCurrents(uint8_t runCurrent, uint8_t holdCurrent, uint8_t iHoldDelay)
{
        if (runCurrent > kMaxCurrent || holdCurrent > kMaxCurrent || iHoldDelay > kMaxIHoldDelay) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        shadowIholdIrun_ = packIholdIrun(holdCurrent, runCurrent, iHoldDelay);
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
