// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_stallguard.h"

#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"

namespace ungula::motor::tmc2209
{

namespace
{
        constexpr uint32_t kTCoolThrsMask20Bit = 0xFFFFF;
}

Tmc2209StallGuard::Tmc2209StallGuard(ITmcUart &uart)
        : uart_(uart)
{
}

Status Tmc2209StallGuard::begin(const Config &cfg)
{
        // StallGuard4 on the TMC2209 is the StealthChop-native variant.
        // SpreadCycle's older SG2 is not available on this chip — the
        // DIAG output stays silent if the chopper is in SpreadCycle.
        // Read GCONF and fail loudly if EN_SPREADCYCLE is set; that's
        // the only failure mode that produces "stall config succeeds but
        // no stall ever fires", which is much harder to diagnose later.
        // ~1.5 ms one-shot UART round-trip at 115200 baud — at boot only.
        if (cfg.verifyChopperMode) {
                auto gconfRead = uart_.readRegister(reg::GCONF);
                if (!gconfRead.ok())
                        return Status::Err(gconfRead.error());
                if ((gconfRead.takeValue() & gconf::EN_SPREADCYCLE) != 0u) {
                        // The chopper is in SpreadCycle. StallGuard cannot
                        // function here. Caller must either switch to
                        // StealthChop first (via Tmc2209Configurator) or
                        // pass `verifyChopperMode = false` to acknowledge
                        // that they know the chip will not report stalls.
                        return Status::Err(ErrorCode::InvalidConfig);
                }
        }

        auto s = setSgThreshold(cfg.sgThreshold);
        if (!s.ok())
                return s;

        s = setTCoolThrs(cfg.tCoolThrs);
        if (!s.ok())
                return s;

        (void)cfg.enableDiagOutput;
        // DIAG pin is hardware-driven by SG comparison. Nothing to write
        // here today; the field is kept for forward compatibility if a
        // future feature needs to route the signal elsewhere.
        return Status::Ok();
}

Status Tmc2209StallGuard::setSgThreshold(uint8_t threshold)
{
        shadowSgThreshold_ = threshold;
        return uart_.writeRegister(reg::SGTHRS, static_cast<uint32_t>(threshold));
}

Status Tmc2209StallGuard::setTCoolThrs(uint32_t value)
{
        // TCOOLTHRS is a 20-bit field; clamp silently rather than refusing,
        // since "any reasonable threshold" maps to the wire format the same
        // way and the caller likely meant "as fast as possible".
        if (value > kTCoolThrsMask20Bit)
                value = kTCoolThrsMask20Bit;
        shadowTCoolThrs_ = value;
        return uart_.writeRegister(reg::TCOOLTHRS, value);
}

Result<uint16_t> Tmc2209StallGuard::readSgResult()
{
        auto r = uart_.readRegister(reg::SG_RESULT);
        if (!r.ok())
                return Result<uint16_t>::Err(r.error());
        // SG_RESULT lives in bits [9:0] of the register (10-bit unsigned).
        return Result<uint16_t>::Ok(static_cast<uint16_t>(r.takeValue() & 0x3FFu));
}

} // namespace ungula::motor::tmc2209
