// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_coolstep.h"

#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"

namespace ungula::motor::tmc2209
{

namespace
{
constexpr uint32_t kTCoolThrsMask20Bit = 0xFFFFF;

constexpr uint8_t kMaxSemin = 15;
constexpr uint8_t kMaxSemax = 15;
}  // namespace

Tmc2209CoolStep::Tmc2209CoolStep(ITmcUart& uart)
    : uart_(uart)
{
}

Status Tmc2209CoolStep::begin(const Config& cfg)
{
    if (cfg.loadThresholdToIncrease > kMaxSemin ||
        cfg.loadThresholdToDecreaseMargin > kMaxSemax) {
        return Status::Err(ErrorCode::InvalidConfig);
    }

    // Build COOLCONF. Datasheet §5.5 packing:
    //   bits  3..0 = SEMIN  (load threshold to increase; 0 disables CS)
    //   bits  6..5 = SEUP   (current up-step width, 2^N steps)
    //   bits 11..8 = SEMAX  (load margin to decrease)
    //   bits 14..13 = SEDN  (current down-step width, 2^N steps)
    //   bit 15     = SEIMIN (0 = floor IRUN/2, 1 = floor IRUN/4)
    uint32_t v = 0;
    v |= (static_cast<uint32_t>(cfg.loadThresholdToIncrease) & 0xFu)
            << coolconf::SEMIN_SHIFT;
    v |= (static_cast<uint32_t>(static_cast<uint8_t>(cfg.currentRampUp)) & 0x3u)
            << coolconf::SEUP_SHIFT;
    v |= (static_cast<uint32_t>(cfg.loadThresholdToDecreaseMargin) & 0xFu)
            << coolconf::SEMAX_SHIFT;
    v |= (static_cast<uint32_t>(static_cast<uint8_t>(cfg.currentRampDown)) & 0x3u)
            << coolconf::SEDN_SHIFT;
    if (cfg.currentFloor == CurrentFloor::Quarter) {
        v |= coolconf::SEIMIN;
    }
    shadowCoolconf_ = v;

    auto s = uart_.writeRegister(reg::COOLCONF, shadowCoolconf_);
    if (!s.ok()) return s;

    // Optional TCOOLTHRS write — see Config doc. Skipping leaves the
    // register at whatever StallGuard (or a previous begin()) wrote.
    if (cfg.tCoolThrs != 0) {
        uint32_t t = cfg.tCoolThrs;
        if (t > kTCoolThrsMask20Bit) t = kTCoolThrsMask20Bit;
        shadowTCoolThrs_ = t;
        s = uart_.writeRegister(reg::TCOOLTHRS, t);
        if (!s.ok()) return s;
    }
    return Status::Ok();
}

Status Tmc2209CoolStep::disable()
{
    // SEMIN = 0 turns CoolStep off per datasheet. Preserve the rest
    // of COOLCONF in the shadow so a future begin() doesn't have to
    // recompute it from scratch.
    shadowCoolconf_ &= ~coolconf::SEMIN_MASK;
    return uart_.writeRegister(reg::COOLCONF, shadowCoolconf_);
}

}  // namespace ungula::motor::tmc2209
