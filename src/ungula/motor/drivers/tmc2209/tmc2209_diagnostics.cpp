// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h"

#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"

namespace ungula::motor::tmc2209
{

// =====================================================================
// Snapshot accessors
// =====================================================================

bool Tmc2209StatusSnapshot::overTemperature() const
{
        return drvStatus & drv_status::OT;
}
bool Tmc2209StatusSnapshot::overTemperaturePreWarn() const
{
        return drvStatus & drv_status::OTPW;
}
bool Tmc2209StatusSnapshot::shortToGround() const
{
        return drvStatus & (drv_status::S2GA | drv_status::S2GB);
}
bool Tmc2209StatusSnapshot::openLoad() const
{
        return drvStatus & (drv_status::OLA | drv_status::OLB);
}
bool Tmc2209StatusSnapshot::standstill() const
{
        return drvStatus & drv_status::STST;
}
bool Tmc2209StatusSnapshot::driveFault() const
{
        return drvStatus & drv_status::FAULT_MASK;
}

// =====================================================================
// Diagnostics
// =====================================================================

Tmc2209Diagnostics::Tmc2209Diagnostics(ITmcUart &uart)
        : uart_(uart)
{
}

Status Tmc2209Diagnostics::refresh()
{
        // Read three registers in a fixed order. On the first transport
        // failure, return the error but keep partial state cached so the
        // caller can still inspect what made it through.
        auto r1 = uart_.readRegister(reg::DRV_STATUS);
        if (!r1.ok())
                return Status::Err(r1.error());
        cached_.drvStatus = r1.takeValue();

        auto r2 = uart_.readRegister(reg::IOIN);
        if (!r2.ok())
                return Status::Err(r2.error());
        cached_.ioin = r2.takeValue();

        auto r3 = uart_.readRegister(reg::GSTAT);
        if (!r3.ok())
                return Status::Err(r3.error());
        cached_.gstat = r3.takeValue();

        return Status::Ok();
}

Result<uint32_t> Tmc2209Diagnostics::readRegister(uint8_t reg)
{
        return uart_.readRegister(reg);
}

Status Tmc2209Diagnostics::dumpRegisters(const uint8_t *regs, uint32_t *values, uint8_t count,
                                         uint8_t *successfulCount)
{
        if (!regs || !values)
                return Status::Err(ErrorCode::InvalidConfig);
        if (successfulCount)
                *successfulCount = 0;

        for (uint8_t i = 0; i < count; ++i) {
                auto r = uart_.readRegister(regs[i]);
                if (!r.ok())
                        return Status::Err(r.error());
                values[i] = r.takeValue();
                if (successfulCount)
                        (*successfulCount)++;
        }
        return Status::Ok();
}

} // namespace ungula::motor::tmc2209
