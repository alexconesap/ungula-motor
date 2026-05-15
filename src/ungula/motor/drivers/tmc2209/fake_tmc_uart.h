// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <array>
#include <cstdint>

#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_registers.h"

namespace ungula::motor::tmc2209
{

/// Header-only test fake. Models a TMC2209's register file in memory:
/// writes update the cell, reads return whatever's there. Tests can
/// pre-seed registers (typically `DRV_STATUS`, `IOIN`, `SG_RESULT`)
/// to exercise diagnostics and stallguard logic.
///
/// Special cases match the real chip:
///   - `IFCNT` is incremented automatically on every successful write
///     (read-only on the wire; the fake honours the increment but
///     allows direct seeding for test setup).
///   - `GSTAT` is W1C: writing 1s into a bit clears that bit; writing
///     0s leaves it unchanged. Mirrors the real silicon so the
///     `clearGstat` regression test fails if the configurator
///     accidentally does a read.
///
/// All counters and the register file are public so tests can inspect
/// them after exercising the driver.
class FakeTmcUart final : public ITmcUart {
    public:
        Status writeRegister(uint8_t reg, uint32_t value) override
        {
                writeCallCount++;
                lastWrittenReg = reg;
                lastWrittenValue = value;

                if (failNextWrite) {
                        failNextWrite = false;
                        return Status::Err(ErrorCode::TransportError);
                }

                if (reg == reg::GSTAT) {
                        // Real chip: write-1-to-clear. Clear bits that the caller
                        // wrote as 1; leave bits the caller wrote as 0 untouched.
                        registers[reg::GSTAT] &= ~value;
                } else {
                        registers[reg] = value;
                }
                // IFCNT increments by 1 on every successful write (modulo 256
                // on real silicon — fake mirrors the wrap-around).
                registers[reg::IFCNT] = (registers[reg::IFCNT] + 1) & 0xFFu;
                return Status::Ok();
        }

        Result<uint32_t> readRegister(uint8_t reg) override
        {
                readCallCount++;
                lastReadReg = reg;

                if (failNextRead) {
                        failNextRead = false;
                        return Result<uint32_t>::Err(ErrorCode::TransportError);
                }
                return Result<uint32_t>::Ok(registers[reg]);
        }

        // ---- Test seeding API --------------------------------------------

        /// Direct-write a register without touching IFCNT. Useful for
        /// pre-seeding DRV_STATUS, SG_RESULT, IOIN before exercising
        /// readers.
        void seed(uint8_t reg, uint32_t value)
        {
                registers[reg] = value;
        }

        // ---- Counters / inspection ---------------------------------------

        std::array<uint32_t, 256> registers{};

        uint32_t writeCallCount = 0;
        uint32_t readCallCount = 0;
        uint8_t lastWrittenReg = 0;
        uint32_t lastWrittenValue = 0;
        uint8_t lastReadReg = 0;

        bool failNextWrite = false;
        bool failNextRead = false;
};

} // namespace ungula::motor::tmc2209
