// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/result.h"

namespace ungula::motor::tmc2209
{

/// Snapshot of selected TMC2209 status registers. Used by
/// `Tmc2209Diagnostics` so a single `refresh()` call costs one round-
/// trip per register and the host can read fields out without further
/// UART traffic.
struct Tmc2209StatusSnapshot {
        uint32_t drvStatus = 0; ///< Raw DRV_STATUS bits
        uint32_t ioin = 0; ///< Raw IOIN bits
        uint32_t gstat = 0; ///< Raw GSTAT bits at refresh time

        /// Convenience accessors over `drvStatus`.
        bool overTemperature() const;
        bool overTemperaturePreWarn() const;
        bool shortToGround() const;
        bool openLoad() const;
        bool standstill() const;
        /// Any drive-fault condition: OT, S2G[AB], S2VS[AB]. Pre-warn and
        /// open-load are notable but not faults.
        bool driveFault() const;
};

/// Read-only diagnostics over a TMC2209. Strictly opt-in, strictly off
/// the motion-timing path:
///
///   - No reads happen in `Tmc2209Configurator::begin()`.
///   - The host calls `refresh()` from a low-priority task (typical
///     cadence: 100 ms or on-demand after a fault).
///   - `dumpRegisters()` exists for debugging and prints to nothing —
///     the host owns logging; this method just fills a caller-supplied
///     buffer with raw register values.
///
/// All UART operations are blocking; an unresponsive bus blocks the
/// calling task for up to `responseTimeoutMs` per read. The motion
/// path is unaffected because the pulse engine lives in the timer ISR.
class Tmc2209Diagnostics {
    public:
        /// `uart` must outlive the diagnostics instance.
        explicit Tmc2209Diagnostics(ITmcUart &uart);

        Tmc2209Diagnostics(const Tmc2209Diagnostics &) = delete;
        Tmc2209Diagnostics &operator=(const Tmc2209Diagnostics &) = delete;

        /// Read DRV_STATUS / IOIN / GSTAT in one round-trip group and
        /// cache them. Returns the first transport error encountered;
        /// successful reads up to that point ARE applied to the snapshot
        /// (partial refresh is better than discarding usable data — the
        /// host can check `snapshot().drvStatus != 0` to know if at
        /// least one register made it).
        Status refresh();

        /// Latest cached snapshot. Zero-initialised before the first
        /// successful `refresh()`.
        const Tmc2209StatusSnapshot &snapshot() const
        {
                return cached_;
        }

        /// Read a single register on demand. Bypasses the cache.
        Result<uint32_t> readRegister(uint8_t reg);

        /// Read up to 16 specific registers into a caller-supplied buffer.
        /// Used for debug dumps. `regs` and `values` must be at least
        /// `count` long. Stops at the first transport error and returns
        /// the error; `*successfulCount` (if non-null) tells the caller
        /// how many entries were filled.
        Status dumpRegisters(const uint8_t *regs, uint32_t *values, uint8_t count,
                             uint8_t *successfulCount = nullptr);

    private:
        ITmcUart &uart_;
        Tmc2209StatusSnapshot cached_{};
};

} // namespace ungula::motor::tmc2209
