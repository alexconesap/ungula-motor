// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/result.h"

namespace ungula::motor::tmc2209
{

/// Abstract TMC2209 UART transport. Hides the SYNC / slave-address /
/// CRC framing from the Configurator / Diagnostics / StallGuard layers
/// — they just see "write this 32-bit value to register X" and "read
/// register Y, give me 32 bits or an error".
///
/// ## Why an interface
///
/// Three reasons:
///   1. The driver components are unit-testable against a `FakeTmcUart`
///      that asserts on register access patterns. No real UART required.
///   2. Multiple drivers on a single shared UART (TMC2209 supports up
///      to 4 chips per bus via the slave address) live behind separate
///      `ITmcUart` instances with different slave addresses — the
///      driver components don't care.
///   3. Keeps the lib_motor headers free of `<ungula/hal/uart/uart.h>`
///      so a host that only uses `Axis` doesn't pull UART in.
///
/// ## Threading
///
/// All methods are TASK-context only. Never call from ISR. The
/// production implementation (`Tmc2209HalUart`) holds the UART for the
/// duration of each call and the response timeout is bounded — but
/// it's still a blocking I/O path that has no business sharing a CPU
/// core with the pulse engine's ISR.
class ITmcUart {
    public:
        virtual ~ITmcUart() = default;

        /// Write 32 bits to register `reg`. Returns `TransportError` on
        /// UART failure, `Ok` on a successful transmit. Note: TMC2209 has
        /// no per-write ACK — the configurator validates writes by reading
        /// the `IFCNT` register (it monotonically increments by 1 per
        /// successful write).
        virtual Status writeRegister(uint8_t reg, uint32_t value) = 0;

        /// Read 32 bits from register `reg`. Returns `TransportError` on
        /// UART timeout / framing error / CRC mismatch.
        virtual Result<uint32_t> readRegister(uint8_t reg) = 0;
};

} // namespace ungula::motor::tmc2209
