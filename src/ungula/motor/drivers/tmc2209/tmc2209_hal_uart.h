// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstddef>
#include <cstdint>

#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"

namespace ungula::hal::uart
{
class Uart;
}

namespace ungula::motor::tmc2209
{

/// Production `ITmcUart` over a `ungula::hal::uart::Uart` transport.
///
/// Implements the TMC2209 wire framing (SYNC byte 0x05, slave address,
/// register address with R/W bit, payload, CRC8 with polynomial 0x07).
/// The driver doesn't ACK writes; verification is via the `IFCNT`
/// counter — left to the calling Configurator, which reads it once
/// after `begin()` to detect cable-not-connected scenarios.
///
/// ## Bus topology
///
/// Up to 4 TMC2209 chips can share one UART line, distinguished by
/// `slaveAddress` (0..3). Each chip's NAI pins set its address. One
/// `Tmc2209HalUart` per chip; multiple instances may share the same
/// `Uart&` — there's no internal locking, so the host must
/// serialise calls if motion is happening on more than one axis at
/// once. (The bus is half-duplex and a read takes ~1 ms; running
/// concurrent reads against different chips would corrupt both.)
///
/// ## Timing
///
/// The TMC2209 datasheet specifies an 8-bit-time reply delay
/// configurable via the `SLAVECONF` register. At 115200 baud one bit
/// is ~8.7 µs, so an 8-bit delay = ~70 µs. We wait
/// `responseTimeoutMs` for the 8-byte reply; the default 50 ms is
/// generous (covers chips configured for the max 15-bit reply delay
/// at any reasonable baud).
class Tmc2209HalUart final : public ITmcUart {
    public:
        /// `uart` must already be `begin()`-d by the host with the right
        /// baud / pins. The driver does NOT call `uart.begin()` — the host
        /// owns the UART configuration so it can share the bus with non-
        /// TMC2209 traffic (rare but supported).
        ///
        /// `slaveAddress` is 0..3, matching the NAI[1:0] pin configuration
        /// of the target chip. Out-of-range values are rejected at first
        /// register access.
        Tmc2209HalUart(ungula::hal::uart::Uart &uart, uint8_t slaveAddress);

        Tmc2209HalUart(const Tmc2209HalUart &) = delete;
        Tmc2209HalUart &operator=(const Tmc2209HalUart &) = delete;

        Status writeRegister(uint8_t reg, uint32_t value) override;
        Result<uint32_t> readRegister(uint8_t reg) override;

        /// Tunable in case the host configured a long `SLAVECONF` reply
        /// delay. Defaults to 50 ms which covers anything sensible at
        /// ≥ 9600 baud. Set 0 to use the default.
        void setResponseTimeoutMs(uint32_t ms);

        /// CRC8 with polynomial 0x07, init 0, lsb-first — the TMC datasheet
        /// algorithm. Exposed for FakeTmcUart and tests to validate framing.
        static uint8_t crc(const uint8_t *data, size_t length);

    private:
        ungula::hal::uart::Uart &uart_;
        uint8_t slaveAddress_ = 0;
        uint32_t responseTimeoutMs_ = 50;
};

} // namespace ungula::motor::tmc2209
