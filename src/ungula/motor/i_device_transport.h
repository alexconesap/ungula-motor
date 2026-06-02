// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstddef>
#include <cstdint>

#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Bus-agnostic transport. Wraps UART / CAN / SPI / RS485 / etc. behind
/// a single frame-oriented interface so chip drivers depend on this
/// instead of `lib_hal::Uart` / `lib_hal::Can`. That makes drivers
/// testable with a `FakeDeviceTransport` and keeps `lib_motor` decoupled
/// from the HAL header set.
///
/// Concrete implementations are paired with the chip driver they serve
/// (e.g. `Tmc2209UartTransport` wraps `lib_hal::Uart` with TMC's
/// SYNC/ADDR/CRC framing). Hosts construct the transport once and pass
/// references to one or more drivers — letting two `Tmc2209Driver`
/// instances share one transport on the same UART.
class IDeviceTransport {
    public:
        virtual ~IDeviceTransport() = default;

        /// Initialise the underlying hardware. Idempotent: calling twice
        /// on the same transport returns OK without re-initialising.
        virtual Status begin() = 0;

        /// Write a complete frame. Returns OK on full write; partial
        /// writes are not exposed — the transport buffers internally or
        /// returns TransportError.
        virtual Status writeFrame(const uint8_t *data, size_t len) = 0;

        /// Read a complete frame up to `maxLen` bytes. `timeoutMs` is
        /// the deadline; 0 means "non-blocking, return what's available
        /// right now". Returns the number of bytes actually read on
        /// success.
        virtual Result<size_t> readFrame(uint8_t *dst, size_t maxLen, uint32_t timeoutMs) = 0;
};

} // namespace ungula::motor
