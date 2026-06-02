// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstddef>
#include <cstdint>

#include "ungula/motor/i_device_transport.h"

namespace ungula::hal::uart
{
class Uart;
}

namespace ungula::motor::tmc2209
{

/// `IDeviceTransport` over a `lib_hal::Uart`. Byte-level pipe only — no
/// TMC-specific framing here. Each `Tmc2209Driver` formats its own
/// frames (SYNC + slave + reg + payload + CRC) and writes them through
/// `writeFrame`; reads pull the half-duplex echo + reply via two
/// `readFrame` calls.
///
/// Why a transport class at all (rather than the driver taking a
/// `lib_hal::Uart&` directly): two `Tmc2209Driver` instances on one
/// physical bus share ONE transport. Each driver has its own slave
/// address in its config and uses the same transport to read/write.
/// The transport is bus-level; the driver is chip-level.
///
/// Tests substitute a `FakeDeviceTransport` (no UART hardware
/// required) to assert the exact bytes the driver writes.
class Tmc2209UartTransport final : public IDeviceTransport {
    public:
        explicit Tmc2209UartTransport(ungula::hal::uart::Uart &uart);

        Tmc2209UartTransport(const Tmc2209UartTransport &) = delete;
        Tmc2209UartTransport &operator=(const Tmc2209UartTransport &) = delete;

        Status begin() override;
        Status writeFrame(const uint8_t *data, size_t len) override;
        Result<size_t> readFrame(uint8_t *dst, size_t maxLen, uint32_t timeoutMs) override;

        /// Tunable for hosts running with a long `SLAVECONF` reply delay.
        /// Defaults to 50 ms which covers any sensible config at ≥ 9600
        /// baud. The driver passes this to `readFrame()` as the timeout.
        void setDefaultTimeoutMs(uint32_t ms);

        uint32_t defaultTimeoutMs() const
        {
                return defaultTimeoutMs_;
        }

    private:
        ungula::hal::uart::Uart &uart_;
        uint32_t defaultTimeoutMs_ = 50;
        bool begun_ = false;
};

} // namespace ungula::motor::tmc2209
