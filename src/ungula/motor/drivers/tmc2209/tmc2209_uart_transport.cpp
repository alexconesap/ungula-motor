// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_uart_transport.h"

#include "ungula/hal/uart/uart.h"

namespace ungula::motor::tmc2209
{

Tmc2209UartTransport::Tmc2209UartTransport(ungula::hal::uart::Uart &uart)
        : uart_(uart)
{
}

Status Tmc2209UartTransport::begin()
{
        // Host owns UART configuration (baud / pins). Repeated begin()
        // is treated as idempotent so multiple drivers can share one
        // transport safely.
        begun_ = true;
        return Status::Ok();
}

void Tmc2209UartTransport::setDefaultTimeoutMs(uint32_t ms)
{
        if (ms > 0) {
                defaultTimeoutMs_ = ms;
        }
}

Status Tmc2209UartTransport::writeFrame(const uint8_t *data, size_t len)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (data == nullptr || len == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Half-duplex bus: drain any stale bytes (notably the chip's
        // half-duplex echo from a prior exchange) before transmitting.
        uart_.flushInput();

        const auto written = uart_.write(data, len);
        if (written != static_cast<int32_t>(len)) {
                return Status::Err(ErrorCode::TransportError);
        }
        uart_.flush();
        return Status::Ok();
}

Result<size_t> Tmc2209UartTransport::readFrame(uint8_t *dst, size_t maxLen, uint32_t timeoutMs)
{
        if (!begun_) {
                return Result<size_t>::Err(ErrorCode::NotInitialized);
        }
        if (dst == nullptr || maxLen == 0) {
                return Result<size_t>::Err(ErrorCode::InvalidConfig);
        }
        const uint32_t deadline = (timeoutMs > 0) ? timeoutMs : defaultTimeoutMs_;
        const auto got = uart_.read(dst, maxLen, deadline);
        if (got < 0) {
                return Result<size_t>::Err(ErrorCode::TransportError);
        }
        return Result<size_t>::Ok(static_cast<size_t>(got));
}

} // namespace ungula::motor::tmc2209
