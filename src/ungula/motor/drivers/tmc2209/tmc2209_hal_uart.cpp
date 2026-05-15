// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h"

#include <cstddef>
#include <cstdint>

#include "ungula/hal/uart/uart.h"

namespace ungula::motor::tmc2209
{

namespace
{
        constexpr uint8_t kSync = 0x05; // TMC datagram start-of-frame
        constexpr uint8_t kWriteFlag = 0x80; // OR'd into register address
        constexpr uint8_t kMaxSlaveAddr = 3; // NAI[1:0]: 4 chips per bus

        constexpr size_t kWriteFrameSize = 8; // SYNC + slave + reg + 4 data + CRC
        constexpr size_t kReadReqSize = 4; // SYNC + slave + reg + CRC
        constexpr size_t kReadReplySize = 8; // SYNC + 0xFF + reg + 4 data + CRC
} // namespace

uint8_t Tmc2209HalUart::crc(const uint8_t *data, size_t length)
{
        // TMC datagram CRC8: polynomial 0x07, init 0, no reflection, no
        // final XOR — bit-by-bit per Trinamic AN001. Inlined here because
        // it's tiny and we want to keep the driver dependency-free.
        uint8_t c = 0;
        for (size_t i = 0; i < length; ++i) {
                uint8_t b = data[i];
                for (uint8_t j = 0; j < 8; ++j) {
                        const uint8_t mix = (c >> 7) ^ (b & 0x01);
                        c = static_cast<uint8_t>(c << 1);
                        if (mix)
                                c ^= 0x07;
                        b = static_cast<uint8_t>(b >> 1);
                }
        }
        return c;
}

Tmc2209HalUart::Tmc2209HalUart(ungula::hal::uart::Uart &uart, uint8_t slaveAddress)
        : uart_(uart)
        , slaveAddress_(slaveAddress)
{
}

void Tmc2209HalUart::setResponseTimeoutMs(uint32_t ms)
{
        if (ms > 0)
                responseTimeoutMs_ = ms;
}

Status Tmc2209HalUart::writeRegister(uint8_t reg, uint32_t value)
{
        if (slaveAddress_ > kMaxSlaveAddr) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        uint8_t frame[kWriteFrameSize];
        frame[0] = kSync;
        frame[1] = slaveAddress_;
        frame[2] = static_cast<uint8_t>(reg | kWriteFlag);
        // TMC datasheet: data sent MSB-first within the 32-bit field.
        frame[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        frame[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
        frame[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
        frame[6] = static_cast<uint8_t>(value & 0xFF);
        frame[7] = crc(frame, 7);

        // Half-duplex bus: the chip's TX driver re-asserts our own bytes
        // back into our RX line. Drain anything left over from the previous
        // exchange so a subsequent read doesn't lock onto stale data.
        uart_.flushInput();

        const auto written = uart_.write(frame, sizeof(frame));
        if (written != static_cast<int32_t>(sizeof(frame))) {
                return Status::Err(ErrorCode::TransportError);
        }
        uart_.flush();
        return Status::Ok();
}

Result<uint32_t> Tmc2209HalUart::readRegister(uint8_t reg)
{
        if (slaveAddress_ > kMaxSlaveAddr) {
                return Result<uint32_t>::Err(ErrorCode::InvalidConfig);
        }

        uint8_t req[kReadReqSize];
        req[0] = kSync;
        req[1] = slaveAddress_;
        req[2] = static_cast<uint8_t>(reg & 0x7F); // R/W bit clear = read
        req[3] = crc(req, 3);

        uart_.flushInput();

        const auto written = uart_.write(req, sizeof(req));
        if (written != static_cast<int32_t>(sizeof(req))) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }
        uart_.flush();

        // Half-duplex echo of the request appears first on the RX line;
        // skip past it. Then read the 8-byte reply.
        uint8_t echo[kReadReqSize];
        uart_.read(echo, sizeof(echo), responseTimeoutMs_);

        uint8_t reply[kReadReplySize];
        const auto got = uart_.read(reply, sizeof(reply), responseTimeoutMs_);
        if (got != static_cast<int32_t>(sizeof(reply))) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }

        // Frame structure: 0x05, 0xFF (master addr), reg, 4 data MSB-first, CRC.
        if (reply[0] != kSync || reply[2] != static_cast<uint8_t>(reg & 0x7F)) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }
        if (crc(reply, kReadReplySize - 1) != reply[kReadReplySize - 1]) {
                return Result<uint32_t>::Err(ErrorCode::TransportError);
        }

        const uint32_t value =
            (static_cast<uint32_t>(reply[3]) << 24) | (static_cast<uint32_t>(reply[4]) << 16) |
            (static_cast<uint32_t>(reply[5]) << 8) | static_cast<uint32_t>(reply[6]);
        return Result<uint32_t>::Ok(value);
}

} // namespace ungula::motor::tmc2209
