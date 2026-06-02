// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "ungula/motor/i_device_transport.h"

namespace ungula::motor::tests
{

/// In-memory `IDeviceTransport` modelling a TMC2209 chip's register
/// file. The driver writes/reads TMC datagrams; this fake parses them
/// (validating SYNC + CRC + slave/reg bytes) and updates / returns
/// register values directly.
///
/// Test usage:
///   - Seed registers via `setRegister(addr, value)` before the
///     driver call that will read them (e.g. IOIN for identity,
///     SG_RESULT for diagnostics).
///   - Inspect `lastWriteRegister()` / `lastWriteValue()` after a
///     driver call to verify the driver wrote what we expect.
///   - For sequence-sensitive tests, use `writeLog()` to walk every
///     (reg, value) write in order.
class FakeTmcTransport final : public IDeviceTransport {
    public:
        FakeTmcTransport()
        {
                // Pre-seed IOIN (reg 0x06) with production-silicon
                // version byte 0x21 so the driver's begin() identity
                // check passes by default. Tests modelling a missing
                // / wrong chip clear it via `setRegister(0x06, 0)`
                // or `setRegister(0x06, 0xFF000000)`.
                registers_[0x06] = 0x21000000u;
        }

        Status begin() override
        {
                begun_ = true;
                return Status::Ok();
        }

        Status writeFrame(const uint8_t *data, size_t len) override
        {
                if (!begun_ || data == nullptr) {
                        return Status::Err(ErrorCode::NotInitialized);
                }
                pendingReply_.clear();

                if (len == 8) {
                        // Write frame: SYNC + slave + reg|0x80 + 4 data + CRC.
                        if (data[0] != 0x05) {
                                return Status::Err(ErrorCode::TransportError);
                        }
                        const uint8_t crc = computeCrc(data, 7);
                        if (crc != data[7]) {
                                return Status::Err(ErrorCode::TransportError);
                        }
                        const uint8_t reg = data[2] & 0x7F;
                        const uint32_t value = (static_cast<uint32_t>(data[3]) << 24) |
                                                (static_cast<uint32_t>(data[4]) << 16) |
                                                (static_cast<uint32_t>(data[5]) << 8) |
                                                static_cast<uint32_t>(data[6]);
                        registers_[reg] = value;
                        writeLog_.push_back({ reg, value });
                        return Status::Ok();
                }
                if (len == 4) {
                        // Read request: SYNC + slave + reg + CRC. Prepare
                        // the reply for the next two readFrame() calls
                        // (echo first, then real reply).
                        if (data[0] != 0x05) {
                                return Status::Err(ErrorCode::TransportError);
                        }
                        const uint8_t crc = computeCrc(data, 3);
                        if (crc != data[3]) {
                                return Status::Err(ErrorCode::TransportError);
                        }
                        const uint8_t reg = data[2] & 0x7F;
                        const uint32_t value = registers_.count(reg) ? registers_[reg] : 0u;
                        readLog_.push_back(reg);

                        // Echo of the request first.
                        pendingReply_.insert(pendingReply_.end(), data, data + 4);

                        // Real reply: SYNC + 0xFF + reg + 4 data MSB-first + CRC.
                        std::array<uint8_t, 8> reply{};
                        reply[0] = 0x05;
                        reply[1] = 0xFF;
                        reply[2] = reg;
                        reply[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
                        reply[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
                        reply[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
                        reply[6] = static_cast<uint8_t>(value & 0xFF);
                        reply[7] = computeCrc(reply.data(), 7);
                        pendingReply_.insert(pendingReply_.end(), reply.begin(), reply.end());
                        return Status::Ok();
                }
                return Status::Err(ErrorCode::TransportError);
        }

        Result<size_t> readFrame(uint8_t *dst, size_t maxLen, uint32_t /*timeoutMs*/) override
        {
                if (!begun_ || dst == nullptr) {
                        return Result<size_t>::Err(ErrorCode::NotInitialized);
                }
                if (pendingReply_.empty()) {
                        return Result<size_t>::Err(ErrorCode::Timeout);
                }
                const size_t n = (maxLen < pendingReply_.size()) ? maxLen : pendingReply_.size();
                for (size_t i = 0; i < n; ++i) {
                        dst[i] = pendingReply_[i];
                }
                pendingReply_.erase(pendingReply_.begin(),
                                     pendingReply_.begin() + static_cast<long>(n));
                return Result<size_t>::Ok(n);
        }

        // ---- Test API ---------------------------------------------------
        void setRegister(uint8_t reg, uint32_t value)
        {
                registers_[reg] = value;
        }
        uint32_t getRegister(uint8_t reg) const
        {
                auto it = registers_.find(reg);
                return (it == registers_.end()) ? 0u : it->second;
        }
        struct WriteEntry {
                uint8_t  reg;
                uint32_t value;
        };
        const std::vector<WriteEntry> &writeLog() const { return writeLog_; }
        const std::vector<uint8_t> &readLog() const { return readLog_; }
        void resetLogs()
        {
                writeLog_.clear();
                readLog_.clear();
        }

    private:
        static uint8_t computeCrc(const uint8_t *data, size_t length)
        {
                uint8_t c = 0;
                for (size_t i = 0; i < length; ++i) {
                        uint8_t b = data[i];
                        for (uint8_t j = 0; j < 8; ++j) {
                                const uint8_t mix = (c >> 7) ^ (b & 0x01);
                                c = static_cast<uint8_t>(c << 1);
                                if (mix) {
                                        c ^= 0x07;
                                }
                                b = static_cast<uint8_t>(b >> 1);
                        }
                }
                return c;
        }

        bool                          begun_ = false;
        std::map<uint8_t, uint32_t>   registers_;
        std::vector<WriteEntry>       writeLog_;
        std::vector<uint8_t>     readLog_;
        std::vector<uint8_t>     pendingReply_;
};

} // namespace ungula::motor::tests
