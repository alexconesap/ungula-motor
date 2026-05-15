// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h"

namespace
{

using namespace ungula::motor::tmc2209;

/// Reference implementation lifted verbatim from the Trinamic datasheet
/// AN001 §4.2.4. Used here as an independent oracle so refactors of
/// the production CRC can't drift unnoticed.
uint8_t referenceCrc(const uint8_t *data, size_t len)
{
        uint8_t crc = 0;
        for (size_t i = 0; i < len; ++i) {
                uint8_t b = data[i];
                for (uint8_t j = 0; j < 8; ++j) {
                        if ((crc >> 7) ^ (b & 0x01)) {
                                crc = static_cast<uint8_t>(crc << 1) ^ 0x07;
                        } else {
                                crc = static_cast<uint8_t>(crc << 1);
                        }
                        b = static_cast<uint8_t>(b >> 1);
                }
        }
        return crc;
}

TEST(Tmc2209HalUartCrcTest, EmptyDataYieldsZero)
{
        EXPECT_EQ(Tmc2209HalUart::crc(nullptr, 0), 0);
}

TEST(Tmc2209HalUartCrcTest, MatchesReferenceImpl_WriteFrames)
{
        // Sweep a handful of representative write frames (without CRC
        // byte) — GCONF, CHOPCONF, IHOLD_IRUN — and verify the production
        // CRC matches the reference algorithm byte-for-byte.
        const uint8_t frames[][7] = {
                { 0x05, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 }, // write GCONF=0 slave 0
                { 0x05, 0x01, 0xEC, 0x00, 0x01, 0x00, 0xC3 }, // write CHOPCONF
                { 0x05, 0x02, 0x90, 0x00, 0x01, 0x08, 0x10 }, // write IHOLD_IRUN
                { 0x05, 0x03, 0xC0, 0x12, 0x34, 0x56, 0x78 }, // arbitrary
        };
        for (const auto &f : frames) {
                EXPECT_EQ(Tmc2209HalUart::crc(f, sizeof(f)), referenceCrc(f, sizeof(f)));
        }
}

TEST(Tmc2209HalUartCrcTest, MatchesReferenceImpl_ReadRequests)
{
        // Read-request frames: SYNC + slave + reg, then CRC.
        const uint8_t frames[][3] = {
                { 0x05, 0x00, 0x00 }, // read GCONF
                { 0x05, 0x00, 0x6F }, // read DRV_STATUS
                { 0x05, 0x01, 0x41 }, // read SG_RESULT slave 1
                { 0x05, 0x02, 0x12 }, // read TSTEP slave 2
                { 0x05, 0x03, 0x6C }, // read CHOPCONF slave 3
        };
        for (const auto &f : frames) {
                EXPECT_EQ(Tmc2209HalUart::crc(f, sizeof(f)), referenceCrc(f, sizeof(f)));
        }
}

} // namespace
