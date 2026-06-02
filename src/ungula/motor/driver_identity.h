// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// Vendor / model / firmware identity for a motor driver. Every concrete
/// `IMotorDriver` answers `identity()` — drivers that have a runtime
/// identity register (TMC2209 IOIN[31:24]) read it during `begin()` and
/// cache the value; drivers without one (YPMC, generic STEP/DIR) return
/// compile-time constants. If a driver cannot determine its identity
/// for any reason it still returns this struct with vendor = "Unknown".
///
/// ## String lifetimes
///
/// `vendor` and `model` are `const char*` pointing to static rodata
/// (compile-time string literals owned by the driver source file).
/// Caller may copy them but must NOT free them. They are guaranteed
/// to outlive the `MotorAxis`.
///
/// Firmware version is split into two numeric bytes so drivers don't
/// need runtime buffers for "v1.3" style strings. Drivers whose
/// identity register is a single opaque byte (TMC2209's IOIN[31:24])
/// put that byte in `firmwareMajor` and leave `firmwareMinor` at 0.
/// `rawId` carries the full raw identity register for diagnostics.
struct DriverIdentity {
        const char *vendor = "Unknown";
        const char *model = "Unknown";
        uint8_t firmwareMajor = 0;
        uint8_t firmwareMinor = 0;
        uint32_t rawId = 0;
};

} // namespace ungula::motor
