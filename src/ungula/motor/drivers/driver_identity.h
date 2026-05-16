// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Vendor / model / firmware identity for a motor driver. Filled by
/// the per-driver `IDriverIdentityProvider` and surfaced via the
/// `IAxisActuator::readDriverIdentity()` / `Axis::readDriverIdentity()`
/// chain.
///
/// Drives with no runtime identity register can still return `Ok`
/// identity via `StaticDriverIdentity` when the host/kit knows what
/// is wired at compile time (for example YPMC + S2SVD15 over plain
/// STEP/DIR). `ErrorCode::Unsupported` is reserved for compose-by-hand
/// setups where the host did not wire any provider at all. There is no
/// `bool available` field inside the struct on purpose (that would
/// invite hosts to check `Ok` and then forget the inner bool).
///
/// ## String lifetimes
///
/// `vendor` and `model` are `const char*` pointing to static rodata
/// (compile-time string literals owned by the driver source file).
/// Caller may copy them but must NOT free them. They are guaranteed
/// to outlive the `Axis`.
///
/// Firmware version is split into two numeric bytes so drivers don't
/// need runtime buffers for "v1.3" style strings. Drivers whose
/// identity register is a single opaque byte (TMC2209's IOIN[31:24])
/// put that byte in `firmwareMajor` and leave `firmwareMinor` at 0.
/// `rawId` carries the full raw identity register for diagnostics.
struct DriverIdentity {
        const char *vendor = nullptr;
        const char *model = nullptr;
        uint8_t firmwareMajor = 0;
        uint8_t firmwareMinor = 0;
        uint32_t rawId = 0;
};

/// Per-driver identity-reading contract. The concrete driver
/// (configurator, transport adapter, CAN protocol object) implements
/// this and gets wired through the actuator's config so the universal
/// `Axis::readDriverIdentity()` works without the actuator knowing
/// anything chip-specific.
///
/// Lifetime: the provider must outlive the actuator that holds a
/// pointer to it. In the kit pattern this is automatic — the kit
/// owns both in `unique_ptr` members with the right destruction
/// order. Compose-by-hand hosts must arrange the lifetime themselves.
class IDriverIdentityProvider {
    public:
        virtual ~IDriverIdentityProvider() = default;

        /// Read identity from the underlying hardware. Returns:
        ///   - `Ok(DriverIdentity{...})` on success.
        ///   - `Err(ErrorCode::Unsupported)` when no provider is wired
        ///     for this axis (compose-by-hand omission).
        ///   - `Err(ErrorCode::TransportError)` if a UART / CAN read
        ///     failed. The chip is configured, the protocol is wrong,
        ///     or the slave address is mismatched.
        ///
        /// May block on UART traffic. NOT for the motion-timing path
        /// — call from setup() or a diagnostics task.
        ///
        /// Same method name as `IAxisActuator::readDriverIdentity()` /
        /// `Axis::readDriverIdentity()` on purpose — one name across
        /// every layer so callers never have to ask "which method?".
        virtual Result<DriverIdentity> readDriverIdentity() = 0;
};

/// `IDriverIdentityProvider` for drives whose identity is fully known
/// at compile time — no register read, no transport traffic. Built for
/// chips/drives that expose no identity register but where the host
/// (or the kit) DOES know what's wired (e.g. RATTMOTOR YPMC + S2SVD15
/// over plain STEP/DIR). Returns the same `DriverIdentity` on every
/// call, never fails.
///
/// Strings passed in MUST have static lifetime (compile-time literals
/// or rodata). The class stores the pointers as-is; it does NOT copy.
class StaticDriverIdentity final : public IDriverIdentityProvider {
    public:
        constexpr StaticDriverIdentity(const char *vendor, const char *model,
                                       uint8_t firmwareMajor = 0,
                                       uint8_t firmwareMinor = 0,
                                       uint32_t rawId = 0)
                : id_{ vendor, model, firmwareMajor, firmwareMinor, rawId }
        {
        }

        Result<DriverIdentity> readDriverIdentity() override
        {
                return Result<DriverIdentity>::Ok(id_);
        }

    private:
        DriverIdentity id_;
};

} // namespace ungula::motor
