// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/drivers/tmc2209/i_tmc_uart.h"
#include "ungula/motor/result.h"

namespace ungula::motor::tmc2209
{

/// TMC2209 StallGuard4 configuration + read paths. Strictly off the
/// motion-timing path — the actual stall trigger reaches the host via
/// the DIAG pin, which is wired as a GPIO interrupt at the Axis layer.
///
/// ## Two detection modes
///
///   1. **DIAG-pin (default)**: the chip drives DIAG high when
///      SG_RESULT drops below 2*SGTHRS during a motion above TCOOLTHRS.
///      The host wires DIAG to a GPIO and the Axis facade registers a
///      sensor with role `SensorRole::Stall` against it (NOT
///      `CrashLimit` — the bank has dedicated stall debouncing via
///      `stallHitsToTrigger` / `stallArmDelayMs`, and stall events
///      route to `FaultCode::Stall` / `StopReason::StallDetected`
///      instead of the generic limit-switch path). Sub-microsecond
///      latency. No UART traffic during motion.
///   2. **SG_RESULT polling (opt-in)**: the host periodically reads
///      SG_RESULT via UART and applies its own threshold logic. Only
///      used when the host needs the raw measurement (e.g. for stall
///      tuning, datalogging). MUST be polled from a low-priority task
///      because each read takes ~1.5 ms at 115200 baud.
///
/// The rule from the refactor prompt:
///
///   > During motion, default stall detection should be DIAG-pin based
///   > if configured. SG_RESULT polling during motion must be optional.
///   > If SG_RESULT polling is enabled, it must use cached async
///   > diagnostics, not blocking direct UART reads from the axis
///   > service path.
///
/// The latter constraint is satisfied by routing `readSgResult()`
/// through this class and forbidding the Axis service path from
/// calling it. Hosts wanting polling should run it from their main
/// loop or a dedicated diagnostics task.
class Tmc2209StallGuard {
    public:
        struct Config {
                /// SGTHRS — 0..255. The chip declares a stall when
                /// SG_RESULT < 2 * SGTHRS. Higher = more sensitive (stalls
                /// earlier); calibrated per motor + mechanics, not a one-size-
                /// fits-all value.
                uint8_t sgThreshold = 10;
                /// TCOOLTHRS — TSTEP threshold below which CoolStep + StallGuard
                /// become active. Stored as a 20-bit value in the register;
                /// values above 0xFFFFF are clamped. 0 disables StallGuard.
                uint32_t tCoolThrs = 0xFFFFF;
                /// If true, route the stall event onto the chip's DIAG pin.
                /// Wires through GCONF's INDEX_OTPW bit interaction — the
                /// real DIAG signal is its own pad and is on by default; this
                /// flag exists for future expansion (e.g. silencing DIAG when
                /// stall detection is disabled at runtime).
                bool enableDiagOutput = true;

                /// When true (default), `begin()` reads GCONF and refuses
                /// to configure StallGuard if the chip is in SpreadCycle
                /// mode — SG4 is StealthChop-only on the TMC2209, and a
                /// silent "stall never fires" misconfiguration is much
                /// harder to diagnose later than an explicit error at
                /// boot. Set to false only if you have a deliberate
                /// reason to write SGTHRS / TCOOLTHRS while in
                /// SpreadCycle (e.g. preparing the registers before
                /// switching chopper mode).
                bool verifyChopperMode = true;
        };

        explicit Tmc2209StallGuard(ITmcUart &uart);

        Tmc2209StallGuard(const Tmc2209StallGuard &) = delete;
        Tmc2209StallGuard &operator=(const Tmc2209StallGuard &) = delete;

        /// Apply the full StallGuard configuration. Writes SGTHRS and
        /// TCOOLTHRS. The DIAG signal itself is hardware-driven by the
        /// chip; no UART is needed to enable it (the chip has it on
        /// whenever SGTHRS / TCOOLTHRS are configured).
        Status begin(const Config &cfg);

        /// Update the SGTHRS register. Safe to call from any task at any
        /// time — the chip latches it on the next SG comparison.
        Status setSgThreshold(uint8_t threshold);

        /// Update TCOOLTHRS. Out-of-range values are clamped to 20-bit
        /// maximum (0xFFFFF) — the wire register is only that wide and
        /// silently truncating in the silicon would be confusing.
        Status setTCoolThrs(uint32_t value);

        /// One-shot read of SG_RESULT. NOT for the motion service path —
        /// this is a blocking UART read. Use it from diagnostics tasks or
        /// from one-off tuning code.
        ///
        /// SG_RESULT is a 10-bit value (0..1023); the chip places it in
        /// bits [9:0] of the register. Higher = less loaded; the chip
        /// triggers a stall on SG_RESULT < 2*SGTHRS.
        Result<uint16_t> readSgResult();

        // ---- Cached state ------------------------------------------------

        uint8_t shadowSgThreshold() const
        {
                return shadowSgThreshold_;
        }
        uint32_t shadowTCoolThrs() const
        {
                return shadowTCoolThrs_;
        }

    private:
        ITmcUart &uart_;
        uint8_t shadowSgThreshold_ = 0;
        uint32_t shadowTCoolThrs_ = 0;
};

} // namespace ungula::motor::tmc2209
