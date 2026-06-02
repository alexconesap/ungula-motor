// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor::rmd
{

/// Host-facing RMD configuration. Every host-decidable field — none
/// of the chip-protocol details (CAN frame layout, command bytes,
/// speed-unit conversion) leaks out.
///
/// The host MUST set `stepsPerRevolution` here to the SAME value used
/// in `MotorAxisConfig::units.stepsPerRevolution`. The driver uses it
/// internally to convert axis-domain SPS / step counts to the RMD
/// protocol's 0.01° / 0.01°·s⁻¹ wire units. Mismatch between the two
/// just shifts the actual physical speed by the ratio — the lib has
/// no way to cross-check.
struct RmdConfig {
        /// Motor address (1..32). Wire CAN ID = `0x140 + motorId`. Set
        /// this from the motor's front-panel address dip switches.
        uint8_t motorId = 1;

        /// Steps per output-shaft revolution. Used internally to
        /// convert axis-domain SPS into RMD 0.01°·s⁻¹. Must match the
        /// axis config's `units.stepsPerRevolution`.
        uint32_t stepsPerRevolution = 0;

        /// Per-command CAN TX timeout. The RMD bus is fast (1 Mbps
        /// typical) so 50 ms is generous — keeps the host from
        /// blocking forever on a wiring fault.
        uint32_t commandTimeoutMs = 50;

        /// If true, `disable()` sends a "motor shutdown" (0x80) frame
        /// that also releases the brake on RMD-X servos that have one
        /// — the rotor will free-wheel. If false, sends "motor stop"
        /// (0x81) instead, which keeps the brake engaged but the
        /// motor remains powered (slightly hotter idle, but won't
        /// drop a load on a vertical axis).
        bool releaseBrakeOnDisable = false;
};

} // namespace ungula::motor::rmd
