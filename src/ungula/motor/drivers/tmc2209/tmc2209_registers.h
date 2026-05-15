// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor::tmc2209
{

/// TMC2209 register map. Addresses match Trinamic's datasheet §5
/// (rev. 1.09). Only the subset the motor library touches is listed —
/// host applications that need additional registers can pass raw
/// addresses to `ITmcUart::readRegister` / `writeRegister`.
namespace reg
{
        inline constexpr uint8_t GCONF = 0x00; ///< Global config
        inline constexpr uint8_t GSTAT = 0x01; ///< Global status (W1C)
        inline constexpr uint8_t IFCNT = 0x02; ///< UART write counter (RO)
        inline constexpr uint8_t SLAVECONF = 0x03; ///< Reply delay
        inline constexpr uint8_t OTP_PROG = 0x04;
        inline constexpr uint8_t OTP_READ = 0x05;
        inline constexpr uint8_t IOIN = 0x06; ///< Input pin levels (RO)
        inline constexpr uint8_t FACTORY_CONF = 0x07;

        inline constexpr uint8_t IHOLD_IRUN = 0x10; ///< Hold + run current
        inline constexpr uint8_t TPOWERDOWN = 0x11;
        inline constexpr uint8_t TSTEP = 0x12; ///< Step interval (RO)
        inline constexpr uint8_t TPWMTHRS = 0x13;
        inline constexpr uint8_t TCOOLTHRS = 0x14; ///< Lower velocity threshold for SG
        inline constexpr uint8_t VACTUAL = 0x22; ///< UART-driven velocity (RW)

        inline constexpr uint8_t SGTHRS = 0x40; ///< StallGuard threshold
        inline constexpr uint8_t SG_RESULT = 0x41; ///< StallGuard measurement (RO)
        inline constexpr uint8_t COOLCONF = 0x42;

        inline constexpr uint8_t MSCNT = 0x6A; ///< Microstep counter (RO)
        inline constexpr uint8_t MSCURACT = 0x6B;
        inline constexpr uint8_t CHOPCONF = 0x6C; ///< Chopper config (microsteps live here)
        inline constexpr uint8_t DRV_STATUS = 0x6F; ///< Driver status (RO)
        inline constexpr uint8_t PWMCONF = 0x70; ///< StealthChop PWM config
} // namespace reg

/// GCONF bit masks. Only fields the configurator touches are named.
namespace gconf
{
        inline constexpr uint32_t I_SCALE_ANALOG = 1u << 0;
        inline constexpr uint32_t INTERNAL_RSENSE = 1u << 1;
        inline constexpr uint32_t EN_SPREADCYCLE = 1u << 2;
        inline constexpr uint32_t SHAFT = 1u << 3;
        inline constexpr uint32_t INDEX_OTPW = 1u << 4;
        inline constexpr uint32_t INDEX_STEP = 1u << 5;
        inline constexpr uint32_t PDN_DISABLE = 1u << 6;
        inline constexpr uint32_t MSTEP_REG_SELECT = 1u << 7;
} // namespace gconf

/// GSTAT W1C bits — write 1 to clear. Reading GSTAT does NOT clear
/// it; the bug in the original driver was `clearGstat` doing a read.
namespace gstat
{
        inline constexpr uint32_t RESET = 1u << 0;
        inline constexpr uint32_t DRV_ERR = 1u << 1;
        inline constexpr uint32_t UV_CP = 1u << 2;
        inline constexpr uint32_t ALL_FLAGS = RESET | DRV_ERR | UV_CP;
} // namespace gstat

/// DRV_STATUS read-only bits surfaced by Diagnostics.
namespace drv_status
{
        inline constexpr uint32_t OTPW = 1u << 0; // overtemperature pre-warning
        inline constexpr uint32_t OT = 1u << 1; // overtemperature
        inline constexpr uint32_t S2GA = 1u << 2; // short to GND, phase A
        inline constexpr uint32_t S2GB = 1u << 3; // short to GND, phase B
        inline constexpr uint32_t S2VSA = 1u << 4;
        inline constexpr uint32_t S2VSB = 1u << 5;
        inline constexpr uint32_t OLA = 1u << 6; // open-load phase A
        inline constexpr uint32_t OLB = 1u << 7;
        inline constexpr uint32_t T120 = 1u << 8;
        inline constexpr uint32_t T143 = 1u << 9;
        inline constexpr uint32_t T150 = 1u << 10;
        inline constexpr uint32_t T157 = 1u << 11;
        inline constexpr uint32_t STEALTH = 1u << 30;
        inline constexpr uint32_t STST = 1u << 31; // standstill
        /// Composite: any condition the host should treat as a fault.
        inline constexpr uint32_t FAULT_MASK = OT | S2GA | S2GB | S2VSA | S2VSB;
} // namespace drv_status

/// CHOPCONF bit positions for the fields the configurator writes.
namespace chopconf
{
        inline constexpr uint32_t VSENSE = 1u << 17; // Vfs scaling
        inline constexpr uint8_t MRES_SHIFT = 24; // 4-bit field
        inline constexpr uint32_t MRES_MASK = 0xFu << MRES_SHIFT;
        inline constexpr uint32_t INTPOL = 1u << 28;
        inline constexpr uint32_t DEDGE = 1u << 29;
} // namespace chopconf

/// COOLCONF bit positions / shifts for the CoolStep configurator.
namespace coolconf
{
        inline constexpr uint8_t SEMIN_SHIFT = 0; // 4-bit field
        inline constexpr uint32_t SEMIN_MASK = 0xFu << SEMIN_SHIFT;
        inline constexpr uint8_t SEUP_SHIFT = 5; // 2-bit field
        inline constexpr uint32_t SEUP_MASK = 0x3u << SEUP_SHIFT;
        inline constexpr uint8_t SEMAX_SHIFT = 8; // 4-bit field
        inline constexpr uint32_t SEMAX_MASK = 0xFu << SEMAX_SHIFT;
        inline constexpr uint8_t SEDN_SHIFT = 13; // 2-bit field
        inline constexpr uint32_t SEDN_MASK = 0x3u << SEDN_SHIFT;
        inline constexpr uint32_t SEIMIN = 1u << 15; // current floor select
} // namespace coolconf

} // namespace ungula::motor::tmc2209
