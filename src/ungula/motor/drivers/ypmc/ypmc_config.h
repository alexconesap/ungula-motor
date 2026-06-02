// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/drivers/generic_stepdir/generic_stepdir_config.h"
#include "ungula/motor/motor_units.h" // GPIO_NONE

namespace ungula::motor::ypmc
{

/// Configuration for a RATTMOTOR YPMC + S2SVD15 STEP/DIR servo drive.
/// Extends the generic STEP/DIR config with three optional inputs the
/// S2SVD15 exposes:
///
///   - **Secondary DIR pin** for tandem-wired pairs (Wendy-Tensy: two
///     YPMC drives sharing one STEP pin, each with its own DIR). The
///     driver writes the secondary DIR BEFORE handing the move to the
///     step signal generator so both DIR levels are stable during the
///     `dirSetupUs` window. `secondaryDirInverted = true` handles the
///     face-to-face mounting case (drive B sees the opposite
///     electrical polarity to produce the same physical rotation as
///     drive A).
///
///   - **Brake pin** for a 24 V holding brake relay. The driver
///     releases the brake before motion starts and engages it after
///     motion completes / faults. `brakeReleaseSettleMs` covers the
///     mechanical release time; `brakeEngageSettleMs` covers
///     mechanical engage. Zero on either disables the wait.
///
///   The YPMC's SRV-ON, ALM, COIN signals are NOT modeled here as
///   driver-internal — those are limit / sensor inputs and belong on
///   the host's `MotorAxisConfig::limits_wiring[]` table (ALM as
///   `EmergencyLimit`, COIN as a polled status read host-side).
struct YpmcConfig {
        stepdir::GenericStepDirConfig generic;

        /// Tandem: a second DIR pin driven in parallel with the
        /// primary DIR (which is owned by the step signal generator).
        /// `GPIO_NONE` (default) = single-motor setup.
        uint8_t secondaryDirPin = GPIO_NONE;
        bool secondaryDirActiveHigh = true;
        /// True for face-to-face mounting: drive B's DIR level is the
        /// OPPOSITE of drive A's per host-commanded direction.
        bool secondaryDirInverted = false;

        /// Brake release relay pin. `GPIO_NONE` = no brake managed by
        /// this driver. The relay is wired so that energising it
        /// (driving the pin to the active level) RELEASES the brake;
        /// de-energising engages it.
        uint8_t brakePin = GPIO_NONE;
        bool brakeReleaseActiveHigh = true;
        /// Milliseconds the driver waits after releasing the brake
        /// before issuing motion (lets the mechanical brake clear).
        uint16_t brakeReleaseSettleMs = 50;
        /// Milliseconds the driver waits after motion ends before
        /// engaging the brake.
        uint16_t brakeEngageSettleMs = 50;
        /// Auto-engage the brake on motion-end events. Set false if
        /// the host wants to manage brake timing explicitly. Faults
        /// always force the brake to engage regardless.
        bool autoEngageOnMotionEnd = true;
};

} // namespace ungula::motor::ypmc
