// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/drivers/generic_stepdir/generic_stepdir_driver.h"
#include "ungula/motor/drivers/ypmc/ypmc_config.h"

namespace ungula::motor::ypmc
{

/// RATTMOTOR YPMC + S2SVD15 STEP/DIR servo driver. Specialises the
/// generic STEP/DIR driver to add:
///
///   - Hardcoded identity (`vendor = "RATTMOTOR"`, `model = "YPMC + S2SVD15"`).
///   - Optional tandem secondary DIR pin (writes before delegating to
///     the step signal generator's `armMove` so both DIRs are stable
///     during `dirSetupUs`).
///   - Optional holding-brake control: releases on motion start,
///     engages on motion end (and always on faults / emergency stop).
///
/// Everything else (planning, pulse generation, EN pin) inherits from
/// `GenericStepDirDriver`. ALM / COIN inputs are NOT handled here —
/// they belong on the host's `MotorAxisConfig::limits_wiring[]` table
/// (ALM as `LimitKind::EmergencyLimit`).
///
/// `MotorIntent::*` for this drive: every intent reports
/// `IntentSupport::Unsupported` except `Default` — the drive's chopper
/// configuration is fixed in silicon, the host can't change it from
/// firmware. This is honest reporting, not a bug.
class YpmcStepDirDriver final : public stepdir::GenericStepDirDriver {
    public:
        /// Self-owns constructor. The driver allocates its own
        /// `RmtStepSignal` (one RMT TX channel) and
        /// `TrapezoidalPlanner` internally. STEP / DIR / EN pins come
        /// from `cfg.generic`; secondary-DIR + brake pins come from
        /// the YPMC-specific fields.
        ///
        /// Tandem use case (Wendy catheter coil winder): two YPMC +
        /// S2SVD15 drives mounted face-to-face on the same shaft,
        /// turning at the same RPM in opposite electrical directions
        /// so they pull on a mandrel from both sides. The two drives
        /// share one STEP pin (driven by this RMT channel) and have
        /// independent DIR pins. Set `secondaryDirPin` to the second
        /// DIR pin and `secondaryDirInverted = true` for that mount.
        explicit YpmcStepDirDriver(YpmcConfig cfg);

        /// Pluggable constructor: host provides a pre-initialised
        /// step signal generator + planner. Same trade-offs as
        /// `GenericStepDirDriver`'s pluggable constructor.
        YpmcStepDirDriver(YpmcConfig cfg, IStepSignalGenerator &stepSignal,
                          IMotionPlanner &planner);

        YpmcStepDirDriver(const YpmcStepDirDriver &) = delete;
        YpmcStepDirDriver &operator=(const YpmcStepDirDriver &) = delete;

        // ---- Overrides from GenericStepDirDriver ------------------------
        Status begin() override;
        Status enable() override;
        Status disable() override;
        Status stop(StopMode mode) override;

        DriverIdentity identity() override;

    protected:
        void onBeforeArm(Direction dir) override;
        void onMotionStarted() override;
        void onMotionStopped() override;

    private:
        void writeSecondaryDir(Direction dir);
        void writeBrake(bool released);
        void engageBrake();
        void releaseBrake();

        YpmcConfig ypmcCfg_;
        bool brakeReleased_ = false;
};

} // namespace ungula::motor::ypmc
