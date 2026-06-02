// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

// Optional driver: requires lib_canbus. Define `UNGULA_USE_CANBUS` at
// build time to compile this driver and pull in the RMD CAN protocol
// headers from lib_canbus. When the macro is not defined the whole TU
// is empty and lib_motor has no dependency on lib_canbus.
#ifdef UNGULA_USE_CANBUS

#include <cstdint>

#include "ungula/hal/can/i_can.h"
#include "ungula/motor/drivers/rmd/rmd_config.h"
#include "ungula/motor/i_motor_driver.h"

namespace ungula::motor::rmd
{

/// `IMotorDriver` for MyActuator / LK Tech RMD servo motors over CAN
/// (V3 protocol). Unlike the STEP/DIR drivers there is NO
/// `IStepSignalGenerator` and NO planner involved — the motor's
/// onboard control loop closes the position / velocity loop. The
/// driver is a thin adapter from axis-domain verbs to the wire
/// protocol implemented in `lib_canbus` (`ungula::canbus::rmd::*`):
///
///   - `armMove`   → `moveAbsolute` (0xA4)
///   - `armJog`    → `sendSpeed`    (0xA2)
///   - `stop`      → `stop` (0x81) for `Decelerate`, `shutdown` (0x80)
///                  for `Immediate`.
///   - `disable`   → `shutdown` (0x80) or `stop` (0x81), depending on
///                  `RmdConfig::releaseBrakeOnDisable`.
///   - `enable`    → `wakeUp` (0x88).
///   - `clearFault`→ `wakeUp` (0x88).
///   - `identity`  → cached at `begin()` from a `readModel` (0x12)
///                  round-trip.
///
/// The driver is appropriate for host-side PID loops: the host runs
/// its own encoder + PID, computes a speed setpoint, and calls
/// `axis.setSpeed(...)` followed by `axis.moveForward()` /
/// `axis.moveBackward()` to update the motor's velocity command.
///
/// Position feedback (`commandedPositionSteps`) tracks what the driver
/// has commanded, NOT the live encoder reading. The host can read the
/// real position via `ungula::canbus::rmd::readPosition` (a separate
/// call - outside the `IMotorDriver` surface).
class RmdCanDriver final : public IMotorDriver {
    public:
        RmdCanDriver(RmdConfig cfg, ungula::hal::can::ICan &bus);

        RmdCanDriver(const RmdCanDriver &) = delete;
        RmdCanDriver &operator=(const RmdCanDriver &) = delete;

        // ---- IMotorDriver -----------------------------------------------
        Status begin() override;
        Status enable() override;
        Status disable() override;
        Status clearFault() override;

        Status armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps, uint32_t accelSps2,
                       uint32_t decelSps2) override;
        Status armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2) override;
        Status stop(StopMode mode) override;

        DriverMotionStatus motionStatus() const override;
        Position commandedPositionSteps() const override;
        uint32_t commandedSpsNow() const override;
        Status resetPosition(Position newSteps) override;

        DriverIdentity identity() override;
        IntentSupport applyIntent(MotorIntent intent) override;
        void fillDriverDiagnostics(MotorDiagnostics &out) const override;

        // ---- Static helpers (exposed for tests / diagnostics) -----------
        /// Wire-frame CAN ID for a given motor address. Convenience for
        /// hosts that want to log / filter on the CAN ID without
        /// re-deriving it. Matches `ungula::canbus::rmd::TX_BASE +
        /// motorId`.
        static constexpr uint32_t canIdFor(uint8_t motorId)
        {
                return 0x140u + static_cast<uint32_t>(motorId);
        }

        /// Convert axis SPS to RMD's 0.01°/s wire unit, given the
        /// configured `stepsPerRevolution`. Returns 0 if SPR is zero.
        static int32_t spsToCentideg(uint32_t sps, uint32_t stepsPerRevolution);

        /// Convert a signed step count to RMD's 0.01° wire unit.
        static int32_t stepsToCentideg(int32_t steps, uint32_t stepsPerRevolution);

    private:
        RmdConfig cfg_;
        ungula::hal::can::ICan &bus_;
        DriverIdentity identity_{ "MyActuator", "RMD", 0, 0, 0 };
        Position commandedPosition_ = 0;
        uint32_t commandedSps_ = 0;
        bool running_ = false;
        bool faulted_ = false;
        StopReason lastFinishedReason_ = StopReason::None;
        bool begun_ = false;
};

} // namespace ungula::motor::rmd

#endif // UNGULA_USE_CANBUS
