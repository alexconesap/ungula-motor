// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/rmd/rmd_can_driver.h"

#ifdef UNGULA_USE_CANBUS

#include "ungula/canbus/devices/rmd/motor.h"
#include "ungula/canbus/devices/rmd/protocol.h"

namespace canbus_rmd = ungula::canbus::rmd;

namespace ungula::motor::rmd
{

// =====================================================================
// Construction
// =====================================================================

RmdCanDriver::RmdCanDriver(RmdConfig cfg, ungula::hal::can::ICan &bus)
        : cfg_(cfg)
        , bus_(bus)
{
}

// =====================================================================
// Unit conversion helpers (also exposed via the class)
// =====================================================================

int32_t RmdCanDriver::spsToCentideg(uint32_t sps, uint32_t stepsPerRevolution)
{
        if (stepsPerRevolution == 0) {
                return 0;
        }
        // SPS × (360 deg / stepsPerRev) × (100 centideg / deg)
        //   = SPS × 36000 / stepsPerRev
        // Use 64-bit math so the multiply doesn't overflow at higher SPS.
        const int64_t v =
            (static_cast<int64_t>(sps) * 36000LL) / static_cast<int64_t>(stepsPerRevolution);
        if (v > INT32_MAX)
                return INT32_MAX;
        if (v < INT32_MIN)
                return INT32_MIN;
        return static_cast<int32_t>(v);
}

int32_t RmdCanDriver::stepsToCentideg(int32_t steps, uint32_t stepsPerRevolution)
{
        if (stepsPerRevolution == 0) {
                return 0;
        }
        const int64_t v =
            (static_cast<int64_t>(steps) * 36000LL) / static_cast<int64_t>(stepsPerRevolution);
        if (v > INT32_MAX)
                return INT32_MAX;
        if (v < INT32_MIN)
                return INT32_MIN;
        return static_cast<int32_t>(v);
}

// =====================================================================
// IMotorDriver
// =====================================================================

Status RmdCanDriver::begin()
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (cfg_.motorId == 0 || cfg_.motorId > canbus_rmd::MOTOR_ID_MAX) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (cfg_.stepsPerRevolution == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Identity read at begin — required. A timeout here means no
        // RMD is responding on this motor ID, so we refuse to come up
        // rather than letting the host arm motion against a phantom
        // drive. Real RMDs reply to 0x12 in well under the 100 ms
        // timeout baked into `readModel`.
        uint8_t reply[8] = {};
        if (!canbus_rmd::readModel(bus_, cfg_.motorId, reply)) {
                return Status::Err(ErrorCode::TransportError);
        }
        identity_.firmwareMajor = reply[1];
        identity_.firmwareMinor = reply[2];
        identity_.rawId =
            (static_cast<uint32_t>(reply[1]) << 24) | (static_cast<uint32_t>(reply[2]) << 16) |
            (static_cast<uint32_t>(reply[3]) << 8) | static_cast<uint32_t>(reply[4]);
        begun_ = true;
        return Status::Ok();
}

Status RmdCanDriver::enable()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        // wakeUp (0x88) wakes the chip from a prior shutdown and clears
        // any latched faults. Safe to send even when already enabled.
        return canbus_rmd::wakeUp(bus_, cfg_.motorId) ?
                   Status::Ok() :
                   Status::Err(ErrorCode::TransportError);
}

Status RmdCanDriver::disable()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        running_ = false;
        const bool ok = cfg_.releaseBrakeOnDisable ?
                            canbus_rmd::shutdown(bus_, cfg_.motorId) :
                            canbus_rmd::stop(bus_, cfg_.motorId);
        return ok ? Status::Ok() : Status::Err(ErrorCode::TransportError);
}

Status RmdCanDriver::clearFault()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        faulted_ = false;
        lastFinishedReason_ = StopReason::None;
        return canbus_rmd::wakeUp(bus_, cfg_.motorId) ?
                   Status::Ok() :
                   Status::Err(ErrorCode::TransportError);
}

Status RmdCanDriver::armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps,
                             uint32_t /*accelSps2*/, uint32_t /*decelSps2*/)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (targetSteps == 0 || cruiseSps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // Build an absolute centideg target from the current
        // commanded position plus the requested delta.
        const int32_t deltaCentideg =
            stepsToCentideg(static_cast<int32_t>(targetSteps), cfg_.stepsPerRevolution);
        const int32_t currentCentideg =
            stepsToCentideg(commandedPosition_, cfg_.stepsPerRevolution);
        const int32_t targetCentideg = (dir == Direction::Forward) ?
                                           (currentCentideg + deltaCentideg) :
                                           (currentCentideg - deltaCentideg);

        // RMD V3's 0xA4 max-speed field is uint16 deg/s (not centideg).
        const int32_t centiPerSec = spsToCentideg(cruiseSps, cfg_.stepsPerRevolution);
        const int32_t degPerSec = centiPerSec / 100;
        const uint16_t maxSpeedDps = (degPerSec <= 0) ?
                                         0u :
                                         (degPerSec >= 65535 ? 65535u :
                                                               static_cast<uint16_t>(degPerSec));

        if (!canbus_rmd::moveAbsolute(bus_, cfg_.motorId, targetCentideg, maxSpeedDps)) {
                return Status::Err(ErrorCode::TransportError);
        }
        commandedPosition_ = (dir == Direction::Forward) ?
                                 commandedPosition_ + static_cast<int32_t>(targetSteps) :
                                 commandedPosition_ - static_cast<int32_t>(targetSteps);
        commandedSps_ = cruiseSps;
        running_ = true;
        lastFinishedReason_ = StopReason::None;
        return Status::Ok();
}

Status RmdCanDriver::armJog(Direction dir, uint32_t cruiseSps, uint32_t /*accelSps2*/)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (cruiseSps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        const int32_t mag = spsToCentideg(cruiseSps, cfg_.stepsPerRevolution);
        const int32_t signed_v = (dir == Direction::Forward) ? mag : -mag;
        if (!canbus_rmd::sendSpeed(bus_, cfg_.motorId, signed_v)) {
                return Status::Err(ErrorCode::TransportError);
        }
        commandedSps_ = cruiseSps;
        running_ = true;
        lastFinishedReason_ = StopReason::None;
        return Status::Ok();
}

Status RmdCanDriver::stop(StopMode mode)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        running_ = false;
        commandedSps_ = 0;
        if (mode == StopMode::Immediate) {
                lastFinishedReason_ = StopReason::EmergencyStop;
                return canbus_rmd::shutdown(bus_, cfg_.motorId) ?
                           Status::Ok() :
                           Status::Err(ErrorCode::TransportError);
        }
        lastFinishedReason_ = StopReason::UserStop;
        return canbus_rmd::stop(bus_, cfg_.motorId) ?
                   Status::Ok() :
                   Status::Err(ErrorCode::TransportError);
}

DriverMotionStatus RmdCanDriver::motionStatus() const
{
        DriverMotionStatus s;
        s.running = running_;
        s.faulted = faulted_;
        s.finishedReason = lastFinishedReason_;
        return s;
}

Position RmdCanDriver::commandedPositionSteps() const
{
        // Tracked from absolute-position commands. The real motor
        // position can drift from this under closed-loop tracking — the
        // host should use `ungula::canbus::rmd::readPosition` (or a
        // separate `lib_hal::Quadrature` channel) if precise feedback
        // is needed.
        return commandedPosition_;
}

uint32_t RmdCanDriver::commandedSpsNow() const
{
        return commandedSps_;
}

Status RmdCanDriver::resetPosition(Position newSteps)
{
        commandedPosition_ = newSteps;
        return Status::Ok();
}

DriverIdentity RmdCanDriver::identity()
{
        return identity_;
}

IntentSupport RmdCanDriver::applyIntent(MotorIntent intent)
{
        if (intent == MotorIntent::Default) {
                return IntentSupport::Supported;
        }
        // RMD motors' chopper / current control is firmware-baked on
        // the motor side. Intents that map to chip behaviour (Quiet,
        // HighTorque, Cool, …) cannot be translated to CAN commands
        // in this protocol. Honest "Unsupported" — the lib does not
        // pretend it honoured the request.
        return IntentSupport::Unsupported;
}

void RmdCanDriver::fillDriverDiagnostics(MotorDiagnostics &out) const
{
        out.identity = identity_;
        out.totalStepsIssued = 0;
        out.stall_valid = false;
        out.adaptive_current_valid = false;
}

} // namespace ungula::motor::rmd

#endif // UNGULA_USE_CANBUS
