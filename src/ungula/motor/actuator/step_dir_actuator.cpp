// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/actuator/step_dir_actuator.h"

#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;

StepDirActuator::StepDirActuator(IPulseEngine &engine, const Config &cfg)
        : engine_(engine)
        , cfg_(cfg)
{
}

// =====================================================================
// Lifecycle
// =====================================================================

Status StepDirActuator::begin()
{
        if (begun_)
                return Status::Err(ErrorCode::AlreadyInitialized);

        // The pulse engine configures STEP and DIR pins as part of its own
        // begin(); we must NOT touch those pins from here (see the
        // "Direction-timing contract" note in the header). We only own the
        // ENABLE pin, if one is configured.
        if (cfg_.enablePin != GPIO_NONE) {
                if (!gpio::configOutput(cfg_.enablePin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                // Seed to the disabled state so motion can't start until
                // `enable()` is called explicitly. Active-LOW driver →
                // disabled means HIGH; active-HIGH → disabled means LOW.
                gpio::write(cfg_.enablePin, cfg_.enableActiveLow ? true : false);
        }
        // Optional secondary enable for tandem-wired setups (two drives
        // sharing the same STEP signal, each with its own SVON / EN
        // input). No timing-sensitive ordering — both writes happen in
        // sequence here and again in `enable` / `disable`.
        if (cfg_.secondaryEnablePin != GPIO_NONE) {
                if (!gpio::configOutput(cfg_.secondaryEnablePin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                gpio::write(cfg_.secondaryEnablePin,
                            cfg_.secondaryEnableActiveLow ? true : false);
        }

        // Bring the pulse engine up. PulseMode::Internal is the only
        // supported mode; External is rejected by the engine itself.
        const auto s = engine_.begin(PulseMode::Internal);
        if (!s.ok() && s.error() != ErrorCode::AlreadyInitialized) {
                return s;
        }

        begun_ = true;
        enabled_ = false;
        return Status::Ok();
}

Status StepDirActuator::enable()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (cfg_.enablePin != GPIO_NONE) {
                // active-LOW → enabled = LOW. active-HIGH → enabled = HIGH.
                gpio::write(cfg_.enablePin, cfg_.enableActiveLow ? false : true);
        }
        if (cfg_.secondaryEnablePin != GPIO_NONE) {
                gpio::write(cfg_.secondaryEnablePin,
                            cfg_.secondaryEnableActiveLow ? false : true);
        }
        enabled_ = true;
        return Status::Ok();
}

Status StepDirActuator::disable()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);

        // Stop motion FIRST so the driver doesn't see a half-pulse at the
        // moment we drop the power stage. `Immediate` mode — we want to
        // halt now, not wait for a decel ramp. (Decel ramp during disable
        // is a user-level orchestration concern; the actuator's contract
        // is "disable now").
        if (engine_.isRunning()) {
                const auto s = engine_.stop(StopMode::Immediate);
                if (!s.ok())
                        return s;
        }

        if (cfg_.enablePin != GPIO_NONE) {
                gpio::write(cfg_.enablePin, cfg_.enableActiveLow ? true : false);
        }
        if (cfg_.secondaryEnablePin != GPIO_NONE) {
                gpio::write(cfg_.secondaryEnablePin,
                            cfg_.secondaryEnableActiveLow ? true : false);
        }
        enabled_ = false;
        return Status::Ok();
}

// =====================================================================
// Motion control — pure forwarding to the pulse engine.
// =====================================================================

Status StepDirActuator::armMotion(const PlannedMove &move)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!enabled_)
                return Status::Err(ErrorCode::NotEnabled);
        return engine_.loadMove(move);
}

Status StepDirActuator::startMotion()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!enabled_)
                return Status::Err(ErrorCode::NotEnabled);
        return engine_.start();
}

Status StepDirActuator::stop(StopMode mode)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        // Forward verbatim — the engine reports `Unsupported` for
        // `Decelerate`. We do NOT swap modes on the caller's behalf;
        // accurate stop semantics matter for motion control safety.
        return engine_.stop(mode);
}

Status StepDirActuator::emergencyStop()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        return engine_.emergencyStop();
}

// =====================================================================
// State queries
// =====================================================================

AxisFeedback StepDirActuator::feedback() const
{
        AxisFeedback f;
        f.commandedPosition = engine_.commandedPositionSteps();

        const auto s = engine_.status();
        // Open-loop stepper: actual position is "assumed equal to
        // commanded" — we don't expose hasActualPosition=true because we
        // have no way to verify it. The host should treat
        // `commandedPosition` as the only authoritative figure. STEP/DIR
        // servos share the same situation until encoder feedback is wired.
        f.actualPosition = f.commandedPosition;
        f.hasActualPosition = false;

        // currentSps derived from the active segment's half-period when
        // running. Status::segmentIndex is updated by the ISR; reading it
        // here is a snapshot. If the engine isn't running, velocity is 0.
        f.currentSps = 0;
        f.direction = Direction::Forward;
        if (s.running) {
                // The actuator doesn't have direct access to the segment list
                // (that's the engine's internal state). `PulseEngineStatus`
                // doesn't currently expose velocity; leaving as 0 with a
                // TODO: when the engine exposes `currentSps()` (needed for
                // `Axis::stop(Decelerate)` synthesis), wire it through here.
        }

        // Alarm / in-position read paths plug in here once the host
        // wires the optional digital inputs through SensorBank. Defaults
        // are safe.
        f.inPosition = !s.running; // simplistic — true when idle
        f.alarmActive = false;
        f.followingErrorSteps = 0;
        f.hasFollowingError = false;

        return f;
}

ActuatorCapabilities StepDirActuator::capabilities() const
{
        ActuatorCapabilities c;
        c.hasEnablePin = (cfg_.enablePin != GPIO_NONE);
        c.hasDirectionPin = true;
        c.hasPulseEngine = true;
        // Open-loop has no feedback path; STEP/DIR servo only has it
        // when an encoder is wired. Encoder wiring is not part of the
        // actuator surface today, so we report false for both kinds.
        c.hasActualPosition = false;
        c.hasInPositionInput = cfg_.hasInPositionInput;
        c.hasAlarmInput = cfg_.hasAlarmInput;
        c.hasFollowingError = false; // requires encoder
        c.hasNativeHoming = false; // STEP/DIR drives never self-home
        c.hasStallDetection = false; // TMC2209 has it, but as a
                                     // driver-level capability surfaced
                                     // via `Tmc2209Configurator`.
        return c;
}

FaultStatus StepDirActuator::faultStatus() const
{
        FaultStatus fs;
        const auto s = engine_.status();
        if (!s.faulted)
                return fs; // FaultCode::None, driverDetail = 0

        // Map engine fault reason to Axis-level FaultCode. Must agree with
        // the codes the Axis emits on the event path (see pumpSensors) —
        // a listener and a `faultStatus()` query against the same event
        // must report the same FaultCode.
        switch (s.finishedReason) {
        case StopReason::EmergencyStop:
                fs.code = FaultCode::EmergencyStop;
                break;
        case StopReason::DriverFault:
                // The pulse engine latches DriverFault on any ISR-side
                // HAL timer failure. At the Axis layer that's a pulse-
                // engine fault, not a stepper-driver fault.
                fs.code = FaultCode::PulseEngineFault;
                break;
        case StopReason::StallDetected:
                fs.code = FaultCode::Stall;
                break;
        case StopReason::LimitSwitch:
        case StopReason::TravelLimit:
                fs.code = FaultCode::LimitExceeded;
                break;
        default:
                // Engine says faulted but the reason doesn't map cleanly
                // — surface it as a generic driver fault rather than
                // hiding it as None.
                fs.code = FaultCode::DriverFault;
                break;
        }
        return fs;
}

Status StepDirActuator::clearFault()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        return engine_.clearFault();
}

Result<DriverIdentity> StepDirActuator::readDriverIdentity()
{
        // Generic STEP/DIR actuator: identity is only available when
        // the host (or kit) wired a chip-specific provider into the
        // Config. Without one, the actuator has no way to know what
        // is on the other end of the STEP/DIR wires (A4988? TMC2209?
        // YPMC?), so the honest answer is Unsupported.
        if (!cfg_.identityProvider)
                return Result<DriverIdentity>::Err(ErrorCode::Unsupported);
        return cfg_.identityProvider->readDriverIdentity();
}

} // namespace ungula::motor
