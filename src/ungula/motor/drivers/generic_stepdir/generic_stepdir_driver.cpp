// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/generic_stepdir/generic_stepdir_driver.h"

#include <memory>

#include "ungula/hal/gpio/gpio.h"
#include "ungula/motor/planners/trapezoidal_planner.h"
#include "ungula/motor/step_signal/rmt_step_signal.h"

namespace ungula::motor::stepdir
{

namespace gpio = ungula::hal::gpio;

GenericStepDirDriver::GenericStepDirDriver(GenericStepDirConfig cfg,
                                           IStepSignalGenerator &stepSignal,
                                           IMotionPlanner &planner)
        : cfg_(cfg)
        , ownedStepSignal_(nullptr)
        , ownedPlanner_(nullptr)
        , stepSignal_(stepSignal)
        , planner_(planner)
{
}

namespace
{
        RmtStepSignal::Config makeRmtCfg(const GenericStepDirConfig &cfg)
        {
                RmtStepSignal::Config rcfg;
                if (cfg.resolutionHz != 0u) {
                        rcfg.resolutionHz = cfg.resolutionHz;
                }
                return rcfg;
        }
} // namespace

GenericStepDirDriver::GenericStepDirDriver(GenericStepDirConfig cfg)
        : cfg_(cfg)
        , ownedStepSignal_(std::make_unique<RmtStepSignal>(makeRmtCfg(cfg)))
        , ownedPlanner_(std::make_unique<TrapezoidalPlanner>())
        , stepSignal_(*ownedStepSignal_)
        , planner_(*ownedPlanner_)
{
}

Status GenericStepDirDriver::setEnablePinLevel(bool enabled)
{
        if (cfg_.enablePin == GPIO_NONE) {
                return Status::Ok();
        }
        const bool level = cfg_.enableActiveLow ? !enabled : enabled;
        gpio::write(cfg_.enablePin, level);
        return Status::Ok();
}

Status GenericStepDirDriver::begin()
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (cfg_.enablePin != GPIO_NONE) {
                if (!gpio::configOutput(cfg_.enablePin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                (void)setEnablePinLevel(false);
        }
        // Self-owns mode: initialise the owned step signal generator
        // with the pin / timing fields from cfg_. Pluggable mode skips
        // this because the host already initialised its generator.
        if (ownedStepSignal_) {
                if (cfg_.stepPin == GPIO_NONE || cfg_.dirPin == GPIO_NONE) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                const auto sg = ownedStepSignal_->begin(cfg_.stepPin, cfg_.dirPin,
                                                        cfg_.dirActiveHigh, cfg_.dirSetupUs,
                                                        cfg_.minPulseHighUs, cfg_.minPulseLowUs);
                if (!sg.ok()) {
                        return sg;
                }
        }
        begun_ = true;
        return Status::Ok();
}

Status GenericStepDirDriver::enable()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        return setEnablePinLevel(true);
}

Status GenericStepDirDriver::disable()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        return setEnablePinLevel(false);
}

Status GenericStepDirDriver::clearFault()
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        return stepSignal_.clearFault();
}

Status GenericStepDirDriver::armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps,
                                     uint32_t accelSps2, uint32_t decelSps2)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (targetSteps == 0 || cruiseSps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // accel / decel of zero mean "no ramp" - planner emits one
        // cruise segment, motor jumps to cruise instantly.

        PlannerLimits L;
        L.maxVelocitySps = cruiseSps;
        L.accelSpsPerSec = accelSps2;
        L.decelSpsPerSec = decelSps2;
        L.hardStepRateCeilingSps = cruiseSps;
        // Pulse widths come from the drive config so the planner
        // honors the electrical timing configured by the host.
        L.minPulseHighUs = cfg_.minPulseHighUs;
        L.minPulseLowUs  = cfg_.minPulseLowUs;

        const Position from = stepSignal_.commandedPosition();
        const Position to = (dir == Direction::Forward) ? from + static_cast<int32_t>(targetSteps) :
                                                          from - static_cast<int32_t>(targetSteps);
        lastPlannedMove_ = planner_.planMove(from, to, L, stepSignal_.timerResolutionHz(),
                                             stepSignal_.minTimerTicks());
        if (lastPlannedMove_.totalSteps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Subclass hook (e.g. write secondary DIR pin BEFORE the
        // generator's armMove so both pins are stable during the
        // dirSetupUs window).
        onBeforeArm(dir);

        const auto s = stepSignal_.armMove(lastPlannedMove_);
        if (s.ok()) {
                motionInFlight_ = true;
                onMotionStarted();
        }
        return s;
}

Status GenericStepDirDriver::armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (cruiseSps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        // accel of zero = no ramp, single cruise segment.
        PlannerLimits L;
        L.maxVelocitySps = cruiseSps;
        L.accelSpsPerSec = accelSps2;
        L.decelSpsPerSec = accelSps2;
        L.hardStepRateCeilingSps = cruiseSps;
        L.minPulseHighUs = cfg_.minPulseHighUs;
        L.minPulseLowUs  = cfg_.minPulseLowUs;

        constexpr uint32_t kJogIndefiniteSteps = 0x7FFFFFFFu;
        lastPlannedMove_ = planner_.planJog(dir, kJogIndefiniteSteps, L,
                                            stepSignal_.timerResolutionHz(),
                                            stepSignal_.minTimerTicks());
        if (lastPlannedMove_.totalSteps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        onBeforeArm(dir);
        const auto s = stepSignal_.armMove(lastPlannedMove_);
        if (s.ok()) {
                motionInFlight_ = true;
                onMotionStarted();
        }
        return s;
}

Status GenericStepDirDriver::stop(StopMode mode)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        const auto s = stepSignal_.stop(mode);
        if (motionInFlight_) {
                motionInFlight_ = false;
                onMotionStopped();
        }
        return s;
}

DriverMotionStatus GenericStepDirDriver::motionStatus() const
{
        const auto s = stepSignal_.status();
        DriverMotionStatus out;
        out.running = s.running;
        out.faulted = s.faulted;
        out.finishedReason = s.finishedReason;
        // Detect natural completion to drive the subclass hook.
        if (motionInFlight_ && !s.running) {
                auto *self = const_cast<GenericStepDirDriver *>(this);
                self->motionInFlight_ = false;
                self->onMotionStopped();
        }
        return out;
}

Position GenericStepDirDriver::commandedPositionSteps() const
{
        return stepSignal_.commandedPosition();
}

uint32_t GenericStepDirDriver::commandedSpsNow() const
{
        return stepSignal_.commandedSpsNow();
}

Status GenericStepDirDriver::resetPosition(Position newSteps)
{
        return stepSignal_.resetPosition(newSteps);
}

DriverIdentity GenericStepDirDriver::identity()
{
        DriverIdentity id;
        id.vendor = "Unknown";
        id.model = "Generic STEP/DIR";
        id.firmwareMajor = 0;
        id.firmwareMinor = 0;
        id.rawId = 0;
        return id;
}

IntentSupport GenericStepDirDriver::applyIntent(MotorIntent intent)
{
        if (intent == MotorIntent::Default) {
                return IntentSupport::Supported;
        }
        // No chip-side knobs: every intent the host might set is
        // honest-not-honoured.
        return IntentSupport::Unsupported;
}

void GenericStepDirDriver::fillDriverDiagnostics(MotorDiagnostics &out) const
{
        out.identity = const_cast<GenericStepDirDriver *>(this)->identity();
        out.totalStepsIssued = 0;
        out.stall_valid = false;
        out.adaptive_current_valid = false;
}

} // namespace ungula::motor::stepdir
