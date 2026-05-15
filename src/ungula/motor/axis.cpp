// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/axis.h"

#include <cmath>

#include "ungula/hal/timer/drivers/hwtimer.h"
#include "ungula/core/time/time.h"

#include "ungula/motor/actuator/i_axis_actuator.h"
#include "ungula/motor/actuator/step_dir_actuator.h"
#include "ungula/motor/homing/i_homing_strategy.h"
#include "ungula/motor/pulse/hal_pulse_engine.h"
#include "ungula/motor/pulse/i_pulse_engine.h"

namespace ungula::motor
{

namespace
{

        constexpr bool isPinSet(uint8_t pin)
        {
                return pin != GPIO_NONE;
        }

        Status validateStepDirCommon(const AxisCommonConfig &common, uint8_t stepPin,
                                     uint8_t dirPin)
        {
                if (!isPinSet(stepPin) || !isPinSet(dirPin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                const auto &L = common.limits;
                if (L.maxVelocitySps <= 0 || L.accelSpsPerSec == 0 || L.decelSpsPerSec == 0) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                if (L.maxStepRateSps == 0) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                return Status::Ok();
        }

        void copyAxisSensors(SensorInputConfig *dst, uint8_t &dstCount,
                             const SensorInputConfig *src, uint8_t count)
        {
                if (count > MAX_SENSOR_INPUTS)
                        count = MAX_SENSOR_INPUTS;
                for (uint8_t i = 0; i < count; ++i)
                        dst[i] = src[i];
                dstCount = count;
        }

} // namespace

// =====================================================================
// Factories
// =====================================================================

Result<std::unique_ptr<Axis>> Axis::createStepDirStepper(const StepDirStepperAxisConfig &cfg)
{
        const auto v = validateStepDirCommon(cfg.common, cfg.stepPin.value, cfg.dirPin.value);
        if (!v.ok())
                return Result<std::unique_ptr<Axis>>::Err(v.error());

        auto timer = std::make_unique<ungula::hal::timer::drivers::HwTimer>();

        HalPulseEngine::Config eCfg;
        eCfg.stepPin = cfg.stepPin.value;
        eCfg.dirPin = cfg.dirPin.value;
        eCfg.dirActiveHigh = cfg.dirActiveHigh;
        eCfg.dirSetupUs = cfg.dirSetupUs;
        eCfg.timerResolutionHz = 1'000'000;
        eCfg.timerMinTicks = 5;
        auto engine = std::make_unique<HalPulseEngine>(*timer, eCfg);

        StepDirActuator::Config aCfg;
        aCfg.kind = StepDirActuatorKind::OpenLoopStepper;
        aCfg.enablePin = cfg.enablePin.value;
        aCfg.enableActiveLow = cfg.enableActiveLow;
        aCfg.hasAlarmInput = false;
        aCfg.hasInPositionInput = false;
        auto actuator = std::make_unique<StepDirActuator>(*engine, aCfg);

        std::unique_ptr<Axis> axis(new Axis());
        axis->common_ = cfg.common;
        axis->axisId_ = cfg.common.axisId;
        axis->timerResolutionHz_ = eCfg.timerResolutionHz;
        axis->timerMinTicks_ = eCfg.timerMinTicks;
        axis->timer_ = std::move(timer);
        axis->engine_ = std::move(engine);
        axis->actuator_ = std::move(actuator);
        copyAxisSensors(axis->sensorConfigs_, axis->sensorConfigCount_, cfg.sensors,
                        cfg.sensorCount);
        return Result<std::unique_ptr<Axis>>::Ok(std::move(axis));
}

Result<std::unique_ptr<Axis>> Axis::createStepDirServo(const StepDirServoAxisConfig &cfg)
{
        const auto v = validateStepDirCommon(cfg.common, cfg.stepPin.value, cfg.dirPin.value);
        if (!v.ok())
                return Result<std::unique_ptr<Axis>>::Err(v.error());

        auto timer = std::make_unique<ungula::hal::timer::drivers::HwTimer>();

        HalPulseEngine::Config eCfg;
        eCfg.stepPin = cfg.stepPin.value;
        eCfg.dirPin = cfg.dirPin.value;
        eCfg.dirActiveHigh = cfg.dirActiveHigh;
        eCfg.dirSetupUs = cfg.dirSetupUs;
        eCfg.timerResolutionHz = 1'000'000;
        eCfg.timerMinTicks = 5;
        auto engine = std::make_unique<HalPulseEngine>(*timer, eCfg);

        StepDirActuator::Config aCfg;
        aCfg.kind = StepDirActuatorKind::StepDirServo;
        aCfg.enablePin = cfg.enablePin.value;
        aCfg.enableActiveLow = cfg.enableActiveLow;
        aCfg.hasAlarmInput = isPinSet(cfg.alarmInputPin.value);
        aCfg.hasInPositionInput = isPinSet(cfg.inPositionInputPin.value);
        auto actuator = std::make_unique<StepDirActuator>(*engine, aCfg);

        std::unique_ptr<Axis> axis(new Axis());
        axis->common_ = cfg.common;
        axis->axisId_ = cfg.common.axisId;
        axis->timerResolutionHz_ = eCfg.timerResolutionHz;
        axis->timerMinTicks_ = eCfg.timerMinTicks;
        axis->timer_ = std::move(timer);
        axis->engine_ = std::move(engine);
        axis->actuator_ = std::move(actuator);
        copyAxisSensors(axis->sensorConfigs_, axis->sensorConfigCount_, cfg.sensors,
                        cfg.sensorCount);
        return Result<std::unique_ptr<Axis>>::Ok(std::move(axis));
}

Result<std::unique_ptr<Axis>> Axis::createCanServo(const CanServoAxisConfig & /*cfg*/)
{
        return Result<std::unique_ptr<Axis>>::Err(ErrorCode::Unsupported);
}

Result<std::unique_ptr<Axis>> Axis::createComposed(ComposedComponents components,
                                                   const SensorInputConfig *sensors,
                                                   uint8_t sensorCount)
{
        if (!components.actuator) {
                return Result<std::unique_ptr<Axis>>::Err(ErrorCode::InvalidConfig);
        }
        if (components.common.limits.maxVelocitySps <= 0 ||
            components.common.limits.accelSpsPerSec == 0 ||
            components.common.limits.decelSpsPerSec == 0 ||
            components.common.limits.maxStepRateSps == 0) {
                return Result<std::unique_ptr<Axis>>::Err(ErrorCode::InvalidConfig);
        }

        std::unique_ptr<Axis> axis(new Axis());
        axis->common_ = components.common;
        axis->axisId_ = components.common.axisId;
        axis->timerResolutionHz_ = components.timerResolutionHz;
        axis->timerMinTicks_ = components.timerMinTicks;
        axis->timer_ = std::move(components.timer);
        axis->engine_ = std::move(components.engine);
        axis->actuator_ = std::move(components.actuator);
        if (sensors && sensorCount > 0) {
                copyAxisSensors(axis->sensorConfigs_, axis->sensorConfigCount_, sensors,
                                sensorCount);
        }
        return Result<std::unique_ptr<Axis>>::Ok(std::move(axis));
}

// =====================================================================
// Construction / destruction
// =====================================================================

Axis::Axis() = default;

Axis::~Axis()
{
        sensors_.end();
}

// =====================================================================
// Lifecycle
// =====================================================================

Status Axis::begin()
{
        if (state_ != AxisState::Uninitialized) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (!actuator_)
                return Status::Err(ErrorCode::InvalidConfig);

        if (timer_) {
                ungula::hal::timer::HwTimerConfig hwCfg;
                hwCfg.resolutionHz = timerResolutionHz_;
                hwCfg.minTicks = timerMinTicks_;
                const auto hs = timer_->begin(hwCfg);
                if (hs != ungula::hal::timer::HwTimerStatus::Ok &&
                    hs != ungula::hal::timer::HwTimerStatus::AlreadyInitialized) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
        }

        const auto s = actuator_->begin();
        if (!s.ok())
                return s;

        // Bring up sensors AFTER the actuator/engine so the crash/estop
        // ISR has a real engine to halt. Skipped if no sensors are
        // configured (no ISR install, no GPIO touched).
        if (sensorConfigCount_ > 0) {
                const auto sb = sensors_.begin(sensorConfigs_, sensorConfigCount_, engine_.get());
                if (!sb.ok())
                        return sb;
        }

        state_ = AxisState::Disabled;
        emitEvent(AxisEventType::StateChanged);
        return Status::Ok();
}

Status Axis::enable()
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }
        const auto s = actuator_->enable();
        if (!s.ok())
                return s;
        state_ = AxisState::Idle;
        emitEvent(AxisEventType::StateChanged);
        return Status::Ok();
}

Status Axis::disable()
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        const auto s = actuator_->disable();
        if (!s.ok())
                return s;
        motionInFlight_ = false;
        state_ = AxisState::Disabled;
        emitEvent(AxisEventType::StateChanged);
        return Status::Ok();
}

// =====================================================================
// Unit + planning helpers
// =====================================================================

Result<Distance> Axis::resolveToSteps(int32_t value, DistanceUnit unit) const
{
        switch (unit) {
        case DistanceUnit::Steps:
                return Result<Distance>::Ok(value);
        case DistanceUnit::Mm: {
                const float k = common_.units.stepsPerMm;
                if (k <= 0.0f)
                        return Result<Distance>::Err(ErrorCode::InvalidConfig);
                return Result<Distance>::Ok(static_cast<Distance>(std::lround(value * k)));
        }
        case DistanceUnit::Cm: {
                const float k = common_.units.stepsPerMm;
                if (k <= 0.0f)
                        return Result<Distance>::Err(ErrorCode::InvalidConfig);
                return Result<Distance>::Ok(static_cast<Distance>(std::lround(value * 10.0f * k)));
        }
        case DistanceUnit::Degrees: {
                const float k = common_.units.stepsPerDegree;
                if (k <= 0.0f)
                        return Result<Distance>::Err(ErrorCode::InvalidConfig);
                return Result<Distance>::Ok(static_cast<Distance>(std::lround(value * k)));
        }
        }
        return Result<Distance>::Err(ErrorCode::InvalidConfig);
}

TrajectoryLimits Axis::limitsForFeed(Velocity feedSps) const
{
        TrajectoryLimits L = common_.limits;
        if (feedSps > 0 && feedSps < L.maxVelocitySps) {
                L.maxVelocitySps = feedSps;
        }
        return L;
}

Status Axis::armAndStart(const PlannedMove &move, AxisState onStart)
{
        if (move.totalSteps == 0 || move.segmentCount == 0) {
                return Status::Ok();
        }
        const auto sArm = actuator_->armMotion(move);
        if (!sArm.ok())
                return sArm;

        const auto sStart = actuator_->startMotion();
        if (!sStart.ok())
                return sStart;

        state_ = onStart;
        lastStopReason_ = StopReason::None;
        lastMoveDirection_ = move.direction;
        motionInFlight_ = true;
        // Arm the stall-detection window so the SensorBank discards
        // DIAG hits during the StealthChop auto-tune transient.
        sensors_.notifyMotionStart(lastServiceMs_ != 0
                                       ? lastServiceMs_
                                       : ungula::core::time::millis());
        emitEvent(AxisEventType::MotionStarted);
        emitEvent(AxisEventType::StateChanged);
        return Status::Ok();
}

// =====================================================================
// Motion commands
// =====================================================================

Status Axis::moveTo(Position target, DistanceUnit unit)
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (state_ == AxisState::Disabled)
                return Status::Err(ErrorCode::NotEnabled);
        if (state_ == AxisState::Moving || state_ == AxisState::Jogging ||
            state_ == AxisState::Homing) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }

        auto resolved = resolveToSteps(target, unit);
        if (!resolved.ok())
                return Status::Err(resolved.error());
        const Position targetSteps = resolved.takeValue();

        const Position currentPos = actuator_->feedback().commandedPosition;
        const auto move = planner_.planTo(currentPos, targetSteps, common_.limits,
                                          timerResolutionHz_, timerMinTicks_);
        return armAndStart(move, AxisState::Moving);
}

Status Axis::moveBy(Distance delta, DistanceUnit unit)
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (state_ == AxisState::Disabled)
                return Status::Err(ErrorCode::NotEnabled);
        if (state_ == AxisState::Moving || state_ == AxisState::Jogging ||
            state_ == AxisState::Homing) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }

        auto resolved = resolveToSteps(delta, unit);
        if (!resolved.ok())
                return Status::Err(resolved.error());

        const auto move = planner_.planBy(resolved.takeValue(), common_.limits, timerResolutionHz_,
                                          timerMinTicks_);
        return armAndStart(move, AxisState::Moving);
}

Status Axis::jog(Direction direction)
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (state_ == AxisState::Disabled)
                return Status::Err(ErrorCode::NotEnabled);
        if (state_ == AxisState::Moving || state_ == AxisState::Jogging ||
            state_ == AxisState::Homing) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }

        constexpr uint32_t kJogSafetySteps = 1'000'000;
        const auto move = planner_.planJog(direction, kJogSafetySteps, common_.limits,
                                           timerResolutionHz_, timerMinTicks_);
        return armAndStart(move, AxisState::Jogging);
}

Status Axis::jog(Direction direction, Speed s)
{
        // Resolve user units against the configured scaling, then run
        // through `limitsForFeed` so the planner sees a one-shot
        // velocity cap without mutating `common_.limits`. The next
        // unqualified `jog(Direction)` reverts to the configured max.
        auto r = toStepsPerSec(s, common_.units);
        if (!r.ok())
                return Status::Err(r.error());

        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (state_ == AxisState::Disabled)
                return Status::Err(ErrorCode::NotEnabled);
        if (state_ == AxisState::Moving || state_ == AxisState::Jogging ||
            state_ == AxisState::Homing) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }

        constexpr uint32_t kJogSafetySteps = 1'000'000;
        const auto L = limitsForFeed(r.takeValue());
        const auto move = planner_.planJog(direction, kJogSafetySteps, L,
                                           timerResolutionHz_, timerMinTicks_);
        return armAndStart(move, AxisState::Jogging);
}

// --- Trajectory tuning (runtime) ---------------------------------

Status Axis::setMaxVelocity(Speed s)
{
        if (motionInFlight_)
                return Status::Err(ErrorCode::MotionInProgress);
        return applyMaxVelocity(common_.limits, s, common_.units);
}

Status Axis::setAcceleration(Acceleration a)
{
        if (motionInFlight_)
                return Status::Err(ErrorCode::MotionInProgress);
        return applyAcceleration(common_.limits, a, common_.units);
}

Status Axis::setDeceleration(Acceleration a)
{
        if (motionInFlight_)
                return Status::Err(ErrorCode::MotionInProgress);
        return applyDeceleration(common_.limits, a, common_.units);
}

Status Axis::setRampProfile(Acceleration a)
{
        if (motionInFlight_)
                return Status::Err(ErrorCode::MotionInProgress);
        return applyRampProfile(common_.limits, a, common_.units);
}

Status Axis::stop(StopMode mode)
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (mode == StopMode::Decelerate)
                return Status::Err(ErrorCode::Unsupported);

        const auto s = actuator_->stop(mode);
        if (!s.ok())
                return s;

        if (motionInFlight_) {
                motionInFlight_ = false;
                lastStopReason_ = StopReason::UserStop;
                state_ = AxisState::Idle;
                emitEvent(AxisEventType::MotionStopped, StopReason::UserStop);
                emitEvent(AxisEventType::StateChanged);
        }
        return Status::Ok();
}

Status Axis::emergencyStop()
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);

        const auto s = actuator_->emergencyStop();
        if (!s.ok())
                return s;

        motionInFlight_ = false;
        state_ = AxisState::EmergencyStopped;
        lastStopReason_ = StopReason::EmergencyStop;
        emitEvent(AxisEventType::EmergencyStopped, StopReason::EmergencyStop,
                  FaultCode::EmergencyStop);
        emitEvent(AxisEventType::FaultRaised, StopReason::EmergencyStop, FaultCode::EmergencyStop);
        emitEvent(AxisEventType::StateChanged);
        return Status::Ok();
}

// =====================================================================
// Homing
// =====================================================================

Status Axis::setHomingStrategy(IHomingStrategy *strategy, uint32_t timeoutMs)
{
        homing_.setStrategy(strategy, timeoutMs);
        return Status::Ok();
}

Status Axis::home()
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (state_ == AxisState::Disabled)
                return Status::Err(ErrorCode::NotEnabled);
        if (state_ == AxisState::Moving || state_ == AxisState::Jogging ||
            state_ == AxisState::Homing) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }

        const int64_t nowMs = ungula::core::time::millis();
        const auto s = homing_.begin(*this, nowMs);
        if (!s.ok())
                return s;

        state_ = AxisState::Homing;
        isHomed_ = false;
        emitEvent(AxisEventType::HomingStarted);
        emitEvent(AxisEventType::StateChanged);
        return Status::Ok();
}

bool Axis::isHoming() const
{
        return homing_.isActive();
}
bool Axis::isHomed() const
{
        return isHomed_;
}

// =====================================================================
// Faults
// =====================================================================

Status Axis::clearFault()
{
        if (state_ == AxisState::Uninitialized)
                return Status::Err(ErrorCode::NotInitialized);
        if (!actuator_)
                return Status::Err(ErrorCode::InvalidConfig);

        const auto s = actuator_->clearFault();
        if (!s.ok())
                return s;

        if (state_ == AxisState::Faulted || state_ == AxisState::EmergencyStopped) {
                state_ = AxisState::Disabled;
                emitEvent(AxisEventType::FaultCleared);
                emitEvent(AxisEventType::StateChanged);
        }
        return Status::Ok();
}

FaultStatus Axis::faultStatus() const
{
        if (!actuator_)
                return {};
        return actuator_->faultStatus();
}

// =====================================================================
// Queries
// =====================================================================

AxisState Axis::state() const
{
        if (actuator_) {
                const auto fs = actuator_->faultStatus();
                if (fs.active() && state_ != AxisState::Faulted &&
                    state_ != AxisState::EmergencyStopped) {
                        return AxisState::Faulted;
                }
        }
        return state_;
}

AxisFeedback Axis::feedback() const
{
        if (!actuator_)
                return {};
        return actuator_->feedback();
}

StopReason Axis::lastStopReason() const
{
        return lastStopReason_;
}
AxisId Axis::id() const
{
        return axisId_;
}

// =====================================================================
// Service path
// =====================================================================

void Axis::service(int64_t nowMs)
{
        lastServiceMs_ = nowMs;

        pumpSensors(nowMs);
        pumpMotionState();

        if (homing_.isActive()) {
                homing_.tick(*this, nowMs);
                // The controller may have transitioned to Complete / Failed
                // during this call. Reflect that in the Axis state and emit
                // the corresponding events.
                if (!homing_.isActive()) {
                        const bool ok = homing_.succeeded();
                        if (ok) {
                                isHomed_ = true;
                                emitEvent(AxisEventType::HomingCompleted);
                        } else {
                                lastStopReason_ = homing_.failureReason();
                                emitEvent(AxisEventType::HomingFailed, lastStopReason_);
                        }
                        state_ = AxisState::Idle;
                        emitEvent(AxisEventType::StateChanged);
                }
        }

        (void)events_.drain();
}

void Axis::pumpSensors(int64_t nowMs)
{
        sensors_.service(nowMs);

        // Crash and E-stop are ISR-latched. The ISR has already halted
        // the pulse engine; here we just bookkeep state + emit events.
        if (sensors_.consumeEstopActivation()) {
                motionInFlight_ = false;
                state_ = AxisState::EmergencyStopped;
                lastStopReason_ = StopReason::EmergencyStop;
                emitEvent(AxisEventType::LimitActivated, StopReason::EmergencyStop);
                emitEvent(AxisEventType::EmergencyStopped, StopReason::EmergencyStop,
                          FaultCode::EmergencyStop);
                emitEvent(AxisEventType::FaultRaised, StopReason::EmergencyStop,
                          FaultCode::EmergencyStop);
                emitEvent(AxisEventType::StateChanged);
        }
        if (sensors_.consumeCrashActivation()) {
                motionInFlight_ = false;
                state_ = AxisState::Faulted;
                lastStopReason_ = StopReason::LimitSwitch;
                emitEvent(AxisEventType::LimitActivated, StopReason::LimitSwitch);
                emitEvent(AxisEventType::FaultRaised, StopReason::LimitSwitch,
                          FaultCode::LimitExceeded);
                emitEvent(AxisEventType::StateChanged);
        }
        if (sensors_.consumeStallActivation()) {
                // Stall is its own reason / fault code. Distinguishes a
                // TMC2209 DIAG-pin event from a generic limit switch.
                motionInFlight_ = false;
                state_ = AxisState::Faulted;
                lastStopReason_ = StopReason::StallDetected;
                emitEvent(AxisEventType::LimitActivated, StopReason::StallDetected);
                emitEvent(AxisEventType::FaultRaised, StopReason::StallDetected,
                          FaultCode::Stall);
                emitEvent(AxisEventType::StateChanged);
        }

        // Travel-limit handling: if a limit in the current move's
        // direction is active, halt immediately. The Axis service tick
        // is fast enough (1-10 ms) that the small overshoot vs. ISR
        // path is acceptable — TravelLimits are recommended to be
        // backed up by CrashLimits on dangerous mechanics.
        if (motionInFlight_ && sensors_.isActive(SensorRole::TravelLimit, lastMoveDirection_)) {
                (void)actuator_->stop(StopMode::Immediate);
                motionInFlight_ = false;
                state_ = AxisState::Idle;
                lastStopReason_ = StopReason::TravelLimit;
                emitEvent(AxisEventType::LimitActivated, StopReason::TravelLimit);
                emitEvent(AxisEventType::MotionStopped, StopReason::TravelLimit);
                emitEvent(AxisEventType::StateChanged);
        }

        // Home sensor during homing: stop the in-flight jog so the
        // controller's next tick sees `isMotionIdle()` and lets the
        // strategy advance to the next phase.
        if (homing_.isActive() && motionInFlight_ && sensors_.isActive(SensorRole::Home)) {
                (void)actuator_->stop(StopMode::Immediate);
                motionInFlight_ = false;
                // Don't update state_ — the axis is still Homing.
                emitEvent(AxisEventType::MotionStopped, StopReason::LimitSwitch);
        }
}

void Axis::pumpMotionState()
{
        if (!engine_)
                return;
        if (!motionInFlight_)
                return;

        const auto st = engine_->status();
        if (st.running)
                return; // still in flight

        motionInFlight_ = false;

        // Map the engine's finished reason onto Axis-level events.
        if (st.faulted) {
                // The engine latched a fault — either from ISR (haltFromIsr
                // via a sensor) or from emergencyStop. The crash/estop
                // sensor handler in pumpSensors() already emitted the
                // appropriate events if the cause was a sensor; here we
                // catch the engine-internal fault path (timer rearm failure).
                if (state_ != AxisState::EmergencyStopped && state_ != AxisState::Faulted) {
                        state_ = AxisState::Faulted;
                        lastStopReason_ = st.finishedReason;
                        emitEvent(AxisEventType::FaultRaised, st.finishedReason,
                                  FaultCode::PulseEngineFault);
                        emitEvent(AxisEventType::StateChanged);
                }
                return;
        }

        lastStopReason_ = st.finishedReason;
        if (state_ == AxisState::Moving || state_ == AxisState::Jogging) {
                state_ = AxisState::Idle;
                if (st.finishedReason == StopReason::TargetReached) {
                        emitEvent(AxisEventType::MotionCompleted, st.finishedReason);
                } else {
                        emitEvent(AxisEventType::MotionStopped, st.finishedReason);
                }
                emitEvent(AxisEventType::StateChanged);
        }
        // During Homing, sub-motion completions are bookkept (above) but
        // no events fire here — the controller emits HomingCompleted /
        // HomingFailed once the whole cycle settles.
}

// =====================================================================
// Events
// =====================================================================

void Axis::emitEvent(AxisEventType type, StopReason stopReason, FaultCode faultCode)
{
        AxisEvent ev;
        ev.sequence = ++eventSequence_;
        ev.timestampMs = (lastServiceMs_ != 0) ? lastServiceMs_ : ungula::core::time::millis();
        ev.axisId = axisId_;
        ev.state = state_;
        if (actuator_) {
                const auto fb = actuator_->feedback();
                ev.commandedPosition = fb.commandedPosition;
                ev.actualPosition = fb.actualPosition;
                ev.hasActualPosition = fb.hasActualPosition;
        }
        ev.type = type;
        ev.stopReason = stopReason;
        ev.faultCode = faultCode;
        (void)events_.enqueue(ev);
}

Status Axis::subscribe(IAxisEventListener *l)
{
        return events_.subscribe(l);
}
uint32_t Axis::serviceEvents()
{
        return events_.drain();
}

// =====================================================================
// IHomingAxis — narrowed view the homing strategy operates against.
// =====================================================================

Status Axis::commandMove(Distance deltaSteps, Velocity feedSps)
{
        if (state_ != AxisState::Homing && state_ != AxisState::Idle) {
                // Allow homing strategies to issue moves while the axis is in
                // the Homing state. Direct callers reach for `moveBy()`.
                return Status::Err(ErrorCode::InvalidState);
        }
        const auto L = limitsForFeed(feedSps);
        const auto move = planner_.planBy(deltaSteps, L, timerResolutionHz_, timerMinTicks_);
        if (move.totalSteps == 0)
                return Status::Err(ErrorCode::InvalidConfig);

        const auto sArm = actuator_->armMotion(move);
        if (!sArm.ok())
                return sArm;
        const auto sStart = actuator_->startMotion();
        if (!sStart.ok())
                return sStart;

        lastMoveDirection_ = move.direction;
        motionInFlight_ = true;
        sensors_.notifyMotionStart(lastServiceMs_ != 0
                                       ? lastServiceMs_
                                       : ungula::core::time::millis());
        emitEvent(AxisEventType::MotionStarted);
        return Status::Ok();
}

Status Axis::commandJog(Direction direction, Velocity feedSps)
{
        if (state_ != AxisState::Homing && state_ != AxisState::Idle) {
                return Status::Err(ErrorCode::InvalidState);
        }
        constexpr uint32_t kJogSafetySteps = 1'000'000;
        const auto L = limitsForFeed(feedSps);
        const auto move =
            planner_.planJog(direction, kJogSafetySteps, L, timerResolutionHz_, timerMinTicks_);
        if (move.totalSteps == 0)
                return Status::Err(ErrorCode::InvalidConfig);

        const auto sArm = actuator_->armMotion(move);
        if (!sArm.ok())
                return sArm;
        const auto sStart = actuator_->startMotion();
        if (!sStart.ok())
                return sStart;

        lastMoveDirection_ = direction;
        motionInFlight_ = true;
        sensors_.notifyMotionStart(lastServiceMs_ != 0
                                       ? lastServiceMs_
                                       : ungula::core::time::millis());
        emitEvent(AxisEventType::MotionStarted);
        return Status::Ok();
}

Status Axis::stopMove()
{
        if (!motionInFlight_)
                return Status::Ok();
        const auto s = actuator_->stop(StopMode::Immediate);
        if (!s.ok())
                return s;
        motionInFlight_ = false;
        return Status::Ok();
}

bool Axis::isHomeActive() const
{
        return sensors_.isActive(SensorRole::Home);
}

bool Axis::isMotionIdle() const
{
        if (engine_)
                return !engine_->isRunning() && !motionInFlight_;
        if (actuator_)
                return actuator_->feedback().inPosition && !motionInFlight_;
        return true;
}

Status Axis::resetPosition(Position positionSteps)
{
        if (motionInFlight_)
                return Status::Err(ErrorCode::MotionInProgress);
        if (!engine_)
                return Status::Err(ErrorCode::Unsupported);
        return engine_->resetPosition(positionSteps);
}

} // namespace ungula::motor
