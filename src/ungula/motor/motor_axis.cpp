// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/motor_axis.h"

#include <cmath>

#include "ungula/core/time/time.h"
#include "ungula/motor/planners/trapezoidal_planner.h"

namespace ungula::motor
{

namespace
{

        // ---- Unit-resolution helpers (host-facing → SPS / steps) ---------
        //
        // Single internal currency: SPS (Velocity), step (Position), SPS².
        // Every host-facing value with a unit tag is resolved at the axis
        // boundary by these helpers using the axis's configured
        // `MotorUnits`. Missing conversion factors (e.g. stepsPerMm == 0
        // when the host asked for mm/sec) produce `InvalidConfig`.

        Status speedToSps(Speed s, const MotorUnits &u, uint32_t &outSps)
        {
                const float v = static_cast<float>(s.value);
                float resolved = 0.0f;
                switch (s.unit) {
                case SpeedUnit::StepsPerSec:
                        resolved = v;
                        break;
                case SpeedUnit::Rpm:
                        if (u.stepsPerRevolution == 0) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = (v * static_cast<float>(u.stepsPerRevolution)) / 60.0f;
                        break;
                case SpeedUnit::DegreesPerSec:
                        if (u.stepsPerDegree == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * u.stepsPerDegree;
                        break;
                case SpeedUnit::MmPerSec:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * u.stepsPerMm;
                        break;
                case SpeedUnit::CmPerSec:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * 10.0f * u.stepsPerMm;
                        break;
                case SpeedUnit::CmPerMin:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        // cm/min → mm/sec → steps/sec.
                        // cm × 10 = mm; /60 = per second.
                        resolved = (v * 10.0f * u.stepsPerMm) / 60.0f;
                        break;
                }
                // Round-half-up. Truncation here biased every conversion
                // DOWN by sub-step (1000 RPM @ 10000 PPR yields 166666.67
                // sps → truncated to 166666). The planner already
                // ceiling-rounds the half-period so actual ≤ requested;
                // pairing those two rounds with a downward bias at the
                // entry layer skewed small inputs randomly low. `lroundf`
                // keeps the request as close as float allows; the planner
                // still owns the final rung pick.
                outSps = (resolved < 0.0f) ? 0u : static_cast<uint32_t>(::lroundf(resolved));
                return Status::Ok();
        }

        Status accelToSps2(Acceleration a, const MotorUnits &u, uint32_t cruiseSps,
                           uint32_t &outSps2)
        {
                const float v = static_cast<float>(a.value);
                float resolved = 0.0f;
                switch (a.unit) {
                case AccelUnit::StepsPerSecSquared:
                        resolved = v;
                        break;
                case AccelUnit::RpmPerSec:
                        if (u.stepsPerRevolution == 0) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = (v * static_cast<float>(u.stepsPerRevolution)) / 60.0f;
                        break;
                case AccelUnit::DegreesPerSecSquared:
                        if (u.stepsPerDegree == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * u.stepsPerDegree;
                        break;
                case AccelUnit::MmPerSecSquared:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * u.stepsPerMm;
                        break;
                case AccelUnit::RampMs:
                        // Ramp duration: a.value is ms from 0 to cruise.
                        // Needs both the cruise SPS (snapshot from the
                        // axis) and a non-zero duration.
                        if (cruiseSps == 0u || a.value == 0u) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = (static_cast<float>(cruiseSps) * 1000.0f) / v;
                        break;
                case AccelUnit::NoRamp:
                        // Explicit "no ramp" sentinel — planner takes the
                        // single-cruise-segment branch. `value` is ignored.
                        resolved = 0.0f;
                        break;
                }
                outSps2 = (resolved < 0.0f) ? 0u : static_cast<uint32_t>(::lroundf(resolved));
                return Status::Ok();
        }

        Status distanceToSteps(DistanceValue d, const MotorUnits &u, int32_t &outSteps)
        {
                const float v = static_cast<float>(d.value);
                float resolved = 0.0f;
                switch (d.unit) {
                case DistanceUnit::Steps:
                        resolved = v;
                        break;
                case DistanceUnit::Revolutions:
                        if (u.stepsPerRevolution == 0) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * static_cast<float>(u.stepsPerRevolution);
                        break;
                case DistanceUnit::Degrees:
                        if (u.stepsPerDegree == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * u.stepsPerDegree;
                        break;
                case DistanceUnit::Mm:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * u.stepsPerMm;
                        break;
                case DistanceUnit::Cm:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        resolved = v * 10.0f * u.stepsPerMm;
                        break;
                case DistanceUnit::Um:
                        if (u.stepsPerMm == 0.0f) {
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        // 1 mm = 1000 um → steps = (um / 1000) × stepsPerMm.
                        resolved = (v * u.stepsPerMm) / 1000.0f;
                        break;
                }
                outSteps = static_cast<int32_t>(resolved);
                return Status::Ok();
        }

        // Reverse: steps → host-facing unit (used by `position()`).
        float stepsToUnit(int32_t steps, DistanceUnit unit, const MotorUnits &u)
        {
                const float s = static_cast<float>(steps);
                switch (unit) {
                case DistanceUnit::Steps:
                        return s;
                case DistanceUnit::Revolutions:
                        return (u.stepsPerRevolution == 0) ?
                                   s :
                                   s / static_cast<float>(u.stepsPerRevolution);
                case DistanceUnit::Degrees:
                        return (u.stepsPerDegree == 0.0f) ? s : s / u.stepsPerDegree;
                case DistanceUnit::Mm:
                        return (u.stepsPerMm == 0.0f) ? s : s / u.stepsPerMm;
                case DistanceUnit::Cm:
                        return (u.stepsPerMm == 0.0f) ? s : (s / u.stepsPerMm) / 10.0f;
                case DistanceUnit::Um:
                        return (u.stepsPerMm == 0.0f) ? s : (s / u.stepsPerMm) * 1000.0f;
                }
                return s;
        }

} // namespace

// =====================================================================
// MotorAxis — ctor / dtor
// =====================================================================

MotorAxis::MotorAxis(MotorAxisConfig cfg, IMotorDriver &driver, ILimitSystem *limits,
                     IHomingStrategy *homing)
        // Order MUST match the declaration order in motor_axis.h —
        // ESP-IDF builds with -Werror=reorder so a mismatch breaks
        // the build outright.
        : activeProfile_(cfg.profile)
        , activeIntent_(cfg.intent)
        , cfg_(cfg)
        , driver_(driver)
        , limits_(limits)
        , homing_(homing)
{
}

MotorAxis::~MotorAxis() = default;

// =====================================================================
// Internal helpers
// =====================================================================

Status MotorAxis::resolveSpeedToSps(Speed s, uint32_t &outSps) const
{
        return speedToSps(s, cfg_.units, outSps);
}

Status MotorAxis::resolveDistanceToSteps(DistanceValue d, int32_t &outSteps) const
{
        return distanceToSteps(d, cfg_.units, outSteps);
}

Status MotorAxis::resolveAccelToSps2(Acceleration a, uint32_t cruiseSps,
                                     uint32_t &outSps2) const
{
        return accelToSps2(a, cfg_.units, cruiseSps, outSps2);
}

void MotorAxis::emit(MotorEventType type, StopReason reason, FaultCode fault)
{
        if (listener_ == nullptr) {
                return;
        }
        MotorEvent ev;
        ev.axisId = cfg_.id;
        ev.type = type;
        ev.state = state_;
        ev.commandedPosition = driver_.commandedPositionSteps();
        ev.stopReason = reason;
        ev.faultCode = fault;
        listener_->onMotorEvent(ev);
}

void MotorAxis::transition(MotorState newState)
{
        if (newState == state_) {
                return;
        }
        state_ = newState;
        emit(MotorEventType::StateChanged);
}

void MotorAxis::applyIntentToDriver()
{
        // Profile intent overrides axis intent when non-default.
        const MotorIntent effective =
            (activeProfile_.intent != MotorIntent::Default) ? activeProfile_.intent : activeIntent_;
        (void)driver_.applyIntent(effective);
}

// =====================================================================
// Lifecycle
// =====================================================================

Status MotorAxis::begin()
{
        if (state_ != MotorState::Uninitialized) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }

        // Resolve the configured limits + profile into the SPS domain.
        // Profile values override axis-level limits when non-zero.
        const Speed cruiseSrc = (activeProfile_.maxSpeed.value != 0u) ? activeProfile_.maxSpeed :
                                                                        cfg_.limits.maxSpeed;
        const Acceleration accelSrc = (activeProfile_.accel.value != 0u) ? activeProfile_.accel :
                                                                           cfg_.limits.accel;
        const Acceleration decelSrc = (activeProfile_.decel.value != 0u) ? activeProfile_.decel :
                                                                           cfg_.limits.decel;

        Status r = resolveSpeedToSps(cruiseSrc, resolvedCruiseSps_);
        if (!r.ok()) {
                return r;
        }
        r = resolveAccelToSps2(accelSrc, resolvedCruiseSps_, resolvedAccelSps2_);
        if (!r.ok()) {
                return r;
        }
        r = resolveAccelToSps2(decelSrc, resolvedCruiseSps_, resolvedDecelSps2_);
        if (!r.ok()) {
                return r;
        }
        cachedHardCeiling_ = cfg_.limits.hardStepRateCeilingSps;

        // Clamp cruise against the hard ceiling.
        if (cachedHardCeiling_ != 0u && resolvedCruiseSps_ > cachedHardCeiling_) {
                resolvedCruiseSps_ = cachedHardCeiling_;
        }
        // Cruise must be non-zero (a zero-cruise motion can't progress);
        // accel and decel are allowed at zero - the planner reads that
        // as "no ramp, instant cruise."
        if (resolvedCruiseSps_ == 0u) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Bring the driver up.
        r = driver_.begin();
        if (!r.ok()) {
                return r;
        }

        // Wire the limit system if one was provided. The host hands the
        // axis a pointer; if `nullptr`, the axis silently runs without
        // limit awareness (matches FS-UV / quick-bring-up sketches).
        if (limits_ != nullptr) {
                // Limit system needs a step-signal generator reference
                // for ISR-driven halts. We don't have direct access to
                // the generator here (the driver owns/borrows it). The
                // driver is responsible for wiring the limit system to
                // its generator if it has one. For Stage 1 the limit
                // system is initialised by the host BEFORE constructing
                // the axis; we just check it has been begun.
        }

        applyIntentToDriver();
        transition(MotorState::Disabled);
        return Status::Ok();
}

Status MotorAxis::enable()
{
        if (state_ == MotorState::Uninitialized) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (state_ == MotorState::Faulted || state_ == MotorState::EmergencyStopped) {
                return Status::Err(ErrorCode::DriverFault);
        }
        if (state_ != MotorState::Disabled && state_ != MotorState::Idle) {
                return Status::Err(ErrorCode::InvalidState);
        }
        const auto s = driver_.enable();
        if (!s.ok()) {
                return s;
        }
        transition(MotorState::Idle);
        return Status::Ok();
}

Status MotorAxis::disable()
{
        if (state_ == MotorState::Uninitialized) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (state_ == MotorState::Moving || state_ == MotorState::Jogging ||
            state_ == MotorState::Homing) {
                (void)stop();
        }
        const auto s = driver_.disable();
        if (!s.ok()) {
                return s;
        }
        transition(MotorState::Disabled);
        return Status::Ok();
}

Status MotorAxis::clearFault()
{
        if (state_ != MotorState::Faulted && state_ != MotorState::EmergencyStopped) {
                return Status::Err(ErrorCode::InvalidState);
        }
        const auto s = driver_.clearFault();
        if (!s.ok()) {
                return s;
        }
        if (limits_ != nullptr) {
                limits_->resetStallHitsTotal();
                // Drain any pending latches so they don't immediately
                // re-fire on the next motion command.
                (void)limits_->consumeEmergencyActivation();
                (void)limits_->consumeStallActivation();
        }
        lastFault_ = FaultCode::None;
        lastStopReason_ = StopReason::None;
        transition(MotorState::Disabled);
        emit(MotorEventType::FaultCleared);
        return Status::Ok();
}

// =====================================================================
// Motion commands
// =====================================================================

Status MotorAxis::armMove(Direction dir, uint32_t targetSteps)
{
        // A home switch is a hard limit at the home end too — refuse to drive INTO
        // either a travel limit or the home sensor that guards this direction.
        if (limits_ != nullptr && (limits_->isActive(LimitKind::TravelLimit, dir) ||
                                   limits_->isActive(LimitKind::HomeSensor, dir))) {
                return Status::Err(ErrorCode::LimitActive);
        }
        const auto s = driver_.armMove(dir, targetSteps, resolvedCruiseSps_, resolvedAccelSps2_,
                                       resolvedDecelSps2_);
        if (!s.ok()) {
                return s;
        }
        activeDirection_ = dir;
        motionInFlight_ = true;
        if (limits_ != nullptr) {
                // Use wall-clock time to arm the limit-system window.
                limits_->notifyMotionStart(ungula::core::time::millis());
        }
        transition(MotorState::Moving);
        emit(MotorEventType::MotionStarted);
        return Status::Ok();
}

Status MotorAxis::armJog(Direction dir)
{
        // See armMove: the home sensor blocks a jog into it the same as a travel
        // limit. (Homing never trips this — its strategy short-circuits to success
        // when it starts already on the home sensor, before arming a jog.)
        if (limits_ != nullptr && (limits_->isActive(LimitKind::TravelLimit, dir) ||
                                   limits_->isActive(LimitKind::HomeSensor, dir))) {
                return Status::Err(ErrorCode::LimitActive);
        }
        const auto s = driver_.armJog(dir, resolvedCruiseSps_, resolvedAccelSps2_);
        if (!s.ok()) {
                return s;
        }
        activeDirection_ = dir;
        motionInFlight_ = true;
        if (limits_ != nullptr) {
                limits_->notifyMotionStart(ungula::core::time::millis());
        }
        // Preserve Homing - the homing strategy uses jog verbs to
        // drive the axis while the axis is supposed to report
        // `Homing` to the host. Falling through to `Jogging` would
        // hide the homing phase from listeners and break the
        // service-tick path that ticks the strategy.
        if (state_ != MotorState::Homing) {
                transition(MotorState::Jogging);
        }
        emit(MotorEventType::MotionStarted);
        return Status::Ok();
}

Status MotorAxis::moveForward()
{
        // Idle is the normal arming state; Homing is allowed because a
        // homing strategy needs to arm a jog AFTER `home()` has already
        // transitioned the axis into Homing. The other motion verbs
        // (moveTo / moveBy) intentionally stay strict-Idle - the homing
        // strategy uses jog verbs.
        if (state_ != MotorState::Idle && state_ != MotorState::Homing) {
                return Status::Err(ErrorCode::InvalidState);
        }
        return armJog(Direction::Forward);
}

Status MotorAxis::moveBackward()
{
        // See moveForward() for the Homing carve-out.
        if (state_ != MotorState::Idle && state_ != MotorState::Homing) {
                return Status::Err(ErrorCode::InvalidState);
        }
        return armJog(Direction::Backward);
}

Status MotorAxis::moveTo(DistanceValue target)
{
        if (state_ != MotorState::Idle) {
                return Status::Err(ErrorCode::InvalidState);
        }
        int32_t targetSteps = 0;
        const auto r = resolveDistanceToSteps(target, targetSteps);
        if (!r.ok()) {
                return r;
        }
        const Position current = driver_.commandedPositionSteps();
        if (targetSteps == current) {
                // Zero-distance move: report completion immediately
                // without arming.
                lastStopReason_ = StopReason::TargetReached;
                emit(MotorEventType::MotionCompleted, StopReason::TargetReached);
                return Status::Ok();
        }
        const Direction dir = (targetSteps > current) ? Direction::Forward : Direction::Backward;
        const uint32_t steps = static_cast<uint32_t>(
            (targetSteps > current) ? (targetSteps - current) : (current - targetSteps));
        return armMove(dir, steps);
}

Status MotorAxis::moveBy(DistanceValue delta)
{
        if (state_ != MotorState::Idle) {
                return Status::Err(ErrorCode::InvalidState);
        }
        int32_t deltaSteps = 0;
        const auto r = resolveDistanceToSteps(delta, deltaSteps);
        if (!r.ok()) {
                return r;
        }
        if (deltaSteps == 0) {
                lastStopReason_ = StopReason::TargetReached;
                emit(MotorEventType::MotionCompleted, StopReason::TargetReached);
                return Status::Ok();
        }
        const Direction dir = (deltaSteps > 0) ? Direction::Forward : Direction::Backward;
        const uint32_t steps = static_cast<uint32_t>(deltaSteps > 0 ? deltaSteps : -deltaSteps);
        return armMove(dir, steps);
}

Status MotorAxis::home()
{
        if (state_ != MotorState::Idle) {
                return Status::Err(ErrorCode::InvalidState);
        }
        if (homing_ == nullptr) {
                return Status::Err(ErrorCode::Unsupported);
        }
        transition(MotorState::Homing);
        const auto s = homing_->start(*this);
        if (!s.ok()) {
                transition(MotorState::Idle);
                return s;
        }
        emit(MotorEventType::MotionStarted);
        return Status::Ok();
}

Status MotorAxis::stop()
{
        if (state_ == MotorState::Idle || state_ == MotorState::Disabled) {
                return Status::Ok();
        }
        if (state_ != MotorState::Moving && state_ != MotorState::Jogging &&
            state_ != MotorState::Homing && state_ != MotorState::Stopping) {
                return Status::Err(ErrorCode::InvalidState);
        }
        // A manual stop during homing ABORTS the homing strategy. Without this the
        // strategy stays active across the Stopping->Idle settle and fights the next
        // home()/jog (the axis looked Idle to the host but kept getting wedged).
        if (homing_ != nullptr && homing_->isActive()) {
                homing_->cancel();
        }
        const auto s = driver_.stop(StopMode::Decelerate);
        // If the driver doesn't support a decel ramp, fall back to
        // immediate stop. We do not pretend the decel happened.
        if (!s.ok() && s.error() == ErrorCode::Unsupported) {
                (void)driver_.stop(StopMode::Immediate);
        }
        if (!motionInFlight_) {
                // Nothing is actually in flight (homing that short-circuited on an
                // already-active sensor, or a motion that just settled). pumpMotion
                // only drives the Stopping->Idle edge while a motion was running, so
                // routing through Stopping here would wedge the axis. Go straight to
                // Idle.
                transition(MotorState::Idle);
        } else if (state_ != MotorState::Stopping) {
                transition(MotorState::Stopping);
        }
        return Status::Ok();
}

Status MotorAxis::softStop()
{
        if (state_ == MotorState::Idle || state_ == MotorState::Disabled) {
                return Status::Ok();
        }
        if (state_ != MotorState::Moving && state_ != MotorState::Jogging &&
            state_ != MotorState::Homing && state_ != MotorState::Stopping) {
                return Status::Err(ErrorCode::InvalidState);
        }
        // A soft stop during homing aborts the search the same way stop() does —
        // the rampdown replaces the in-flight move and the strategy must not
        // re-arm over it.
        if (homing_ != nullptr && homing_->isActive()) {
                homing_->cancel();
        }
        // Stop in a QUARTER of the acceleration ramp TIME. Acceleration is stored
        // as a RATE (steps/s²); for the same cruise speed, a quarter of the ramp
        // time means FOUR TIMES the rate (rate ∝ 1/time). So a heavy load can still
        // ramp UP gently while the stop just takes the edge off the clunk quickly.
        // If a distinct decel rate WAS configured (resolvedDecelSps2_ != accel),
        // honour it as-is. Zero accel (no-ramp config) → nothing to scale → hard stop.
        const bool hasExplicitDecel =
            resolvedDecelSps2_ != 0u && resolvedDecelSps2_ != resolvedAccelSps2_;
        const uint64_t quarterTimeRate = static_cast<uint64_t>(resolvedAccelSps2_) * 4u;
        const uint32_t fastDecel =
            (quarterTimeRate > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(quarterTimeRate);
        const uint32_t decel = hasExplicitDecel ? resolvedDecelSps2_ : fastDecel;
        const auto s = (decel > 0u) ? driver_.decelStop(decel) : driver_.stop(StopMode::Immediate);
        if (!motionInFlight_) {
                // Nothing actually in flight — go straight to Idle (mirrors stop()).
                transition(MotorState::Idle);
        } else if (state_ != MotorState::Stopping) {
                transition(MotorState::Stopping);
        }
        return s;
}

Status MotorAxis::emergencyStop()
{
        if (homing_ != nullptr && homing_->isActive()) {
                homing_->cancel();
        }
        const auto s = driver_.stop(StopMode::Immediate);
        (void)s; // we transition regardless of driver outcome
        if (limits_ != nullptr) {
                limits_->notifyMotionEnd();
        }
        motionInFlight_ = false;
        lastStopReason_ = StopReason::EmergencyStop;
        lastFault_ = FaultCode::Other;
        transition(MotorState::EmergencyStopped);
        emit(MotorEventType::EmergencyStopped, StopReason::EmergencyStop, FaultCode::Other);
        return Status::Ok();
}

// =====================================================================
// Runtime tuning
// =====================================================================

Status MotorAxis::setSpeed(Speed s)
{
        uint32_t sps = 0;
        const auto r = resolveSpeedToSps(s, sps);
        if (!r.ok()) {
                return r;
        }
        // Zero cruise SPS makes the planner emit an empty PlannedMove
        // and the driver fail the next arm with InvalidConfig; reject
        // here so the host sees the bad value at the setter call,
        // not three layers down.
        if (sps == 0u) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (cachedHardCeiling_ != 0u && sps > cachedHardCeiling_) {
                sps = cachedHardCeiling_;
        }
        resolvedCruiseSps_ = sps;
        return Status::Ok();
}

Status MotorAxis::setAcceleration(Acceleration a)
{
        uint32_t sps2 = 0;
        // Resolve against the live cruise so `Acceleration::rampMs(...)`
        // means "ramp from 0 to the speed the axis is currently using".
        const auto r = resolveAccelToSps2(a, resolvedCruiseSps_, sps2);
        if (!r.ok()) {
                return r;
        }
        // Zero is allowed and means "no ramp" - the planner will
        // emit a single cruise segment, the motor jumps to cruise
        // instantly. Safe at low SPS (jog at a few RPM), expect lost
        // steps if the host pairs accel=0 with a high cruise rate.
        resolvedAccelSps2_ = sps2;
        resolvedDecelSps2_ = sps2;
        return Status::Ok();
}

Status MotorAxis::setProfile(MotionProfile p)
{
        activeProfile_ = p;
        // Reapply resolutions if the profile carries an override. Three
        // encodings per knob:
        //   value > 0                → "use this value"
        //   value == 0, unit != NoRamp → "no override, leave current alone"
        //   unit == NoRamp           → "explicitly disable ramping"
        // The NoRamp sentinel exists because the `value == 0` encoding
        // is reserved for "skip" — without it, hosts had no way to
        // express "I want zero accel" through `MotionProfile`.
        if (p.maxSpeed.value != 0u) {
                const auto r = setSpeed(p.maxSpeed);
                if (!r.ok()) {
                        return r;
                }
        }
        if (p.accel.unit == AccelUnit::NoRamp || p.accel.value != 0u) {
                const auto r = setAcceleration(p.accel);
                if (!r.ok()) {
                        return r;
                }
        }
        // Profile intent applies via applyIntentToDriver().
        applyIntentToDriver();
        return Status::Ok();
}

Status MotorAxis::setIntent(MotorIntent intent)
{
        activeIntent_ = intent;
        applyIntentToDriver();
        return Status::Ok();
}

// =====================================================================
// Service tick
// =====================================================================

void MotorAxis::pumpLimits(int64_t nowMs)
{
        if (limits_ == nullptr) {
                return;
        }
        limits_->service(nowMs);

        // ISR-latched events are consumed and routed by precedence:
        // Emergency > Stall > TravelLimit.
        if (limits_->consumeEmergencyActivation()) {
                motionInFlight_ = false;
                lastStopReason_ = StopReason::EmergencyStop;
                lastFault_ = FaultCode::Other;
                if (state_ != MotorState::EmergencyStopped) {
                        transition(MotorState::EmergencyStopped);
                        emit(MotorEventType::EmergencyStopped, StopReason::EmergencyStop,
                             FaultCode::Other);
                }
                return;
        }
        if (limits_->consumeStallActivation()) {
                motionInFlight_ = false;
                // Close the stall-arm window. Without this, `stallArmedAtMs_`
                // stays set from the just-stopped motion; after the host
                // calls clearFault() the polled-level path immediately
                // re-latches because DIAG is still HIGH and the arm
                // window appears expired. Matches the symmetry the
                // TravelLimit path uses a few lines below.
                limits_->notifyMotionEnd();
                lastStopReason_ = StopReason::StallDetected;
                lastFault_ = FaultCode::Stall;
                if (state_ != MotorState::Faulted) {
                        transition(MotorState::Faulted);
                        emit(MotorEventType::LimitActivated, StopReason::StallDetected,
                             FaultCode::Stall);
                        emit(MotorEventType::FaultRaised, StopReason::StallDetected,
                             FaultCode::Stall);
                }
                return;
        }
        // Polled limit halt if active in the in-flight direction. The home sensor
        // counts as a hard limit for any NON-homing motion (manual jog, section
        // move) — its whole reason to exist is to stop the carriage at the home end
        // and prevent damage; homing is excluded because its own strategy issues
        // the stop and resets position.
        const bool travelHit = limits_->isActive(LimitKind::TravelLimit, activeDirection_);
        const bool homeHit = state_ != MotorState::Homing &&
                             limits_->isActive(LimitKind::HomeSensor, activeDirection_);
        if (motionInFlight_ && (travelHit || homeHit)) {
                (void)driver_.stop(StopMode::Immediate);
                motionInFlight_ = false;
                limits_->notifyMotionEnd();
                lastStopReason_ = StopReason::TravelLimit;
                transition(MotorState::Idle);
                emit(MotorEventType::LimitActivated, StopReason::TravelLimit);
                emit(MotorEventType::MotionStopped, StopReason::TravelLimit);
        }
}

void MotorAxis::pumpMotion()
{
        if (!motionInFlight_) {
                return;
        }
        const auto st = driver_.motionStatus();
        if (st.running) {
                return;
        }
        motionInFlight_ = false;
        if (limits_ != nullptr) {
                limits_->notifyMotionEnd();
        }
        lastStopReason_ = st.finishedReason;

        if (st.faulted) {
                if (state_ != MotorState::EmergencyStopped && state_ != MotorState::Faulted) {
                        lastFault_ = FaultCode::PulseEngineFault;
                        transition(MotorState::Faulted);
                        emit(MotorEventType::FaultRaised, st.finishedReason,
                             FaultCode::PulseEngineFault);
                }
                return;
        }
        // Natural completion or stop().
        if (state_ == MotorState::Moving || state_ == MotorState::Jogging ||
            state_ == MotorState::Stopping) {
                transition(MotorState::Idle);
                if (st.finishedReason == StopReason::TargetReached) {
                        emit(MotorEventType::MotionCompleted, st.finishedReason);
                } else {
                        emit(MotorEventType::MotionStopped, st.finishedReason);
                }
        }
}

void MotorAxis::service(int64_t nowMs)
{
        if (state_ == MotorState::Uninitialized) {
                return;
        }
        pumpLimits(nowMs);
        pumpMotion();

        // Tick the homing strategy whenever it's active, regardless of
        // the current axis state. The strategy may call axis.stop()
        // which transitions Homing -> Stopping -> Idle on the way to
        // completion; gating only on `state == Homing` would stop
        // ticking the strategy mid-flight and the post-process below
        // (resetPosition / HomingCompleted) would never fire.
        if (homing_ != nullptr && homing_->isActive()) {
                homing_->tick(*this, nowMs);
                if (!homing_->isActive()) {
                        if (homing_->succeeded()) {
                                // Declare the current position to be the
                                // home reference.
                                (void)driver_.resetPosition(0);
                                homed_ = true;
                                lastStopReason_ = StopReason::TargetReached;
                                transition(MotorState::Idle);
                                emit(MotorEventType::HomingCompleted);
                        } else {
                                lastStopReason_ = homing_->failureReason();
                                transition(MotorState::Idle);
                                emit(MotorEventType::HomingFailed, lastStopReason_);
                        }
                }
        }
}

// =====================================================================
// Queries
// =====================================================================

Position MotorAxis::positionSteps() const
{
        return driver_.commandedPositionSteps();
}

float MotorAxis::position(DistanceUnit unit) const
{
        return stepsToUnit(driver_.commandedPositionSteps(), unit, cfg_.units);
}

uint32_t MotorAxis::currentSps() const
{
        return driver_.commandedSpsNow();
}

uint32_t MotorAxis::actualCruiseSps() const
{
        const uint32_t res = driver_.timerResolutionHz();
        if (res == 0u) {
                // Driver does not expose timer ticks (e.g. RMD CAN
                // servos plan internally). Best we can do is echo the
                // axis-resolved request.
                return resolvedCruiseSps_;
        }
        return TrapezoidalPlanner::actualSpsFor(resolvedCruiseSps_, res);
}

DriverIdentity MotorAxis::identity()
{
        return driver_.identity();
}

MotorDiagnostics MotorAxis::diagnostics() const
{
        MotorDiagnostics d;
        d.axisId = cfg_.id;
        d.state = state_;
        d.commandedPosition = driver_.commandedPositionSteps();
        d.currentSps = driver_.commandedSpsNow();
        d.targetSps = resolvedCruiseSps_;
        d.stepsToTarget = 0; // driver-owned; filled below
        d.lastStopReason = lastStopReason_;
        d.lastFault = lastFault_;
        d.totalStepsIssued = 0; // driver-owned; filled below
        d.homed = homed_;
        // Driver fills identity + optional fields + totals.
        driver_.fillDriverDiagnostics(d);
        return d;
}

// =====================================================================
// Events
// =====================================================================

void MotorAxis::subscribe(IMotorEventListener &listener)
{
        listener_ = &listener;
}

void MotorAxis::unsubscribe()
{
        listener_ = nullptr;
}

} // namespace ungula::motor
