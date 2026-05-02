// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "local_motor.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <time/time_control.h>
#include <cassert>

namespace motor {

    // Spinlock protecting shared mutable state between the service timer
    // (esp_timer task) and caller context (motion commands from any task).
    // Protects: moveTarget_, hasTarget_, decelerating_, pendingProfile_,
    // hasPendingProfile_. Keep critical sections short — no blocking calls.
    static portMUX_TYPE g_motorMux = portMUX_INITIALIZER_UNLOCKED;

    // ---- Internal service timer trampoline ----
    // Runs in esp_timer task context (not ISR), safe for FSM and callbacks.

    void LocalMotor::onServiceTimer(void* arg) {
        static_cast<LocalMotor*>(arg)->handleServiceTimer();
    }

    // ============================================================
    // Wiring
    // ============================================================

    void LocalMotor::setDriver(IMotorDriver& driver) {
        driver_ = &driver;
    }

    void LocalMotor::addLimitBackward(uint8_t pin) {
        if (backwardLimitCount_ < limit::MAX_PER_DIRECTION) {
            limitsBackward_[backwardLimitCount_].configure(pin);
            backwardLimitCount_++;
        }
    }

    void LocalMotor::addLimitForward(uint8_t pin) {
        if (forwardLimitCount_ < limit::MAX_PER_DIRECTION) {
            limitsForward_[forwardLimitCount_].configure(pin);
            forwardLimitCount_++;
        }
    }

    void LocalMotor::setHomingStrategy(IHomingStrategy* strategy) {
        homingStrategy_ = strategy;
    }

    void LocalMotor::setHomingTimeout(int64_t timeoutMs) {
        homingTimeoutMs_ = timeoutMs;
    }

    bool LocalMotor::subscribe(IMotorEventListener* listener) {
        return eventPublisher_.subscribe(listener);
    }

    bool LocalMotor::unsubscribe(IMotorEventListener* listener) {
        return eventPublisher_.unsubscribe(listener);
    }

    // ============================================================
    // Configuration
    // ============================================================

    void LocalMotor::setAutoStopOnStall(bool enabled) {
        autoStopOnStall_ = enabled;
    }

    void LocalMotor::setMicrosteps(uint16_t microsteps) {
        if (driver_ != nullptr) {
            driver_->setMicrosteps(microsteps);
        }
    }

    void LocalMotor::setRunCurrent(uint16_t milliAmps) {
        if (driver_ != nullptr) {
            driver_->setRunCurrent(milliAmps);
        }
    }

    void LocalMotor::setCurrentCurve(const CurrentCurve& curve) {
        currentCurve_ = curve;
    }

    void LocalMotor::setCurrentCurveEnabled(bool enabled) {
        currentCurveEnabled_ = enabled;
    }

    void LocalMotor::setStepsPerMm(float stepsPerMm) {
        stepsPerMm_ = stepsPerMm;
    }

    void LocalMotor::setStepsPerDegree(float stepsPerDeg) {
        stepsPerDeg_ = stepsPerDeg;
    }

    void LocalMotor::setProfileSpeed(MotionProfile profile, int32_t speedSps) {
        profiles_[static_cast<uint8_t>(profile)].speedSps = speedSps;
    }

    void LocalMotor::setProfileSpeed(MotionProfile profile, SpeedValue speed) {
        profiles_[static_cast<uint8_t>(profile)].speedSps = convertToSps(speed);
    }

    void LocalMotor::setProfileAccel(MotionProfile profile, uint32_t accelMs) {
        profiles_[static_cast<uint8_t>(profile)].accelTimeMs = accelMs;
    }

    void LocalMotor::setProfileDecel(MotionProfile profile, uint32_t decelMs) {
        profiles_[static_cast<uint8_t>(profile)].decelTimeMs = decelMs;
    }

    void LocalMotor::setActiveProfile(MotionProfile profile) {
        activeProfile_ = profile;
    }

    // ============================================================
    // Lifecycle
    // ============================================================

    bool LocalMotor::begin() {
        assert(driver_ != nullptr);

        // Seed every profile that the user did not configure with a safe
        // default — low speed, smooth ramp. Picks a value only when the
        // slot is still zero, so explicit setProfile*() calls before
        // begin() are preserved. The numbers are conservative: 500 SPS
        // with a 500 ms ramp rarely damages anything and is visible to
        // the eye, which is what a "first run" project wants.
        for (int32_t i = 0; i < PROFILE_COUNT; ++i) {
            ProfileConfig& p = profiles_[i];
            if (p.speedSps == 0) {
                p.speedSps = 500;
            }
            if (p.accelTimeMs == 0U) {
                p.accelTimeMs = 500U;
            }
            if (p.decelTimeMs == 0U) {
                p.decelTimeMs = 500U;
            }
        }

        // Initialize motor driver — configures GPIO pins and register defaults
        driver_->begin();

        // Step generator — creates timer and configures step pin
        if (!stepper_.begin(driver_->stepPin())) {
            return false;
        }

        // Wire FSM to event publisher and position source for event stamping
        fsm_.setPublisher(&eventPublisher_);
        fsm_.setPositionSource(&cachedPosition_);

        // Internal service timer — periodic 10 ms callback for safety monitoring.
        // Runs in esp_timer task context (not ISR), so FSM transitions and
        // callbacks are safe. Replaces the external service() call entirely.
        esp_timer_create_args_t timerArgs = {};
        timerArgs.callback = &LocalMotor::onServiceTimer;
        timerArgs.arg = this;
        timerArgs.name = "motor_svc";
        timerArgs.dispatch_method = ESP_TIMER_TASK;

        esp_timer_handle_t svcTimer = nullptr;
        if (esp_timer_create(&timerArgs, &svcTimer) != ESP_OK) {
            return false;
        }
        serviceTimer_ = svcTimer;

        if (esp_timer_start_periodic(svcTimer, svc::MOTOR_SERVICE_US) != ESP_OK) {
            esp_timer_delete(svcTimer);
            serviceTimer_ = nullptr;
            return false;
        }

        // Seed isHomed_ from the configured strategy. For limit-switch
        // homing, this means "if the axis happens to be sitting on the
        // switch after a reboot, we already know we're at home". Stall
        // strategies have no such signal and always return false.
        if (homingStrategy_ != nullptr) {
            isHomed_ = homingStrategy_->isAtHomeReference(*this);
        }

        return true;
    }

    void LocalMotor::end() {
        // Stop service timer
        if (serviceTimer_ != nullptr) {
            esp_timer_handle_t svcTimer = static_cast<esp_timer_handle_t>(serviceTimer_);
            esp_timer_stop(svcTimer);
            esp_timer_delete(svcTimer);
            serviceTimer_ = nullptr;
        }

        // Stop step generator timers
        stepper_.end();
    }

    // ============================================================
    // IMotor — enable / disable
    // ============================================================

    // Helper used at every motion-end transition and at idle: implements
    // the per-host idle policy chosen with LocalMotor::setIdleMode().
    static inline void parkDriverForIdle(motor::IMotorDriver* drv,
                                         motor::LocalMotor::IdleMode mode) {
        if (mode == motor::LocalMotor::IdleMode::AutoDisable) {
            drv->disable();
        }
        // HoldCurrent: leave EN low so coils stay energised at IHOLD.
    }

    void LocalMotor::enable() {
        if (fsm_.state() != MotorFsmState::Disabled) {
            return;
        }
        // Idle EN-pin policy is host-configurable via setIdleMode().
        // AutoDisable (default, matches OLD basic_motor): EN HIGH at idle.
        // HoldCurrent: EN LOW at idle so the chip holds at IHOLD.
        if (idleMode_ == IdleMode::AutoDisable) {
            parkDriverForIdle(driver_, idleMode_);
        } else {
            driver_->enable();
        }
        fsm_.requestEnable();
        // Pre-park current at the curve floor so when motion starts
        // the first applyCurrentForSpeed sees a known IRUN/IHOLD.
        applyCurrentForSpeed(0);
    }

    void LocalMotor::disable() {
        abortHomingIfRunning();
        stepper_.hardStop();
        portENTER_CRITICAL(&g_motorMux);
        hasTarget_ = false;
        hasPendingProfile_ = false;
        decelerating_ = false;
        portEXIT_CRITICAL(&g_motorMux);
        driver_->disable();
        fsm_.requestDisable();
        lastStopReason_ = StopReason::Disabled;
    }

    // ============================================================
    // IMotor — motion commands
    // ============================================================

    void LocalMotor::moveForward() {
        if (fsm_.state() != MotorFsmState::Idle) {
            return;
        }
        invalidateHomedIfUserMotion();
        lastStopReason_ = StopReason::None;
        direction_ = Direction::FORWARD;
        applyDirection();
        stepper_.clearTargetPosition();  // Continuous — no ISR target
        auto& profile = profiles_[static_cast<uint8_t>(activeProfile_)];
        startMotion(profile.speedSps, profile.accelTimeMs, profile.decelTimeMs);
        fsm_.requestMoveForward();
        fsm_.requestRunning(Direction::FORWARD);
    }

    void LocalMotor::moveBackward() {
        if (fsm_.state() != MotorFsmState::Idle) {
            return;
        }
        invalidateHomedIfUserMotion();
        lastStopReason_ = StopReason::None;
        direction_ = Direction::BACKWARD;
        applyDirection();
        stepper_.clearTargetPosition();  // Continuous — no ISR target
        auto& profile = profiles_[static_cast<uint8_t>(activeProfile_)];
        startMotion(profile.speedSps, profile.accelTimeMs, profile.decelTimeMs);
        fsm_.requestMoveBackward();
        fsm_.requestRunning(Direction::BACKWARD);
    }

    void LocalMotor::moveTo(float target, DistanceUnit unit) {
        moveToAbsolute(convertToSteps(target, unit));
    }

    void LocalMotor::moveToAbsolute(int32_t absoluteTarget) {
        if (fsm_.state() != MotorFsmState::Idle) {
            return;
        }
        int32_t currentPos = stepper_.position();
        if (absoluteTarget == currentPos) {
            return;
        }

        invalidateHomedIfUserMotion();
        lastStopReason_ = StopReason::None;

        portENTER_CRITICAL(&g_motorMux);
        moveTarget_ = absoluteTarget;
        hasTarget_ = true;
        decelerating_ = false;
        portEXIT_CRITICAL(&g_motorMux);

        Direction dir = (absoluteTarget > currentPos) ? Direction::FORWARD : Direction::BACKWARD;
        direction_ = dir;
        applyDirection();

        // Set ISR-level target for single-step precision stop.
        // The step ISR checks position on every step and stops immediately
        // when the target is reached — zero overshoot.
        stepper_.setTargetPosition(absoluteTarget);

        auto& profile = profiles_[static_cast<uint8_t>(activeProfile_)];
        startMotion(profile.speedSps, profile.accelTimeMs, profile.decelTimeMs);
        fsm_.requestStarting();
        fsm_.requestRunning(dir);
    }

    void LocalMotor::moveBy(float delta, DistanceUnit unit) {
        int32_t deltaSteps = convertToSteps(delta, unit);
        if (deltaSteps == 0) {
            return;
        }
        int32_t currentPos = stepper_.position();
        moveToAbsolute(currentPos + deltaSteps);
    }

    void LocalMotor::executeProfile(const MotionProfileSpec& profile) {
        if (fsm_.state() != MotorFsmState::Idle) {
            return;
        }

        invalidateHomedIfUserMotion();
        lastStopReason_ = StopReason::None;

        portENTER_CRITICAL(&g_motorMux);
        pendingProfile_ = profile;
        hasPendingProfile_ = true;
        portEXIT_CRITICAL(&g_motorMux);

        int64_t nowMs = ungula::TimeControl::syncNow();
        if (profile.startTimeMs == 0 || profile.startTimeMs <= nowMs) {
            servicePendingProfile(nowMs);
        } else {
            fsm_.requestWaitStart();
        }
    }

    void LocalMotor::updateSpeed(int32_t speedSps, uint32_t accelMs, uint32_t decelMs) {
        if (!fsm_.isMoving()) {
            return;
        }
        driver_->updateStallDetectionSpeed(speedSps);
        applyCurrentForSpeed(speedSps);
        stepper_.setSpeed(speedSps, accelMs, decelMs);
    }

    void LocalMotor::updateSpeed(SpeedValue speed, uint32_t accelMs, uint32_t decelMs) {
        updateSpeed(convertToSps(speed), accelMs, decelMs);
    }

    void LocalMotor::stop() {
        // A soft stop still counts as "stop homing" — the strategy can't
        // make progress if we're ramping to zero.
        abortHomingIfRunning();
        stepper_.stop();
        fsm_.requestDecelerate();
    }

    void LocalMotor::emergencyStop() {
        abortHomingIfRunning();
        stepper_.hardStop();
        stepper_.clearTargetPosition();
        portENTER_CRITICAL(&g_motorMux);
        hasTarget_ = false;
        hasPendingProfile_ = false;
        decelerating_ = false;
        portEXIT_CRITICAL(&g_motorMux);
        // Internal e-stop calls inside the homing strategy must not
        // overwrite the latched reason — homing's success/failure path
        // sets the reason later. Skip latching for those.
        fsm_.requestEmergencyStop();
        if (!internalMotionFromHoming_) {
            lastStopReason_ = StopReason::EmergencyStop;
        }
        applyCurrentForSpeed(0);
        parkDriverForIdle(driver_, idleMode_);
    }

    // ============================================================
    // IMotor — state queries
    // ============================================================

    MotorFsmState LocalMotor::state() const {
        return fsm_.state();
    }

    int32_t LocalMotor::positionSteps() const {
        return stepper_.position();
    }

    bool LocalMotor::isMoving() const {
        return fsm_.isMoving();
    }

    bool LocalMotor::isIdle() const {
        return fsm_.state() == MotorFsmState::Idle;
    }

    bool LocalMotor::isStalling() const {
        // Either the FSM is latched in Stall (waiting for clearStall) or
        // the driver is asserting stall right now. One getter, both views.
        if (fsm_.state() == MotorFsmState::Stall) {
            return true;
        }
        return (driver_ != nullptr) && driver_->isStalling();
    }

    StopReason LocalMotor::lastStopReason() const {
        return lastStopReason_;
    }

    bool LocalMotor::wasLimitHit() const {
        return wasLimitHit_;
    }

    bool LocalMotor::isLimitActive(Direction dir) const {
        return (dir == Direction::BACKWARD) ? isBackwardLimitHit() : isForwardLimitHit();
    }

    bool LocalMotor::isLimitActive(Direction dir, int32_t index) const {
        if (index < 0 || index >= limit::MAX_PER_DIRECTION) {
            return false;
        }
        if (dir == Direction::BACKWARD) {
            if (index >= backwardLimitCount_) {
                return false;
            }
            return limitsBackward_[index].isTriggered();
        }
        if (index >= forwardLimitCount_) {
            return false;
        }
        return limitsForward_[index].isTriggered();
    }

    int32_t LocalMotor::limitCount(Direction dir) const {
        return (dir == Direction::BACKWARD) ? backwardLimitCount_ : forwardLimitCount_;
    }

    bool LocalMotor::isHoming() const {
        return homingPhase_ == HomingPhase::Running;
    }

    bool LocalMotor::isHomed() const {
        return isHomed_;
    }

    // ============================================================
    // Position
    // ============================================================

    void LocalMotor::resetPosition() {
        if (fsm_.isMoving()) {
            return;
        }
        stepper_.resetPosition();
    }

    // ============================================================
    // Homing (internal black-box)
    // ============================================================

    void LocalMotor::home() {
        if (homingStrategy_ == nullptr) {
            return;  // nothing configured — home() is a no-op
        }
        // Cancel whatever was happening. This also aborts a previous home()
        // run cleanly (abortHomingIfRunning is called by emergencyStop).
        emergencyStop();
        // The cancellation above latches StopReason::EmergencyStop — wipe
        // it so the homing run starts with a clean "no reason yet" state.
        lastStopReason_ = StopReason::None;

        isHomed_ = false;
        homingPhase_ = HomingPhase::Running;
        homingStartMs_ = ungula::TimeControl::syncNow();
        // strategy.begin() will issue motion commands; mark them as
        // "internal" so isHomed_ isn't immediately invalidated by them.
        internalMotionFromHoming_ = true;
        homingStrategy_->begin(*this);
        internalMotionFromHoming_ = false;
    }

    void LocalMotor::abortHomingIfRunning() {
        if (homingPhase_ != HomingPhase::Running) {
            return;
        }
        // Re-entrancy guard. Strategies legally call motor.emergencyStop()
        // as part of their normal phase transitions (LimitReached → backoff,
        // TargetReached → slow re-approach). Those internal calls must NOT
        // abort the homing run — only user-initiated stop/emergencyStop do.
        if (internalMotionFromHoming_) {
            return;
        }
        // Set phase FIRST. strategy.finish() calls motor.emergencyStop() if the
        // motor is still moving, which re-enters this function — without the
        // early-out above + early phase flip below, we'd stack-overflow.
        homingPhase_ = HomingPhase::Failed;
        if (homingStrategy_ != nullptr) {
            internalMotionFromHoming_ = true;
            homingStrategy_->finish(*this, false);
            internalMotionFromHoming_ = false;
        }
    }

    void LocalMotor::invalidateHomedIfUserMotion() {
        if (!internalMotionFromHoming_) {
            isHomed_ = false;
        }
    }

    // Runs from handleServiceTimer — in-phase with FSM transitions so the
    // strategy sees TargetReached before the service loop's auto-clear
    // collapses it to Idle.
    void LocalMotor::serviceHoming(int64_t nowMs) {
        if (homingPhase_ != HomingPhase::Running || homingStrategy_ == nullptr) {
            return;
        }
        // Wall-clock guard (0 = disabled, strategy owns its own deadline).
        if (homingTimeoutMs_ != 0U && (nowMs - homingStartMs_) >= homingTimeoutMs_) {
            internalMotionFromHoming_ = true;
            homingStrategy_->finish(*this, false);
            internalMotionFromHoming_ = false;
            homingPhase_ = HomingPhase::Failed;
            return;
        }

        internalMotionFromHoming_ = true;
        const bool done = homingStrategy_->tick(*this);
        if (done) {
            const bool success = homingStrategy_->succeeded();
            homingStrategy_->finish(*this, success);
            homingPhase_ = success ? HomingPhase::Done : HomingPhase::Failed;
            if (success) {
                isHomed_ = true;
            }
        }
        internalMotionFromHoming_ = false;
    }

    // ============================================================
    // Fault acknowledgement
    // ============================================================

    void LocalMotor::clearStall() {
        lastStopReason_ = StopReason::None;
        driver_->clearStall();
        fsm_.clearStall();
    }

    void LocalMotor::clearFault() {
        fsm_.clearFault();
    }

    // ============================================================
    // Internal service timer (runs every ~10 ms via esp_timer)
    // ============================================================
    //
    // This replaces the old public service() method. It runs autonomously
    // in the esp_timer task, independent of the main application loop.
    // Handles: limit switches, stall detection, target position management,
    // auto-stop detection, and FSM terminal state transitions.

    void LocalMotor::handleServiceTimer() {
        // Snapshot position for FSM event stamping and service logic
        cachedPosition_ = stepper_.position();

        // Update limit switches (debounce polling)
        int64_t nowMs = ungula::TimeControl::syncNow();

        // Drive the step-pulse acceleration ramp from this single service
        // tick — keeps ramp updates and gptimer alarm reconfig in one task.
        // Avoids the cross-task race that produced chunky stepping when the
        // main loop was mid-ADC burst on RBB2.
        stepper_.serviceRamp(static_cast<uint32_t>(nowMs));
        for (int32_t idx = 0; idx < backwardLimitCount_; idx++) {
            limitsBackward_[idx].update(nowMs);
        }
        for (int32_t idx = 0; idx < forwardLimitCount_; idx++) {
            limitsForward_[idx].update(nowMs);
        }

        // Sticky limit-hit flag clear condition: we're actively moving
        // again AND every limit has released. This matches the spec —
        // wasLimitHit() stays true from the stop until the axis has
        // physically backed off the switch.
        if (wasLimitHit_ && fsm_.isMoving() && !isBackwardLimitHit() && !isForwardLimitHit()) {
            wasLimitHit_ = false;
        }

        MotorFsmState currentState = fsm_.state();

        // Snapshot shared flags under lock — callers may write from another context
        portENTER_CRITICAL(&g_motorMux);
        bool snapHasPending = hasPendingProfile_;
        bool snapDecelerating = decelerating_;
        portEXIT_CRITICAL(&g_motorMux);

        // Handle pending profile start time
        if (snapHasPending && currentState == MotorFsmState::WaitingStart) {
            servicePendingProfile(nowMs);
        }

        // Stall detection — delegated to driver. The driver handles pin reading,
        // register polling, blanking, and scoring. We only ask for the verdict.
        if (fsm_.isMoving() && driver_ != nullptr && !snapDecelerating) {
            driver_->serviceStallDetection();
        }

        // Safety checks during motion
        if (fsm_.isMoving()) {
            // Stall check — driver's verdict
            if (driver_ != nullptr && driver_->isStalling()) {
                handleStall();
                return;
            }
            // Limit switch detection
            if (isLimitHitInDirection()) {
                stepper_.hardStop();
                portENTER_CRITICAL(&g_motorMux);
                hasTarget_ = false;
                portEXIT_CRITICAL(&g_motorMux);
                fsm_.requestLimitHit();
                // Latch the sticky flag so the host can observe the stop
                // after the FSM's auto-clear has returned us to Idle.
                wasLimitHit_ = true;
                lastStopReason_ = StopReason::LimitHit;
                // Honour the host-configured idle policy.
                parkDriverForIdle(driver_, idleMode_);
                // Drive the homing strategy before returning — it may want
                // to act on this LimitReached event this very tick.
                serviceHoming(nowMs);
                return;
            }
            // Target position check + deceleration
            serviceMoving();
        }

        // Detect ISR-level target reached (single-step precision)
        if (stepper_.consumeTargetReached() && fsm_.isMoving()) {
            portENTER_CRITICAL(&g_motorMux);
            hasTarget_ = false;
            decelerating_ = false;
            portEXIT_CRITICAL(&g_motorMux);
            fsm_.requestTargetReached();
            lastStopReason_ = StopReason::TargetReached;
            // Match OLD basic_motor behaviour: disable EN at every motion
            // end so the chopper stops and coils freewheel — silent at
            // standstill, no heating.
            applyCurrentForSpeed(0);
            parkDriverForIdle(driver_, idleMode_);
        }

        // Detect stepper auto-stop (ramp reached zero inside ISR). This
        // path runs when stop() was called and the deceleration ramp has
        // finished — the user-stop completion case.
        if (stepper_.consumeAutoStop() && fsm_.isMoving()) {
            portENTER_CRITICAL(&g_motorMux);
            hasTarget_ = false;
            decelerating_ = false;
            portEXIT_CRITICAL(&g_motorMux);
            fsm_.requestStop();
            lastStopReason_ = StopReason::UserStop;
            applyCurrentForSpeed(0);
            parkDriverForIdle(driver_, idleMode_);
        }

        // Drive the homing strategy in-phase with the FSM transitions it
        // watches for — specifically TargetReached, which the auto-clear
        // below would otherwise collapse to Idle in the same tick before
        // the strategy has a chance to react.
        serviceHoming(nowMs);

        // Auto-clear of the soft terminal states. Stall / Fault are
        // intentionally NOT cleared here — those require an explicit
        // host acknowledgement.
        MotorFsmState afterState = fsm_.state();
        if (afterState == MotorFsmState::TargetReached ||
            afterState == MotorFsmState::LimitReached) {
            fsm_.requestStop();
        }
    }

    // ============================================================
    // Diagnostics
    // ============================================================

    float LocalMotor::currentSpeed() const {
        return stepper_.currentSpeed();
    }

    // ============================================================
    // Internal helpers
    // ============================================================

    void LocalMotor::applyDirection() {
        stepper_.setDirectionForward(direction_ == Direction::FORWARD);
        driver_->setDirection(direction_);
    }

    bool LocalMotor::isBackwardLimitHit() const {
        for (int32_t idx = 0; idx < backwardLimitCount_; idx++) {
            if (limitsBackward_[idx].isTriggered()) {
                return true;
            }
        }
        return false;
    }

    bool LocalMotor::isForwardLimitHit() const {
        for (int32_t idx = 0; idx < forwardLimitCount_; idx++) {
            if (limitsForward_[idx].isTriggered()) {
                return true;
            }
        }
        return false;
    }

    bool LocalMotor::isLimitHitInDirection() const {
        if (direction_ == Direction::BACKWARD) {
            return isBackwardLimitHit();
        }
        return isForwardLimitHit();
    }

    void LocalMotor::handleStall() {
        if (autoStopOnStall_) {
            stepper_.hardStop();
            portENTER_CRITICAL(&g_motorMux);
            hasTarget_ = false;
            portEXIT_CRITICAL(&g_motorMux);
        }
        fsm_.requestStallDetected();
        lastStopReason_ = StopReason::Stall;
    }

    void LocalMotor::serviceMoving() {
        // Snapshot shared state under lock
        portENTER_CRITICAL(&g_motorMux);
        bool snapHasTarget = hasTarget_;
        int32_t snapMoveTarget = moveTarget_;
        bool snapDecelerating = decelerating_;
        portEXIT_CRITICAL(&g_motorMux);

        if (!snapHasTarget) {
            return;
        }

        int32_t pos = cachedPosition_;
        int32_t remaining = (direction_ == Direction::FORWARD) ? (snapMoveTarget - pos)
                                                               : (pos - snapMoveTarget);

        // Hard limit — if we overshot, stop immediately
        if (remaining <= 0) {
            stepper_.hardStop();
            portENTER_CRITICAL(&g_motorMux);
            hasTarget_ = false;
            decelerating_ = false;
            portEXIT_CRITICAL(&g_motorMux);
            fsm_.requestTargetReached();
            lastStopReason_ = StopReason::TargetReached;
            return;
        }

        // Deceleration distance: d = v^2 / (2 * a)
        // Standard kinematics — how many steps it takes to ramp from current
        // speed down to zero at the configured deceleration rate.
        // When decelRate is 0 (instant stop), decelSteps is 0 — stop immediately at target.
        if (!snapDecelerating) {
            float currentSps = stepper_.currentSpeed();
            float decelRate = stepper_.decelerationRate();
            int32_t decelSteps = 0;
            if (decelRate > 0.0F) {
                decelSteps = static_cast<int32_t>((currentSps * currentSps) / (2.0F * decelRate));
            }

            if (remaining <= decelSteps) {
                stepper_.stop();  // Soft stop — ramp to zero
                fsm_.requestDecelerate();
                portENTER_CRITICAL(&g_motorMux);
                decelerating_ = true;
                portEXIT_CRITICAL(&g_motorMux);
            }
        }
    }

    void LocalMotor::servicePendingProfile(int64_t nowMs) {
        // Snapshot profile under lock — caller may write from another context
        portENTER_CRITICAL(&g_motorMux);
        if (!hasPendingProfile_) {
            portEXIT_CRITICAL(&g_motorMux);
            return;
        }
        MotionProfileSpec profile = pendingProfile_;
        hasPendingProfile_ = false;
        portEXIT_CRITICAL(&g_motorMux);

        if (profile.startTimeMs != 0 && nowMs < profile.startTimeMs) {
            // Not ready yet — put it back
            portENTER_CRITICAL(&g_motorMux);
            pendingProfile_ = profile;
            hasPendingProfile_ = true;
            portEXIT_CRITICAL(&g_motorMux);
            return;
        }

        int32_t currentPos = cachedPosition_;
        if (profile.targetPosition > currentPos) {
            direction_ = Direction::FORWARD;
        } else {
            direction_ = Direction::BACKWARD;
        }
        applyDirection();

        portENTER_CRITICAL(&g_motorMux);
        moveTarget_ = profile.targetPosition;
        hasTarget_ = true;
        portEXIT_CRITICAL(&g_motorMux);

        startMotion(profile.maxVelocitySps, 0, 0);

        fsm_.requestStarting();
        fsm_.requestRunning(direction_);
    }

    void LocalMotor::startMotion(int32_t speedSps, uint32_t accelMs, uint32_t decelMs) {
        decelerating_ = false;

        // Re-energise the driver. The lib auto-disables EN at every
        // motion-end (mirrors OLD basic_motor behaviour) so coils
        // freewheel at standstill and the chopper stays silent. Each
        // new motion has to switch EN back on before stepping. Idempotent
        // when the driver is already enabled.
        driver_->enable();

        // Tell the driver to prepare stall detection for this motion
        driver_->prepareStallDetection(speedSps, accelMs);

        // Speed-proportional current (opt-in). No-op unless the user configured
        // a curve and turned it on. Runs before stepper_.start() so the first
        // step pulses already carry the right current.
        applyCurrentForSpeed(speedSps);

        stepper_.setSpeed(speedSps, accelMs, decelMs);
        stepper_.start();
    }

    void LocalMotor::applyCurrentForSpeed(int32_t speedSps) {
        if (!currentCurveEnabled_ || driver_ == nullptr) {
            return;
        }
        driver_->setRunCurrent(currentMaForSps(currentCurve_, speedSps));
    }

    int32_t LocalMotor::convertToSps(SpeedValue speed) const {
        switch (speed.unit) {
            case SpeedUnit::STEPS_PER_SEC:
                return static_cast<int32_t>(speed.value);
            case SpeedUnit::MM_PER_SEC:
                return static_cast<int32_t>(speed.value * stepsPerMm_);
            case SpeedUnit::CM_PER_SEC:
                return static_cast<int32_t>(speed.value * CM_TO_MM * stepsPerMm_);
            case SpeedUnit::DEGREES_PER_SEC:
                return static_cast<int32_t>(speed.value * stepsPerDeg_);
        }
        return static_cast<int32_t>(speed.value);
    }

    int32_t LocalMotor::convertToSteps(float value, DistanceUnit unit) const {
        switch (unit) {
            case DistanceUnit::STEPS:
                return static_cast<int32_t>(value);
            case DistanceUnit::MM:
                return static_cast<int32_t>(value * stepsPerMm_);
            case DistanceUnit::CM:
                return static_cast<int32_t>(value * CM_TO_MM * stepsPerMm_);
            case DistanceUnit::DEGREES:
                return static_cast<int32_t>(value * stepsPerDeg_);
        }
        return static_cast<int32_t>(value);
    }

}  // namespace motor
