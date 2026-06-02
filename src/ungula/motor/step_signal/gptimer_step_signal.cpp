// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/step_signal/gptimer_step_signal.h"

#include "ungula/core/time/time.h"
#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;
namespace timer = ungula::hal::timer;

GptimerStepSignal::GptimerStepSignal(timer::IHwTimer &tmr, const Config &cfg)
        : timer_(tmr)
        , cfg_(cfg)
{
}

void GptimerStepSignal::onAlarmIsr(void *ctx)
{
        auto *self = static_cast<GptimerStepSignal *>(ctx);
        self->handleAlarmIsr();
}

Status GptimerStepSignal::begin(uint8_t stepPin, uint8_t dirPin, bool dirActiveHigh,
                                uint32_t dirSetupUs, uint32_t minPulseHighUs,
                                uint32_t minPulseLowUs)
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (stepPin == GPIO_NONE || dirPin == GPIO_NONE) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (cfg_.timerResolutionHz == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        stepPin_ = stepPin;
        dirPin_ = dirPin;
        dirActiveHigh_ = dirActiveHigh;
        dirSetupUs_ = dirSetupUs;
        minPulseHighUs_ = minPulseHighUs;
        minPulseLowUs_ = minPulseLowUs;

        if (!gpio::configOutput(stepPin_)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (!gpio::configOutput(dirPin_)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        gpio::setLow(stepPin_);
        // Idle DIR at the "not-forward" level. armMove() rewrites it
        // alongside the dirSetupUs window.
        gpio::write(dirPin_, dirActiveHigh_ ? false : true);
        stepPinHigh_ = false;

        // Bring up the HAL timer and wire its ISR callback.
        timer::HwTimerConfig tcfg;
        tcfg.resolutionHz = cfg_.timerResolutionHz;
        tcfg.minTicks = cfg_.timerMinTicks ? cfg_.timerMinTicks : 1u;

        const auto tStatus = timer_.begin(tcfg);
        if (tStatus != timer::HwTimerStatus::Ok &&
            tStatus != timer::HwTimerStatus::AlreadyInitialized) {
                return Status::Err(ErrorCode::DriverFault);
        }
        if (timer_.setCallback(&GptimerStepSignal::onAlarmIsr, this) != timer::HwTimerStatus::Ok) {
                return Status::Err(ErrorCode::DriverFault);
        }

        begun_ = true;
        return Status::Ok();
}

void GptimerStepSignal::end()
{
        if (begun_ && running_.load(std::memory_order_acquire)) {
                (void)stop(StopMode::Immediate);
        }
        // Best effort — concrete HAL impl owns ISR detach inside.
        begun_ = false;
        moveLoaded_ = false;
        running_.store(false, std::memory_order_release);
        faulted_.store(false, std::memory_order_release);
}

Status GptimerStepSignal::armMove(const PlannedMove &move)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (running_.load(std::memory_order_acquire)) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        if (faulted_.load(std::memory_order_acquire)) {
                return Status::Err(ErrorCode::DriverFault);
        }
        if (move.segmentCount == 0 || move.totalSteps == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Validate every segment up-front so the ISR can trust the queue.
        const uint32_t minHp = cfg_.timerMinTicks ? cfg_.timerMinTicks : 1u;
        uint32_t segSum = 0;
        for (uint8_t i = 0; i < move.segmentCount; ++i) {
                if (move.segments[i].stepCount == 0) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                if (move.segments[i].halfPeriodTicks < minHp) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                segSum += move.segments[i].stepCount;
        }
        if (segSum != move.totalSteps) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Copy move into our storage (caller can free / reuse theirs).
        move_ = move;
        moveLoaded_ = true;

        // Reset per-move counters.
        currentSegment_.store(0, std::memory_order_release);
        stepsInSegment_.store(0, std::memory_order_release);
        totalStepsEmitted_.store(0, std::memory_order_release);
        finishedReason_.store(StopReason::None, std::memory_order_release);

        // Drive DIR and wait the configured setup window before the first
        // STEP edge. Task context only.
        const bool wantHigh = (move_.direction == Direction::Forward) ? dirActiveHigh_ :
                                                                        !dirActiveHigh_;
        gpio::write(dirPin_, wantHigh);
        if (dirSetupUs_ > 0) {
                ungula::core::time::delayUs(static_cast<int64_t>(dirSetupUs_));
        }

        gpio::setLow(stepPin_);
        stepPinHigh_ = false;
        finalEdgePending_ = false;

        running_.store(true, std::memory_order_release);

        const uint32_t firstHp = move_.segments[0].halfPeriodTicks;
        const auto status = timer_.startOneShotTicks(firstHp);
        if (status != timer::HwTimerStatus::Ok) {
                running_.store(false, std::memory_order_release);
                finishedReason_.store(StopReason::DriverFault, std::memory_order_release);
                return Status::Err(ErrorCode::DriverFault);
        }
        return Status::Ok();
}

Status GptimerStepSignal::stop(StopMode mode)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        // Mid-flight decel ramp belongs at the axis layer (it can
        // synthesise a decel-only PlannedMove and re-arm); the
        // generator only knows hard stop.
        if (mode == StopMode::Decelerate) {
                return Status::Err(ErrorCode::Unsupported);
        }
        if (!running_.load(std::memory_order_acquire)) {
                return Status::Ok();
        }

        // ISR-safe disarm. Calling stop(Immediate) from either task or
        // ISR context is allowed because `disarmFromIsr` is what we
        // hit either way — task-context `timer_.stop()` does more
        // (it also halts the counter), but that's safe to defer to a
        // task-context follow-up if needed.
        (void)timer_.disarmFromIsr();
        gpio::setLow(stepPin_);
        stepPinHigh_ = false;
        finalEdgePending_ = false;

        running_.store(false, std::memory_order_release);
        finishedReason_.store(StopReason::UserStop, std::memory_order_release);
        return Status::Ok();
}

StepSignalStatus GptimerStepSignal::status() const
{
        StepSignalStatus s;
        s.running = running_.load(std::memory_order_acquire);
        s.faulted = faulted_.load(std::memory_order_acquire);
        s.finishedReason = finishedReason_.load(std::memory_order_acquire);
        return s;
}

Position GptimerStepSignal::commandedPosition() const
{
        return commandedPosition_.load(std::memory_order_acquire);
}

uint32_t GptimerStepSignal::commandedSpsNow() const
{
        if (!running_.load(std::memory_order_acquire)) {
                return 0u;
        }
        const uint8_t segIdx = currentSegment_.load(std::memory_order_acquire);
        if (segIdx >= move_.segmentCount) {
                return 0u;
        }
        const uint32_t hp = move_.segments[segIdx].halfPeriodTicks;
        if (hp == 0u) {
                return 0u;
        }
        // SPS = resolution / (2 * halfPeriodTicks). Two alarms per step.
        const uint64_t sps = static_cast<uint64_t>(cfg_.timerResolutionHz) / (2ull * hp);
        return (sps > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(sps);
}

Status GptimerStepSignal::resetPosition(Position newSteps)
{
        if (running_.load(std::memory_order_acquire)) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        commandedPosition_.store(newSteps, std::memory_order_release);
        return Status::Ok();
}

Status GptimerStepSignal::clearFault()
{
        if (running_.load(std::memory_order_acquire)) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        faulted_.store(false, std::memory_order_release);
        finishedReason_.store(StopReason::None, std::memory_order_release);
        return Status::Ok();
}

// =====================================================================
// ISR body constraints: no logging, no UART, no flash access. All HAL
// calls used here are documented ISR-safe by lib_hal.
// =====================================================================

namespace
{

        UNGULA_ISR_ATTR inline void latchFaultFromIsr(std::atomic<bool> &running,
                                                      std::atomic<bool> &faulted,
                                                      std::atomic<StopReason> &finishedReason,
                                                      ungula::hal::timer::IHwTimer &timer)
        {
                running.store(false, std::memory_order_release);
                faulted.store(true, std::memory_order_release);
                finishedReason.store(StopReason::DriverFault, std::memory_order_release);
                timer.disarmFromIsr();
        }

} // namespace

void GptimerStepSignal::handleAlarmIsr()
{
        if (!running_.load(std::memory_order_acquire)) {
                return;
        }

        const bool nowHigh = !stepPinHigh_;
        stepPinHigh_ = nowHigh;
        if (nowHigh) {
                gpio::setHigh(stepPin_);
        } else {
                gpio::setLow(stepPin_);
        }

        // Falling-edge path.
        if (!nowHigh) {
                if (finalEdgePending_) {
                        running_.store(false, std::memory_order_release);
                        finishedReason_.store(StopReason::TargetReached, std::memory_order_release);
                        finalEdgePending_ = false;
                        const auto ds = timer_.disarmFromIsr();
                        if (ds != timer::HwTimerStatus::Ok) {
                                faulted_.store(true, std::memory_order_release);
                        }
                        return;
                }
                const uint8_t segIdx = currentSegment_.load(std::memory_order_relaxed);
                const auto rs = timer_.rearmFromIsr(move_.segments[segIdx].halfPeriodTicks);
                if (rs != timer::HwTimerStatus::Ok) {
                        latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
                }
                return;
        }

        // Rising-edge path.
        if (move_.direction == Direction::Forward) {
                commandedPosition_.fetch_add(1, std::memory_order_relaxed);
        } else {
                commandedPosition_.fetch_sub(1, std::memory_order_relaxed);
        }
        totalStepsEmitted_.fetch_add(1, std::memory_order_relaxed);

        const uint32_t newStepsInSeg = stepsInSegment_.fetch_add(1, std::memory_order_relaxed) + 1u;
        const uint8_t segIdx = currentSegment_.load(std::memory_order_relaxed);

        if (newStepsInSeg >= move_.segments[segIdx].stepCount) {
                const uint8_t nextIdx = static_cast<uint8_t>(segIdx + 1u);
                if (nextIdx >= move_.segmentCount) {
                        // Last step of the last segment — defer completion to
                        // the falling edge so the final HIGH gets a full
                        // half-period of width (industrial drives need this).
                        finalEdgePending_ = true;
                        const auto rs = timer_.rearmFromIsr(move_.segments[segIdx].halfPeriodTicks);
                        if (rs != timer::HwTimerStatus::Ok) {
                                latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
                        }
                        return;
                }
                stepsInSegment_.store(0, std::memory_order_relaxed);
                currentSegment_.store(nextIdx, std::memory_order_relaxed);
                const auto rs = timer_.rearmFromIsr(move_.segments[nextIdx].halfPeriodTicks);
                if (rs != timer::HwTimerStatus::Ok) {
                        latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
                }
                return;
        }

        const auto rs = timer_.rearmFromIsr(move_.segments[segIdx].halfPeriodTicks);
        if (rs != timer::HwTimerStatus::Ok) {
                latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
        }
}

} // namespace ungula::motor
