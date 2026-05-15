// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/pulse/hal_pulse_engine.h"

#include "ungula/core/time/time.h"
#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;
namespace timer = ungula::hal::timer;

HalPulseEngine::HalPulseEngine(timer::IHwTimer &tmr, const Config &cfg)
        : timer_(tmr)
        , cfg_(cfg)
{
}

void HalPulseEngine::onAlarmIsr(void *ctx)
{
        auto *self = static_cast<HalPulseEngine *>(ctx);
        self->handleAlarmIsr();
}

Status HalPulseEngine::begin(PulseMode mode)
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (cfg_.stepPin == GPIO_NONE || cfg_.dirPin == GPIO_NONE) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (mode != PulseMode::Internal) {
                // External mode lands in a later phase; explicit rejection
                // beats silently ignoring the mode flag.
                return Status::Err(ErrorCode::Unsupported);
        }
        if (cfg_.timerResolutionHz == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Configure the pins. STEP and DIR are both outputs, both seeded low.
        // We use the *checked* config* functions here because pin numbers
        // come from user config; we want to surface a real error rather
        // than crash on a bad pin.
        if (!gpio::configOutput(cfg_.stepPin)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (!gpio::configOutput(cfg_.dirPin)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        gpio::setLow(cfg_.stepPin);
        gpio::write(cfg_.dirPin, cfg_.dirActiveHigh ?
                                     false :
                                     true); // forward = HIGH by default; idle = !forward
        stepPinHigh_ = false;

        // Optional secondary DIR mirror — see `Config::secondaryDirPin`
        // doc. We seed it to "idle" (the polarity-neutral, non-forward
        // state) the same way the primary is seeded above; `start()`
        // writes the real polarity-resolved level alongside the primary.
        if (cfg_.secondaryDirPin != GPIO_NONE) {
                if (!gpio::configOutput(cfg_.secondaryDirPin)) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                const bool idleSecondaryHigh = cfg_.secondaryDirActiveHigh ? false : true;
                gpio::write(cfg_.secondaryDirPin, idleSecondaryHigh);
        }

        // Bring up the HAL timer and wire its ISR callback to us.
        timer::HwTimerConfig tcfg;
        tcfg.resolutionHz = cfg_.timerResolutionHz;
        tcfg.minTicks = cfg_.timerMinTicks ? cfg_.timerMinTicks : 1u;

        const auto tStatus = timer_.begin(tcfg);
        if (tStatus != timer::HwTimerStatus::Ok &&
            tStatus != timer::HwTimerStatus::AlreadyInitialized) {
                return Status::Err(ErrorCode::DriverFault);
        }
        if (timer_.setCallback(&HalPulseEngine::onAlarmIsr, this) != timer::HwTimerStatus::Ok) {
                return Status::Err(ErrorCode::DriverFault);
        }

        mode_ = mode;
        begun_ = true;
        return Status::Ok();
}

Status HalPulseEngine::loadMove(const PlannedMove &move)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (running_.load())
                return Status::Err(ErrorCode::MotionInProgress);
        if (move.segmentCount == 0 || move.totalSteps == 0)
                return Status::Err(ErrorCode::InvalidConfig);

        // Validate every segment up-front so the ISR can trust the queue.
        // Two-layer defence (planner also clamps): refuse anything the HAL
        // timer would reject downstream — `halfPeriodTicks < timerMinTicks`
        // would surface as `InvalidTicks` at the first `rearmFromIsr` call
        // and we'd have to fault the engine inside the ISR. Easier and
        // cleaner to reject it here.
        const uint32_t minHp = cfg_.timerMinTicks ? cfg_.timerMinTicks : 1u;
        uint32_t segSum = 0;
        for (uint8_t i = 0; i < move.segmentCount; ++i) {
                if (move.segments[i].stepCount == 0)
                        return Status::Err(ErrorCode::InvalidConfig);
                if (move.segments[i].halfPeriodTicks < minHp)
                        return Status::Err(ErrorCode::InvalidConfig);
                segSum += move.segments[i].stepCount;
        }
        // The planner guarantees `sum(segments) == totalSteps`; refuse any
        // hand-built move that violates the invariant — the ISR depends on
        // it for the move-complete detection.
        if (segSum != move.totalSteps) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        // Copy the move into our own storage so the caller can free / reuse
        // theirs immediately. PlannedMove is a POD; copy is cheap.
        move_ = move;
        moveLoaded_ = true;

        // Reset per-move counters.
        currentSegment_.store(0, std::memory_order_release);
        stepsInSegment_.store(0, std::memory_order_release);
        totalStepsEmitted_.store(0, std::memory_order_release);
        finishedReason_.store(StopReason::None, std::memory_order_release);

        return Status::Ok();
}

Status HalPulseEngine::start()
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);
        if (!moveLoaded_)
                return Status::Err(ErrorCode::InvalidState);
        if (running_.load())
                return Status::Err(ErrorCode::MotionInProgress);
        if (faulted_.load())
                return Status::Err(ErrorCode::DriverFault);

        // Drive the direction pin first. Polarity:
        //   Forward & dirActiveHigh=true  → DIR = HIGH
        //   Forward & dirActiveHigh=false → DIR = LOW
        //   Backward inverts.
        const bool wantHigh = (move_.direction == Direction::Forward) ? cfg_.dirActiveHigh :
                                                                        !cfg_.dirActiveHigh;
        gpio::write(cfg_.dirPin, wantHigh);

        // Optional secondary DIR mirror — written BEFORE the dirSetupUs
        // delay so the same setup window covers both pins. The intent
        // ("does motor 2 turn the same physical direction as motor 1?")
        // is encoded by `secondaryDirInverted`: when true, the second
        // drive sees the OPPOSITE electrical polarity to produce the
        // SAME shaft rotation as the primary (face-to-face mounting).
        if (cfg_.secondaryDirPin != GPIO_NONE) {
                // First map the move direction to "logical forward / backward"
                // (independent of the primary's wiring), then apply the
                // secondary's polarity + the optional invert.
                const bool secondaryWantForward =
                        (move_.direction == Direction::Forward) != cfg_.secondaryDirInverted;
                const bool secondaryHigh = secondaryWantForward ?
                                                   cfg_.secondaryDirActiveHigh :
                                                   !cfg_.secondaryDirActiveHigh;
                gpio::write(cfg_.secondaryDirPin, secondaryHigh);
        }

        // Hold DIR stable before the first STEP edge. We're in task
        // context — `delayUs` is the project's busy-wait API.
        if (cfg_.dirSetupUs > 0) {
                ungula::core::time::delayUs(static_cast<int64_t>(cfg_.dirSetupUs));
        }

        // Reseed STEP-pin state. STEP is low entering the train; the first
        // alarm will flip it high.
        gpio::setLow(cfg_.stepPin);
        stepPinHigh_ = false;
        finalEdgePending_ = false;

        // Mark running BEFORE arming the timer so the ISR sees a valid
        // state if it fires immediately. `release` pairs with `acquire`
        // reads from task context queries.
        running_.store(true, std::memory_order_release);

        // Arm the timer with the first segment's half-period. If this
        // fails, undo the running flag and surface the failure.
        const uint32_t firstHp = move_.segments[0].halfPeriodTicks;
        const auto status = timer_.startOneShotTicks(firstHp);
        if (status != timer::HwTimerStatus::Ok) {
                running_.store(false, std::memory_order_release);
                finishedReason_.store(StopReason::DriverFault, std::memory_order_release);
                return Status::Err(ErrorCode::DriverFault);
        }
        return Status::Ok();
}

Status HalPulseEngine::stop(StopMode mode)
{
        if (!begun_)
                return Status::Err(ErrorCode::NotInitialized);

        // Decelerate requires mid-flight segment-queue replacement (engine
        // doesn't support that yet). REFUSE rather than silently treating
        // Decelerate as Immediate — motion control depends on accurate stop
        // semantics, and a "soft stop" that's actually a hard stop is a
        // safety bug. The Axis facade is the layer responsible for planning
        // a decel ramp; until that path exists, Decelerate is Unsupported
        // everywhere.
        if (mode == StopMode::Decelerate) {
                return Status::Err(ErrorCode::Unsupported);
        }

        if (!running_.load()) {
                return Status::Ok(); // already idle; idempotent
        }

        const StopReason reason = (mode == StopMode::Emergency) ? StopReason::EmergencyStop :
                                                                  StopReason::UserStop;

        // Stop the hardware counter cleanly from task context — `stop()` is
        // task-safe and clears the timer's armed flag so any in-flight
        // alarm that hasn't fired yet won't run our ISR. (Already-pending
        // ISR firings on another core are dropped by the HAL timer's
        // own software gate at the trampoline.)
        timer_.stop();

        // Park STEP low so a freshly-arrived high edge doesn't leave the
        // driver in an indeterminate state at next motion.
        gpio::setLow(cfg_.stepPin);
        stepPinHigh_ = false;
        finalEdgePending_ = false;

        running_.store(false, std::memory_order_release);
        finishedReason_.store(reason, std::memory_order_release);

        return Status::Ok();
}

Status HalPulseEngine::emergencyStop()
{
        const auto s = stop(StopMode::Emergency);
        if (!s.ok())
                return s;
        // Latch a fault — Axis must call `clearFault()` to clear before
        // the next motion can be armed.
        faulted_.store(true, std::memory_order_release);
        return Status::Ok();
}

UNGULA_ISR_ATTR void HalPulseEngine::haltFromIsr(StopReason reason)
{
        // ISR-context emergency halt — wired to GPIO ISRs on crash
        // limits and E-stop inputs. Three atomic operations, one
        // ISR-safe timer disarm, nothing else. The Axis service path
        // picks up `faulted_` + `finishedReason_` on its next tick
        // and dispatches the `LimitActivated` / `EmergencyStopped`
        // event from task context.
        //
        // `timer_.disarmFromIsr` returns a status we cannot meaningfully
        // act on from ISR (logging would be a contract violation). The
        // worst-case failure is "timer already disarmed", which is
        // benign — the alarm just won't fire again.
        (void)timer_.disarmFromIsr();
        running_.store(false, std::memory_order_release);
        faulted_.store(true, std::memory_order_release);
        finishedReason_.store(reason, std::memory_order_release);
}

bool HalPulseEngine::isRunning() const
{
        return running_.load(std::memory_order_acquire);
}

PulseEngineStatus HalPulseEngine::status() const
{
        PulseEngineStatus s;
        s.running = running_.load(std::memory_order_acquire);
        s.faulted = faulted_.load(std::memory_order_acquire);
        s.emittedSteps = totalStepsEmitted_.load(std::memory_order_acquire);
        s.totalSteps = move_.totalSteps;
        s.segmentIndex = currentSegment_.load(std::memory_order_acquire);
        s.finishedReason = finishedReason_.load(std::memory_order_acquire);
        return s;
}

int32_t HalPulseEngine::commandedPositionSteps() const
{
        return commandedPosition_.load(std::memory_order_acquire);
}

Status HalPulseEngine::resetPosition(int32_t positionSteps)
{
        if (running_.load())
                return Status::Err(ErrorCode::MotionInProgress);
        commandedPosition_.store(positionSteps, std::memory_order_release);
        return Status::Ok();
}

Status HalPulseEngine::clearFault()
{
        if (running_.load())
                return Status::Err(ErrorCode::MotionInProgress);
        faulted_.store(false, std::memory_order_release);
        finishedReason_.store(StopReason::None, std::memory_order_release);
        return Status::Ok();
}

// =====================================================================
// ISR body
// =====================================================================
//
// Runs from the HAL timer's alarm callback. Constraints:
//   - No logging, no UART, no I2C/SPI.
//   - No floating point.
//   - All HAL calls used here are documented ISR-safe by lib_hal:
//       * `gpio::setHigh` / `setLow` compile to direct `gpio_ll`
//         register writes (W1TS / W1TC) — single-cycle, ISR-safe by
//         contract (see lib_hal/README.md "Hardware timer" section).
//       * `timer.rearmFromIsr` / `disarmFromIsr` are the HAL timer's
//         explicit ISR-safe interface.
//   - All shared state is `std::atomic<>` or strictly ISR-local.
//
// Failure handling: every timer call's return is captured. If the HAL
// timer ever refuses an operation (e.g. `InvalidTicks`, `BackendError`),
// the engine immediately latches `PulseEngineFault` + `StopReason::DriverFault`
// and best-effort disarms. The task-context status query surfaces the
// fault to the host. Doing this defensively in the ISR closes the
// "engine claims running but no edges fire" failure mode the user
// flagged.
// =====================================================================

/// Called from inside the ISR on any non-Ok return from the HAL timer.
/// Latches a `DriverFault` and disarms the timer (best effort). Must be
/// IRAM-safe; the placement attribute lives on this definition. Use the
/// portable `UNGULA_ISR_ATTR` so the host build (where `IRAM_ATTR` is
/// undefined) still compiles.
static inline void UNGULA_ISR_ATTR latchFaultFromIsr(std::atomic<bool> &running,
                                                     std::atomic<bool> &faulted,
                                                     std::atomic<StopReason> &finishedReason,
                                                     ungula::hal::timer::IHwTimer &timer)
{
        running.store(false, std::memory_order_release);
        faulted.store(true, std::memory_order_release);
        finishedReason.store(StopReason::DriverFault, std::memory_order_release);
        timer.disarmFromIsr(); // best-effort; ignore return
}

void HalPulseEngine::handleAlarmIsr()
{
        // Cheap early exit if a task-context stop landed between the
        // alarm being scheduled and this fire. The HAL timer also has its
        // own gate, but checking here lets us skip the GPIO write too.
        if (!running_.load(std::memory_order_acquire)) {
                return;
        }

        // Toggle STEP. `stepPinHigh_` is the cached pin state — ISR-local.
        //
        // `gpio::setHigh`/`setLow` are ISR-safe by lib_hal contract: on
        // ESP32 they expand to `gpio_ll_set_level` (a single W1TS/W1TC
        // register write, no critical section, no flash access). Documented
        // in lib_hal/README.md and lib_hal/API.md "GPIO — unchecked digital
        // I/O (hot-path)". If the lib_hal contract ever changes, this ISR
        // must move to a dedicated `IFastStepPin` primitive — the tight
        // ISR loop cannot tolerate a function call into ESP-IDF gpio.c.
        const bool nowHigh = !stepPinHigh_;
        stepPinHigh_ = nowHigh;
        if (nowHigh) {
                gpio::setHigh(cfg_.stepPin);
        } else {
                gpio::setLow(cfg_.stepPin);
        }

        // Falling-edge path. Two sub-cases:
        //   1) `finalEdgePending_` was set on the previous rising edge → this
        //      is the closing half of the move's last pulse. The driver got
        //      a full half-period of HIGH; safe to disarm now.
        //   2) Otherwise → ordinary falling edge; re-arm with the current
        //      segment's half-period for the next rising edge.
        if (!nowHigh) {
                if (finalEdgePending_) {
                        running_.store(false, std::memory_order_release);
                        finishedReason_.store(StopReason::TargetReached, std::memory_order_release);
                        finalEdgePending_ = false;
                        const auto ds = timer_.disarmFromIsr();
                        if (ds != timer::HwTimerStatus::Ok) {
                                // disarm itself failed — still treat the move as
                                // completed (we set TargetReached above), but ALSO
                                // latch fault so the host sees a record of the HAL
                                // hiccup. We can't undo the running_=false either.
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

        // Rising-edge path. Count the step and advance segment / mark final
        // edge pending if this is the last step of the last segment.
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
                        // ---- Last step of the move ----
                        // Defer completion to the next (falling) edge so the final
                        // high pulse gets a full half-period of width. Industrial
                        // servo drives (Yaskawa, Delta, etc.) want ≥ 2.5 µs minimum
                        // pulse — completing here would truncate to the ISR's
                        // execution time (~ 1 µs on ESP32 IRAM), which is marginal.
                        finalEdgePending_ = true;
                        const auto rs = timer_.rearmFromIsr(move_.segments[segIdx].halfPeriodTicks);
                        if (rs != timer::HwTimerStatus::Ok) {
                                latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
                        }
                        return;
                }
                // ---- Advance to next segment ----
                stepsInSegment_.store(0, std::memory_order_relaxed);
                currentSegment_.store(nextIdx, std::memory_order_relaxed);
                const auto rs = timer_.rearmFromIsr(move_.segments[nextIdx].halfPeriodTicks);
                if (rs != timer::HwTimerStatus::Ok) {
                        latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
                }
                return;
        }

        // Ordinary rising edge inside the current segment.
        const auto rs = timer_.rearmFromIsr(move_.segments[segIdx].halfPeriodTicks);
        if (rs != timer::HwTimerStatus::Ok) {
                latchFaultFromIsr(running_, faulted_, finishedReason_, timer_);
        }
}

} // namespace ungula::motor
