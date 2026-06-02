// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <atomic>
#include <cstdint>

#include "ungula/hal/core/compiler_attrs.h"
#include "ungula/hal/timer/i_hwtimer.h"

#include "ungula/motor/i_step_signal_generator.h"
#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motor_step_timing.h"

namespace ungula::motor
{

/// Default step-signal generator on ESP32 (and any platform with an
/// `ungula::hal::timer::IHwTimer`). Wraps a hardware timer + one STEP
/// pin + one DIR pin into a `PlannedMove` runtime.
///
/// `IStepSignalGenerator` contract:
///   - `armMove(move)` arms and starts one planned move.
///   - Drivers that need a secondary DIR pin write it via
///     `lib_hal::gpio` before arming.
///   - ISR limit paths call `stop(Immediate)` from interrupt context.
///   - `commandedSpsNow()` / `timerResolutionHz()` / `minTimerTicks()`
///     expose runtime state without leaking internals.
///
/// Determinism: the pulse train runs entirely from the timer ISR. No
/// task-context code participates in step timing once a move is armed.
/// The ISR depends only on cached half-period values inside the move's
/// segment list — no heap, no UART, no flash access.
///
/// Thread / ISR boundaries:
///   - `begin`, `end`, `armMove`, `stop` (task variants), `resetPosition`,
///     `clearFault` are TASK-context only.
///   - `status`, `commandedPosition`, `commandedSpsNow`,
///     `timerResolutionHz`, `minTimerTicks` are SAFE from any context.
///   - `onAlarmIsr` runs in ISR context; the implementation is
///     responsible for ISR safety.
///
/// `stop(StopMode::Immediate)` is the only `stop()` mode that may be
/// invoked from ISR context (and it is — the `LimitSystem`'s emergency
/// / stall ISR path calls it directly). Implementation is IRAM-safe and
/// only touches atomics + the HAL timer's `disarmFromIsr`.
class GptimerStepSignal final : public IStepSignalGenerator {
    public:
        /// Configuration the generator needs that lives outside the
        /// `IStepSignalGenerator::begin()` interface — specifically the
        /// timer's tick rate and minimum schedulable interval. These are
        /// properties of the injected `IHwTimer` and the host must know
        /// them anyway to pre-plan moves with matching segment ticks.
        struct Config {
                uint32_t timerResolutionHz = 1'000'000u;
                uint32_t timerMinTicks = 5u;
        };

        GptimerStepSignal(ungula::hal::timer::IHwTimer &timer, const Config &cfg);

        GptimerStepSignal(const GptimerStepSignal &) = delete;
        GptimerStepSignal &operator=(const GptimerStepSignal &) = delete;

        // ---- IStepSignalGenerator ----------------------------------------
        Status begin(uint8_t stepPin, uint8_t dirPin, bool dirActiveHigh, uint32_t dirSetupUs,
                     uint32_t minPulseHighUs, uint32_t minPulseLowUs) override;
        void end() override;
        Status armMove(const PlannedMove &move) override;
        Status stop(StopMode mode) override;
        StepSignalStatus status() const override;
        Position commandedPosition() const override;
        uint32_t commandedSpsNow() const override;
        Status resetPosition(Position newSteps) override;
        Status clearFault() override;
        uint32_t timerResolutionHz() const override
        {
                return cfg_.timerResolutionHz;
        }
        uint32_t minTimerTicks() const override
        {
                return cfg_.timerMinTicks;
        }

        // ---- ISR trampoline ---------------------------------------------
        /// Public so the HAL timer's callback slot can address it. NOT
        /// for application code.
        static void onAlarmIsr(void *ctx) UNGULA_ISR_ATTR;

    private:
        void handleAlarmIsr() UNGULA_ISR_ATTR;

        ungula::hal::timer::IHwTimer &timer_;
        Config cfg_;

        // Pin / timing config (set at begin()).
        uint8_t stepPin_ = GPIO_NONE;
        uint8_t dirPin_ = GPIO_NONE;
        bool dirActiveHigh_ = true;
        uint32_t dirSetupUs_     = timing::kDefaultDirSetupUs;
        uint32_t minPulseHighUs_ = timing::kDefaultMinPulseHighUs;
        uint32_t minPulseLowUs_  = timing::kDefaultMinPulseLowUs;
        bool begun_ = false;

        // Move queue.
        PlannedMove move_{};
        bool moveLoaded_ = false;

        // Shared with ISR — atomics required.
        std::atomic<bool> running_{ false };
        std::atomic<bool> faulted_{ false };
        std::atomic<uint8_t> currentSegment_{ 0 };
        std::atomic<uint32_t> stepsInSegment_{ 0 };
        std::atomic<uint32_t> totalStepsEmitted_{ 0 };
        std::atomic<int32_t> commandedPosition_{ 0 };
        std::atomic<StopReason> finishedReason_{ StopReason::None };

        // ISR-local pin-state cache + final-edge bookkeeping.
        bool stepPinHigh_ = false;
        bool finalEdgePending_ = false;
};

} // namespace ungula::motor
