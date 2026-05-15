// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <atomic>
#include <cstdint>

#include "ungula/hal/core/compiler_attrs.h"
#include "ungula/hal/timer/i_hwtimer.h"

#include "ungula/motor/axis_types.h"
#include "ungula/motor/limits/sensor_input.h" // GPIO_NONE
#include "ungula/motor/planning/planned_move.h"
#include "ungula/motor/pulse/i_pulse_engine.h"

namespace ungula::motor
{

/// Production pulse engine. Drives an `ungula::hal::timer::IHwTimer` +
/// two `ungula::hal::gpio` pins (STEP, DIR) to emit a deterministic
/// step train described by a `PlannedMove`.
///
/// ## Operating model
///
///   1. Construct with a HAL timer reference + a `Config` describing the
///      step/dir pins, direction polarity, dir-setup time, and the timer
///      tick rate.
///   2. `begin(PulseMode::Internal)` — configures GPIO pins as outputs,
///      seeds the STEP pin low, sets the timer's ISR callback, and
///      brings the timer up via `timer.begin()`.
///   3. `loadMove(PlannedMove&)` — copies the planned segment list into
///      the engine. Allowed only while not running.
///   4. `start()` — writes the DIR pin, waits `dirSetupUs` (task
///      context), then arms the timer with the first segment's
///      half-period. From here the alarm ISR drives every edge.
///   5. The ISR toggles STEP on each fire, counts rising edges as
///      steps, advances segments at boundaries, and re-arms the timer.
///      At the last step of the last segment it calls `disarmFromIsr`
///      and latches `finishedReason_ = TargetReached`.
///
/// ## Determinism
///
/// The pulse train is owned by the timer ISR. The main loop, UART,
/// logging, and event listeners cannot disturb step timing — the only
/// thing the ISR depends on is the cached half-period value already
/// stored in `move_.segments[]` (no heap, no UART, no atomics ops more
/// expensive than a single-byte exchange).
///
/// ## Thread / ISR boundaries
///
///   - `begin`, `loadMove`, `start`, `stop`, `emergencyStop`,
///     `resetPosition` are TASK-context only.
///   - `isRunning`, `status`, `commandedPositionSteps` are SAFE from
///     any context (atomic reads).
///   - `onAlarmIsr` / `handleAlarmIsr` are ISR-context only — invoked
///     by the HAL timer's trampoline.
///   - The `PlannedMove` lives in the engine's storage and is only
///     mutated by `loadMove`. The ISR only reads it. Between `start`
///     and "move complete", `loadMove` is rejected with `InvalidState`,
///     so the ISR can read the segment array without a critical
///     section.
class HalPulseEngine final : public IPulseEngine {
    public:
        struct Config {
                uint8_t stepPin = GPIO_NONE;
                uint8_t dirPin = GPIO_NONE;
                bool dirActiveHigh = true;
                /// Microseconds to hold DIR stable after writing it before
                /// toggling STEP for the first time. Industrial drives often
                /// require ≥ 1 µs; we default to 5 µs as a safe baseline.
                uint32_t dirSetupUs = 5;
                /// Tick rate the HAL timer was (or will be) configured with.
                /// Must match what the planner used to compute `halfPeriodTicks`
                /// values in the segment list, otherwise pulse rates will scale
                /// wrong.
                uint32_t timerResolutionHz = 1'000'000;
                /// Minimum ticks for any alarm scheduling on this timer. Passed
                /// straight to `IHwTimer::begin().minTicks`. Set ≥ 5 to leave
                /// the ISR comfortable headroom.
                uint32_t timerMinTicks = 5;
        };

        HalPulseEngine(ungula::hal::timer::IHwTimer &timer, const Config &cfg);

        HalPulseEngine(const HalPulseEngine &) = delete;
        HalPulseEngine &operator=(const HalPulseEngine &) = delete;

        // ---- IPulseEngine -------------------------------------------------

        Status begin(PulseMode mode) override;
        Status loadMove(const PlannedMove &move) override;
        Status start() override;
        Status stop(StopMode mode) override;
        Status emergencyStop() override;
        void haltFromIsr(StopReason reason) override;
        bool isRunning() const override;
        PulseEngineStatus status() const override;
        int32_t commandedPositionSteps() const override;
        Status resetPosition(int32_t positionSteps) override;
        Status clearFault() override;

        // External pulse mode is not implemented. Override the default
        // (Unsupported) explicitly so the intent is documented.
        bool due(int64_t /*nowMicros*/) const override
        {
                return false;
        }
        Status tick(int64_t /*nowMicros*/) override
        {
                return Status::Err(ErrorCode::Unsupported);
        }

        // ---- ISR trampoline target ---------------------------------------

        /// Free-function-shaped entry the HAL timer dispatches to via its
        /// `IsrTimerCallback` slot. Public only so the timer can address it.
        static void onAlarmIsr(void *ctx) UNGULA_ISR_ATTR;

    private:
        /// Body of the ISR. Toggles STEP, accounts for the edge, advances
        /// segment when due, and re-arms (or disarms) the timer.
        void handleAlarmIsr() UNGULA_ISR_ATTR;

        ungula::hal::timer::IHwTimer &timer_;
        Config cfg_;

        /// Currently loaded move. Copied at `loadMove` time and read by the
        /// ISR. Mutated only when `running_ == false`.
        PlannedMove move_{};
        bool moveLoaded_ = false;

        PulseMode mode_ = PulseMode::Internal;
        bool begun_ = false;

        // ---- ISR-touched state (atomic where shared with task context) ---

        std::atomic<bool> running_{ false };
        std::atomic<bool> faulted_{ false };
        std::atomic<uint8_t> currentSegment_{ 0 };
        std::atomic<uint32_t> stepsInSegment_{ 0 };
        std::atomic<uint32_t> totalStepsEmitted_{ 0 };
        std::atomic<int32_t> commandedPosition_{ 0 };

        /// StopReason is a `uint8_t`-backed enum — `std::atomic<StopReason>`
        /// is lock-free on every target we care about. Used so the task-
        /// context `status()` query can read why the ISR ended a move
        /// without holding a lock.
        std::atomic<StopReason> finishedReason_{ StopReason::None };

        /// Current logical level of the STEP pin. ISR-only access — same
        /// core, single writer, no atomicity needed. Tracking it here lets
        /// us implement "rising edge = count step" without re-reading the
        /// pin (which would be ~10× slower than this bool on Xtensa).
        bool stepPinHigh_ = false;

        /// Set on the final step's RISING edge; consumed on the following
        /// FALLING edge to disarm the timer and latch `TargetReached`.
        /// Lets the last high pulse get a full half-period instead of being
        /// truncated to the ISR's execution time — matters for industrial
        /// servo drives that require ≥2.5 µs minimum pulse width. ISR-only
        /// access (set on rising, cleared on falling); reset in `start()`.
        bool finalEdgePending_ = false;
};

} // namespace ungula::motor
