// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <atomic>
#include <cstdint>

#include "ungula/motor/i_limit_system.h"
#include "ungula/motor/i_step_signal_generator.h"

namespace ungula::motor
{

/// Default polled-plus-ISR limit system.
///
/// `EmergencyLimit` triggers immediate halt and transitions the axis to
/// `EmergencyStopped`. ISR-side halt calls
/// `IStepSignalGenerator::stop(Immediate)`, and stop reason is tracked
/// in the latch state.
class LimitSystem final : public ILimitSystem {
    public:
        LimitSystem() = default;
        LimitSystem(const LimitSystem &) = delete;
        LimitSystem &operator=(const LimitSystem &) = delete;

        Status begin(const LimitWiring *wirings, uint8_t count,
                     IStepSignalGenerator *engineForIsr) override;

        /// Auto-count overload: pass the limits_wiring array and the
        /// step signal generator pointer; the system iterates up to
        /// `MAX_LIMIT_INPUTS` slots and treats any row with
        /// `pin == GPIO_NONE` as unused. Hosts do not need to track
        /// the row count themselves.
        Status begin(const LimitWiring (&wirings)[MAX_LIMIT_INPUTS],
                     IStepSignalGenerator *engineForIsr);

        void end() override;
        void service(int64_t nowMs) override;

        bool isActive(LimitKind kind) const override;
        bool isActive(LimitKind kind, Direction dir) const override;

        bool consumeEmergencyActivation() override;
        bool consumeStallActivation() override;

        void notifyMotionStart(int64_t nowMs) override;
        void notifyMotionEnd() override;

        uint32_t totalStallHits() const override;
        void resetStallHitsTotal() override;

        // ---- Diagnostics / test helpers ---------------------------------
        bool isAssertedLive(LimitKind kind) const;
        uint8_t homePin() const;
        void simulateIsrEdgeForTesting(LimitKind kind);

    private:
        /// True if the kind's ISR halts the engine directly (calls
        /// `engineForIsr_->stop(Immediate)`). EmergencyLimit and
        /// StallSensor do; TravelLimit only edge-latches per-slot and
        /// lets the task-side direction-aware path issue the stop.
        static bool stopsInIsr(LimitKind k);

        /// True if the kind requires a GPIO interrupt to be attached.
        /// EmergencyLimit, StallSensor, and TravelLimit all attach an
        /// ISR — TravelLimit for fast edge-latching so short hand-taps
        /// are not missed by polling. HomeSensor remains polled (the
        /// homing strategy reads it via `isActive` at its own cadence).
        static bool attachesIsr(LimitKind k);

        static bool readActive(const LimitWiring &cfg);

        struct Slot {
                LimitWiring cfg{};
                bool inUse = false;
                bool stableActive = false;
                bool candidateActive = false;
                int64_t candidateSinceMs = 0;
                /// Set by the GPIO ISR for TravelLimit when an
                /// asserting edge fires. Drained task-side in
                /// `service()`, which copies the flag into
                /// `edgeArmed` so the activation state machine can
                /// carry the "ISR saw an edge recently" hint across
                /// multiple service ticks. The latch itself is
                /// edge-only (cleared on first `exchange`).
                std::atomic<bool> isrEdgeLatched{ false };
                /// Task-side mirror of `isrEdgeLatched` that persists
                /// across service ticks until either confirmed by a
                /// second active polled read (promote to
                /// `stableActive`) or disarmed by a single inactive
                /// polled read (the edge was a noise spike). The
                /// two-tick requirement is what gives the lib
                /// noise immunity against EMI bursts that fire the
                /// ISR but happen to coincide with at most one
                /// polled read at active.
                bool edgeArmed = false;

                // std::atomic kills the implicit copy ctor / assign;
                // make that explicit so a future maintainer is not
                // surprised by a missing assignment.
                Slot() = default;
                Slot(const Slot &) = delete;
                Slot &operator=(const Slot &) = delete;
        };

        struct IsrBinding {
                LimitSystem *bank = nullptr;
                uint8_t pin = GPIO_NONE;
                LimitKind kind = LimitKind::TravelLimit;
                /// Index into `slots_` so the ISR can reach the
                /// per-slot edge latch in O(1). 0xFF means "unused".
                uint8_t slotIndex = 0xFF;
        };
        static void onIsrTrampoline(void *ctx);

        Slot slots_[MAX_LIMIT_INPUTS]{};
        uint8_t slotCount_ = 0;
        IsrBinding isrBindings_[MAX_LIMIT_INPUTS]{};
        uint8_t isrBindingCount_ = 0;

        IStepSignalGenerator *engineForIsr_ = nullptr;

        std::atomic<bool> emergencyLatched_{ false };
        std::atomic<bool> stallLatched_{ false };
        std::atomic<uint32_t> stallHitCounter_{ 0 };
        std::atomic<int64_t> stallArmedAtMs_{ 0 };
        std::atomic<uint32_t> stallHitsTotal_{ 0 };
        uint8_t stallHitsToTrigger_ = 4;
        uint16_t stallArmDelayMs_ = 200;

        bool begun_ = false;
};

} // namespace ungula::motor
