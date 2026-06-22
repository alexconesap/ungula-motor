// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <atomic>
#include <cstdint>

#include "ungula/motor/i_step_signal_generator.h"
#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motor_step_timing.h"

namespace ungula::motor
{

/// `IStepSignalGenerator` backed by the ESP32 RMT (Remote Control)
/// peripheral. This is the **recommended** step signal generator for
/// any motor that needs reliable step rates above ~100 kSPS, and the
/// path the brief calls out as "vital" for Wendy-Tensy and similar
/// high-RPM setups.
///
/// ## Why RMT
///
/// The gptimer-based generator (`GptimerStepSignal`) drives STEP from
/// a high-frequency alarm ISR — every pulse costs an ISR fire. Once
/// the alarm rate exceeds the ISR's wall-clock cost (~5–10 µs on
/// ESP32), the CPU spends 100 % of its time in the ISR and the
/// Interrupt watchdog trips. On ESP32 that practical ceiling is
/// ~100 kSPS.
///
/// RMT is a **hardware** pulse generator. The CPU loads a small
/// symbol buffer; the RMT peripheral drives the STEP pin
/// autonomously — no per-pulse ISR involvement. The ceiling is the
/// RMT's own clock (up to 80 MHz, or 40 MSPS theoretical). In
/// practice 250–500 kSPS is comfortable; higher is achievable with
/// careful symbol buffer sizing.
///
/// ## Operating model
///
/// Each `PlannedMove` segment is queued as one RMT transmission with
/// a single-symbol payload (HIGH for `halfPeriodTicks`, LOW for
/// `halfPeriodTicks`) and `loop_count = stepCount - 1`. The driver's
/// transmission queue (`trans_queue_depth`) chains segments
/// automatically — no per-segment ISR callback latency. Motion
/// completes when the last queued transmission finishes; an
/// `on_trans_done` callback decrements a pending-segments counter so
/// `status().running` reflects real motion state.
///
/// ## Position tracking
///
/// Open-loop "commanded position" — advances in real time as the
/// custom encoder pushes symbols. `armMove` records the pre-arm
/// position as a baseline and the move direction sign; the encoder
/// keeps an atomic per-symbol counter; `commandedPosition()` returns
/// `baseline + sign * emitted` computed live (wrap-safe via uint32
/// modular arithmetic — the counter is allowed to roll past
/// INT32_MAX during a long indefinite jog). On `stop` / completion
/// the live value is committed to the steady-state field.
///
/// Latency: the encoder is up to `memBlockSymbols` symbols ahead of
/// the GPIO (at 1 MHz resolution / 416 kSPS that's ≈150 µs ahead).
/// For host-visible position queries this is well below noise. Tests
/// needing per-edge accuracy should still wire a PCNT channel.
///
/// ## Platform
///
/// ESP-IDF only. The implementation lives in
/// `rmt_step_signal.cpp` guarded by `ESP_PLATFORM`/`ARDUINO_ARCH_ESP32`
/// — host tests don't link against this class (they use
/// `FakeStepSignal`). The class declaration here is platform-neutral
/// (opaque `void*` handles) so the public header is safe to include
/// from any target.
class RmtStepSignal final : public IStepSignalGenerator {
    public:
        /// Configuration for the RMT TX channel. Defaults are
        /// reasonable for stepper motion in the 1 kSPS – 500 kSPS
        /// range.
        struct Config {
                /// RMT tick rate. 1 MHz (1 µs/tick) covers any
                /// practical motor rate; bump to 10 MHz (100 ns) when a
                /// drive (e.g. a YPMC servo) needs finer step timing.
                /// Each symbol duration field is 15 bits (max 0x7FFF
                /// ticks), but the encoder splits a step whose
                /// half-period exceeds one slot across several symbols,
                /// so even the slow tail of a ramp at 10 MHz is emitted
                /// correctly — high resolutions no longer cap the
                /// slowest representable step rate.
                uint32_t resolutionHz = 1'000'000u;
                /// Symbols held in the RMT channel's hardware buffer.
                /// 64 is the ESP-IDF default; bump for very long
                /// segments if you see encoder underruns.
                uint32_t memBlockSymbols = 64u;
                /// Transmissions the queue can hold without
                /// blocking. The planner emits ≤32 segments per move
                /// (14 accel + cruise + 14 decel + headroom), so 32
                /// is enough to queue a full move upfront and let the
                /// hardware chain them.
                uint32_t transQueueDepth = 32u;
        };

        RmtStepSignal();
        explicit RmtStepSignal(const Config &cfg);
        ~RmtStepSignal() override;

        RmtStepSignal(const RmtStepSignal &) = delete;
        RmtStepSignal &operator=(const RmtStepSignal &) = delete;

        // ---- IStepSignalGenerator ----------------------------------------
        Status begin(uint8_t stepPin, uint8_t dirPin, bool dirActiveHigh, uint32_t dirSetupUs,
                     uint32_t minPulseHighUs, uint32_t minPulseLowUs) override;
        void end() override;
        Status armMove(const PlannedMove &move) override;
        Status stop(StopMode mode) override;
        Status armDecelStop(const PlannedMove &decelMove) override;
        StepSignalStatus status() const override;
        Position commandedPosition() const override;
        uint32_t commandedSpsNow() const override;
        Status resetPosition(Position newSteps) override;
        Status clearFault() override;
        uint32_t timerResolutionHz() const override
        {
                return cfg_.resolutionHz;
        }
        /// Minimum half-period the RMT can sustain. RMT's per-symbol
        /// duration field is 15-bit (so very long pulses split into
        /// multiple symbols), and the practical floor is 1 tick (the
        /// hardware can drive single-tick edges). We report 1 here
        /// so the planner can use the full configured resolution.
        uint32_t minTimerTicks() const override
        {
                return 1u;
        }

    private:
        Config cfg_;

        // Opaque platform handles. On ESP-IDF these are
        // `rmt_channel_handle_t` and `rmt_encoder_handle_t`; on host
        // they stay nullptr and the implementation methods all
        // return InvalidConfig / NotInitialized.
        void *channel_ = nullptr;
        void *encoder_ = nullptr;

        // Pin / timing config (set at begin()).
        uint8_t stepPin_ = 0xFFu;
        uint8_t dirPin_ = 0xFFu;
        bool dirActiveHigh_ = true;
        uint32_t dirSetupUs_     = timing::kDefaultDirSetupUs;
        uint32_t minPulseHighUs_ = timing::kDefaultMinPulseHighUs;
        uint32_t minPulseLowUs_  = timing::kDefaultMinPulseLowUs;
        bool begun_ = false;

        // Motion state shared with the on_trans_done ISR callback.
        std::atomic<bool> running_{ false };
        std::atomic<bool> faulted_{ false };
        // Set true by stop() when it halts a running transmission. The encoder is
        // told to COMPLETE, so that transmission's on_trans_done STILL fires — this
        // flag makes the next completion callback no-op so a stopped move's late ISR
        // can't clear `running_` of the NEXT move armed right after it (which left
        // the axis wedged: motor moving but running_=false, or stuck busy).
        std::atomic<bool> ignoreNextDone_{ false };
        std::atomic<int32_t> pendingSegments_{ 0 };
        std::atomic<int32_t> commandedPosition_{ 0 };
        std::atomic<uint32_t> commandedSpsNow_{ 0 };
        std::atomic<StopReason> finishedReason_{ StopReason::None };

        // Position-baseline state — set by armMove, read by both
        // commandedPosition() (task context) and the on_done ISR
        // when it commits the live value to commandedPosition_.
        //   live position = baselinePosition_ + directionSign_ * emitted
        // where `emitted` is the encoder's atomic step counter.
        std::atomic<int32_t> baselinePosition_{ 0 };
        std::atomic<int8_t> directionSign_{ 1 };

        // Static ISR trampoline (returns `bool need_yield` per ESP-IDF
        // RMT callback contract).
        static bool onTxDoneCallback(void *channel, const void *edata, void *userCtx);

        // Helpers (defined only in the ESP-IDF .cpp build).
        Status sendSegment(const MotionSegment &seg);
};

} // namespace ungula::motor
