// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/step_signal/rmt_step_signal.h"

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#include <atomic>
#include <cstddef>
#include <cstring>

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_types.h>
#include <esp_attr.h>
#include <esp_err.h>

#include "ungula/core/time/time.h"
#include "ungula/hal/gpio/gpio.h"
#include "ungula/motor/step_signal/step_symbol_plan.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;

// =====================================================================
// Custom RMT encoder — walks a PlannedMove's segments and emits one
// symbol per step.
// =====================================================================
//
// ESP32 classic's RMT peripheral does NOT support `rmt_transmit_config_t::
// loop_count` (only ESP32-S2/S3/C3/C6 do). We can't just say "loop this
// one symbol N times" — we have to actually produce N symbols.
//
// For long segments (the cruise of an indefinite jog has billions of
// steps), pre-allocating N symbols in RAM is impossible. The right
// pattern is a custom encoder: the RMT driver calls us repeatedly to
// refill its memory block as it transmits. We emit one symbol per
// step, track our position across calls via per-encoder state, and
// signal `RMT_ENCODING_COMPLETE` when every segment has been pushed.
//
// Implementation modelled on ESP-IDF's stepper_motor example. The
// encoder uses an inner `copy_encoder` to actually push symbols into
// the channel's memory; we orchestrate which symbol to emit when.

namespace
{

        struct StepEncoder {
                rmt_encoder_t base;
                rmt_encoder_t *copyEncoder = nullptr;
                PlannedMove move{};

                // Walks `move` into its RMT symbol stream one symbol at a time.
                // Handles both fast steps (one symbol) and the slow ends of a
                // ramp whose half-period overflows a single 15-bit RMT slot
                // (several symbols per step). See step_symbol_plan.h.
                rmtgen::MoveSymbolExpander expander{};
                rmt_symbol_word_t currentSymbol{};
                bool symbolReady = false; // currentSymbol awaits / mid-push
                bool currentStepCompletes = false; // it is the last symbol of its step

                // Cumulative steps pushed through the inner copy
                // encoder since the last `armMove` / `reset`. Read
                // from any context (task: `commandedPosition()`,
                // ISR: on_trans_done callback's commit path).
                // Wraps naturally at UINT32_MAX — at 416 kSPS that
                // takes ≈170 minutes of continuous motion.
                std::atomic<uint32_t> stepsEmittedSinceArm{ 0u };

                // Task-side abort signal. `RmtStepSignal::stop()` sets
                // this BEFORE calling `rmt_disable`. The next
                // `stepEncode()` call returns `RMT_ENCODING_COMPLETE`,
                // the in-flight TX drains, on_trans_done fires, and
                // `rmt_disable` settles in microseconds instead of
                // spinning forever waiting for an infinite-stream
                // encoder. Without this flag, stopping mid-wind hangs
                // the calling CPU in `rmt_tx_disable`'s poll loop
                // (interrupts disabled) → CPU IWDT panic.
                std::atomic<bool> stopRequested{ false };
        };

        inline void applySlots(StepEncoder *self, const rmtgen::SymbolSlots &s)
        {
                self->currentSymbol.level0 = s.level0;
                self->currentSymbol.duration0 = s.duration0;
                self->currentSymbol.level1 = s.level1;
                self->currentSymbol.duration1 = s.duration1;
        }

        // Pull the next symbol from the expander into `currentSymbol`. Returns
        // false (and clears symbolReady) once the move is fully expanded.
        inline bool loadNextSymbol(StepEncoder *self)
        {
                if (self->expander.done()) {
                        self->symbolReady = false;
                        return false;
                }
                bool stepDone = false;
                applySlots(self, self->expander.next(stepDone));
                self->currentStepCompletes = stepDone;
                self->symbolReady = true;
                return true;
        }

        // Main encode entry — called by the RMT driver to refill the
        // channel's memory block. Emits as many symbols as fit, then
        // returns either MEM_FULL (driver will call us again later) or
        // COMPLETE (all segments done, no more symbols).
        size_t IRAM_ATTR stepEncode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void * /*primary_data*/, size_t /*data_size*/,
                                    rmt_encode_state_t *ret_state)
        {
                auto *self = __containerof(encoder, StepEncoder, base);
                rmt_encode_state_t state = static_cast<rmt_encode_state_t>(0);
                size_t totalEncoded = 0;

                // Task asked us to stop. Drop whatever's left in this
                // move and report COMPLETE so the channel can drain
                // and `rmt_disable` returns promptly.
                if (self->stopRequested.load(std::memory_order_acquire)) {
                        *ret_state = RMT_ENCODING_COMPLETE;
                        return 0;
                }
                // First entry of a fresh move (or empty move): prime the first
                // symbol. loadNextSymbol clears symbolReady when there is none.
                if (!self->symbolReady && !loadNextSymbol(self)) {
                        *ret_state = RMT_ENCODING_COMPLETE;
                        return 0;
                }

                while (self->symbolReady) {
                        // Push one symbol via the inner copy encoder.
                        rmt_encode_state_t innerState = static_cast<rmt_encode_state_t>(0);
                        const size_t pushed = self->copyEncoder->encode(self->copyEncoder, channel,
                                                                        &self->currentSymbol,
                                                                        sizeof(self->currentSymbol),
                                                                        &innerState);
                        totalEncoded += pushed;

                        if (innerState & RMT_ENCODING_COMPLETE) {
                                // One symbol fully pushed; reset the inner
                                // encoder for the next push.
                                self->copyEncoder->reset(self->copyEncoder);
                                // A step is committed only when its LAST symbol
                                // lands (a wide step spans several symbols).
                                if (self->currentStepCompletes) {
                                        // Cumulative counter — wraps at
                                        // UINT32_MAX, intentionally.
                                        self->stepsEmittedSinceArm.fetch_add(1u, std::memory_order_relaxed);
                                }
                                // Fetch the next symbol; if the move is done,
                                // loadNextSymbol clears symbolReady and the loop
                                // exits → COMPLETE below.
                                (void)loadNextSymbol(self);
                        }
                        if (innerState & RMT_ENCODING_MEM_FULL) {
                                state =
                                    static_cast<rmt_encode_state_t>(state | RMT_ENCODING_MEM_FULL);
                                break;
                        }
                        if (pushed == 0 && !(innerState & RMT_ENCODING_COMPLETE)) {
                                // Inner encoder couldn't push — bail
                                // to avoid an infinite loop.
                                state =
                                    static_cast<rmt_encode_state_t>(state | RMT_ENCODING_MEM_FULL);
                                break;
                        }
                }

                if (!self->symbolReady) {
                        state = static_cast<rmt_encode_state_t>(state | RMT_ENCODING_COMPLETE);
                }
                *ret_state = state;
                return totalEncoded;
        }

        esp_err_t stepEncoderDel(rmt_encoder_t *encoder)
        {
                auto *self = __containerof(encoder, StepEncoder, base);
                if (self->copyEncoder != nullptr) {
                        rmt_del_encoder(self->copyEncoder);
                }
                delete self;
                return ESP_OK;
        }

        esp_err_t stepEncoderReset(rmt_encoder_t *encoder)
        {
                auto *self = __containerof(encoder, StepEncoder, base);
                if (self->copyEncoder != nullptr) {
                        rmt_encoder_reset(self->copyEncoder);
                }
                self->expander.reset(self->move);
                self->symbolReady = false;
                self->currentStepCompletes = false;
                self->stepsEmittedSinceArm.store(0u, std::memory_order_relaxed);
                self->stopRequested.store(false, std::memory_order_relaxed);
                return ESP_OK;
        }

        // Allocate the encoder and its inner copy encoder. Returns nullptr
        // on failure.
        StepEncoder *createStepEncoder()
        {
                auto *enc = new (std::nothrow) StepEncoder();
                if (enc == nullptr) {
                        return nullptr;
                }
                enc->base.encode = &stepEncode;
                enc->base.del = &stepEncoderDel;
                enc->base.reset = &stepEncoderReset;

                rmt_copy_encoder_config_t cfg = {};
                if (rmt_new_copy_encoder(&cfg, &enc->copyEncoder) != ESP_OK) {
                        delete enc;
                        return nullptr;
                }
                return enc;
        }

} // namespace

// =====================================================================
// Construction / destruction
// =====================================================================

RmtStepSignal::RmtStepSignal()
        : RmtStepSignal(Config{})
{
}

RmtStepSignal::RmtStepSignal(const Config &cfg)
        : cfg_(cfg)
{
}

RmtStepSignal::~RmtStepSignal()
{
        if (begun_) {
                end();
        }
}

// =====================================================================
// Lifecycle
// =====================================================================

Status RmtStepSignal::begin(uint8_t stepPin, uint8_t dirPin, bool dirActiveHigh,
                            uint32_t dirSetupUs, uint32_t minPulseHighUs, uint32_t minPulseLowUs)
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (stepPin == 0xFFu || dirPin == 0xFFu || cfg_.resolutionHz == 0) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        stepPin_ = stepPin;
        dirPin_ = dirPin;
        dirActiveHigh_ = dirActiveHigh;
        dirSetupUs_ = dirSetupUs;
        minPulseHighUs_ = minPulseHighUs;
        minPulseLowUs_ = minPulseLowUs;

        // DIR pin is plain GPIO — the host writes it before each move.
        if (!gpio::configOutput(dirPin_)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        gpio::write(dirPin_, dirActiveHigh_ ? false : true); // idle = non-forward

        // Allocate the RMT TX channel. The driver takes ownership of
        // the STEP pin for the channel's lifetime.
        rmt_tx_channel_config_t txCfg = {};
        txCfg.gpio_num = static_cast<gpio_num_t>(stepPin_);
        txCfg.clk_src = RMT_CLK_SRC_DEFAULT;
        txCfg.resolution_hz = cfg_.resolutionHz;
        txCfg.mem_block_symbols = cfg_.memBlockSymbols;
        txCfg.trans_queue_depth = cfg_.transQueueDepth;
        txCfg.flags.invert_out = 0;
        txCfg.flags.with_dma = 0; // small payloads, no DMA needed
        txCfg.flags.io_loop_back = 0;
        txCfg.flags.io_od_mode = 0;

        rmt_channel_handle_t channel = nullptr;
        if (rmt_new_tx_channel(&txCfg, &channel) != ESP_OK || channel == nullptr) {
                return Status::Err(ErrorCode::DriverFault);
        }

        // Custom step encoder — walks PlannedMove segments and emits
        // one symbol per step. ESP32 classic doesn't support
        // `rmt_transmit_config_t::loop_count`, so this is the only
        // way to play arbitrary-length step trains.
        StepEncoder *stepEncoder = createStepEncoder();
        if (stepEncoder == nullptr) {
                rmt_del_channel(channel);
                return Status::Err(ErrorCode::DriverFault);
        }
        rmt_encoder_handle_t encoderHandle = &stepEncoder->base;

        // Register the on-done callback so we can track motion
        // completion. ESP-IDF's `rmt_tx_done_callback_t` uses the
        // strongly typed `rmt_channel_handle_t` /
        // `rmt_tx_done_event_data_t*` signature; our public callback
        // is declared with `void*` parameters to keep ESP-IDF types
        // out of the public header. Bridge via a local typed shim
        // that casts and forwards to the void* method.
        struct CbBridge {
                static bool IRAM_ATTR onDone(rmt_channel_handle_t ch,
                                             const rmt_tx_done_event_data_t *edata, void *userCtx)
                {
                        return RmtStepSignal::onTxDoneCallback(
                            static_cast<void *>(ch), static_cast<const void *>(edata), userCtx);
                }
        };
        rmt_tx_event_callbacks_t cbs = {};
        cbs.on_trans_done = &CbBridge::onDone;
        if (rmt_tx_register_event_callbacks(channel, &cbs, this) != ESP_OK) {
                rmt_del_encoder(encoderHandle);
                rmt_del_channel(channel);
                return Status::Err(ErrorCode::DriverFault);
        }

        if (rmt_enable(channel) != ESP_OK) {
                rmt_del_encoder(encoderHandle);
                rmt_del_channel(channel);
                return Status::Err(ErrorCode::DriverFault);
        }

        channel_ = channel;
        encoder_ = encoderHandle;
        begun_ = true;
        return Status::Ok();
}

void RmtStepSignal::end()
{
        if (!begun_) {
                return;
        }
        if (running_.load(std::memory_order_acquire)) {
                (void)stop(StopMode::Immediate);
        }
        if (channel_ != nullptr) {
                rmt_disable(static_cast<rmt_channel_handle_t>(channel_));
                rmt_del_channel(static_cast<rmt_channel_handle_t>(channel_));
                channel_ = nullptr;
        }
        if (encoder_ != nullptr) {
                rmt_del_encoder(static_cast<rmt_encoder_handle_t>(encoder_));
                encoder_ = nullptr;
        }
        begun_ = false;
        running_.store(false, std::memory_order_release);
        faulted_.store(false, std::memory_order_release);
        ignoreNextDone_.store(false, std::memory_order_release);
        pendingSegments_.store(0, std::memory_order_release);
}

// =====================================================================
// Motion
// =====================================================================

Status RmtStepSignal::sendSegment(const MotionSegment & /*seg*/)
{
        // Unused on ESP-IDF — the custom encoder consumes segments
        // directly from the StepEncoder's stored move. Kept on the
        // class signature so the host stub path can link.
        return Status::Err(ErrorCode::Unsupported);
}

Status RmtStepSignal::armMove(const PlannedMove &move)
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
        // A zero half-period is a planner bug (a step with no period) — the
        // encoder can't emit it and a 0-duration RMT slot is an end-marker.
        // Half-periods ABOVE the 15-bit slot (0x7FFF ticks) are fine: the
        // encoder splits those steps across several symbols (see
        // StepSymbolCursor), so high generator resolutions (e.g. 10 MHz for
        // the YPMC servo) keep working at the slow ends of a ramp.
        for (uint8_t i = 0; i < move.segmentCount; ++i) {
                if (move.segments[i].halfPeriodTicks == 0) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
        }

        // Hand the move to the custom encoder. The encoder owns a
        // copy and walks it across many encode() callbacks as the
        // RMT driver drains its memory block.
        auto *stepEnc = __containerof(static_cast<rmt_encoder_t *>(encoder_), StepEncoder, base);
        stepEnc->move = move;
        stepEnc->stepsEmittedSinceArm.store(0u, std::memory_order_release);
        // Clear any leftover abort from a prior stop() — the encoder is
        // free to emit again. Done BEFORE rmt_encoder_reset so the
        // reset hook sees a clean slate.
        stepEnc->stopRequested.store(false, std::memory_order_release);
        // rmt_encoder_reset() runs stepEncoderReset(), which points the
        // expander at the freshly-assigned `move` and clears symbol state.
        rmt_encoder_reset(static_cast<rmt_encoder_handle_t>(encoder_));

        // Snapshot the pre-arm position + direction sign. From here
        // until on_done commits, commandedPosition() is computed
        // live as `baselinePosition_ + sign * encoder.emitted`.
        baselinePosition_.store(commandedPosition_.load(std::memory_order_acquire),
                                std::memory_order_release);
        directionSign_.store((move.direction == Direction::Forward) ? static_cast<int8_t>(1) :
                                                                      static_cast<int8_t>(-1),
                             std::memory_order_release);

        // Write DIR + wait setup window — task-context, before the
        // first RMT edge fires.
        const bool wantHigh = (move.direction == Direction::Forward) ? dirActiveHigh_ :
                                                                       !dirActiveHigh_;
        gpio::write(dirPin_, wantHigh);
        if (dirSetupUs_ > 0) {
                ungula::core::time::delayUs(static_cast<int64_t>(dirSetupUs_));
        }

        finishedReason_.store(StopReason::None, std::memory_order_release);
        commandedSpsNow_.store(move.cruiseSps, std::memory_order_release);
        // ONE transmission, not one per segment — the encoder walks
        // segments internally. on_trans_done fires once when the
        // encoder reports COMPLETE.
        pendingSegments_.store(1, std::memory_order_release);
        running_.store(true, std::memory_order_release);

        // The encoder ignores `primary_data` (its move lives in the
        // StepEncoder struct). ESP-IDF still requires a non-null /
        // non-zero pair, so we pass the move pointer + size for
        // debug-friendliness.
        rmt_transmit_config_t txCfg = {};
        txCfg.loop_count = 0; // ESP32 classic doesn't support looping
        txCfg.flags.eot_level = 0;
        txCfg.flags.queue_nonblocking = 0;

        const auto err = rmt_transmit(static_cast<rmt_channel_handle_t>(channel_),
                                      static_cast<rmt_encoder_handle_t>(encoder_), &stepEnc->move,
                                      sizeof(stepEnc->move), &txCfg);
        if (err != ESP_OK) {
                running_.store(false, std::memory_order_release);
                pendingSegments_.store(0, std::memory_order_release);
                finishedReason_.store(StopReason::DriverFault, std::memory_order_release);
                return Status::Err(ErrorCode::DriverFault);
        }

        // NOTE: commandedPosition_ is committed at motion end (on_done)
        // or stop(). During motion, commandedPosition() computes live
        // position from baseline + emitted steps.
        return Status::Ok();
}

namespace
{

        // Snapshot the encoder's emitted counter, fold it into the baseline
        // + sign to give a signed step position. Wrap-safe: arithmetic is
        // done in uint32 (well-defined modular wrap); the int32 cast at the
        // end reinterprets the bit pattern. With a long indefinite jog the
        // position will roll past INT32_MAX into the negative half of int32
        // and continue past 0 — that's the intended "counter is allowed to
        // wrap" behaviour.
        int32_t computeLivePosition(const StepEncoder *enc, int32_t baseline, int8_t sign)
        {
                const uint32_t emitted = enc->stepsEmittedSinceArm.load(std::memory_order_acquire);
                const uint32_t baseU = static_cast<uint32_t>(baseline);
                const uint32_t deltaU = (sign >= 0) ? emitted : (0u - emitted); // unsigned negate
                const uint32_t posU = baseU + deltaU; // mod 2^32, defined
                return static_cast<int32_t>(posU);
        }

} // namespace

Status RmtStepSignal::stop(StopMode mode)
{
        if (!begun_) {
                return Status::Err(ErrorCode::NotInitialized);
        }
        if (mode == StopMode::Decelerate) {
                // Synthesised decel ramps aren't supported by this
                // generator yet — the queued segments must drain or
                // be cancelled. Phase 2 follow-up: chain a planned
                // decel move on top of the in-flight one.
                return Status::Err(ErrorCode::Unsupported);
        }
        if (!running_.load(std::memory_order_acquire)) {
                return Status::Ok();
        }

        // Capture the live position BEFORE clobbering the channel:
        // after stop() the host expects commandedPosition() to
        // reflect where the motor was when we cut the pulses.
        if (encoder_ != nullptr) {
                auto *stepEnc =
                    __containerof(static_cast<rmt_encoder_t *>(encoder_), StepEncoder, base);
                const int32_t live =
                    computeLivePosition(stepEnc, baselinePosition_.load(std::memory_order_acquire),
                                        directionSign_.load(std::memory_order_acquire));
                commandedPosition_.store(live, std::memory_order_release);
                // Tell the encoder to stop feeding symbols. The next
                // `stepEncode()` call returns COMPLETE, the channel
                // drains its tiny remaining queue, and `rmt_disable`
                // settles in microseconds. Without this raise,
                // `rmt_disable` spins forever polling for "channel
                // idle" (interrupts disabled the entire time) →
                // CPU IWDT panic on any stop mid-wind.
                stepEnc->stopRequested.store(true, std::memory_order_release);
        }

        // Hard halt: disable and re-enable the channel. RMT's disable
        // drops every queued transmission AND the in-flight one;
        // re-enable preps the channel for the next armMove.
        rmt_disable(static_cast<rmt_channel_handle_t>(channel_));
        rmt_enable(static_cast<rmt_channel_handle_t>(channel_));

        running_.store(false, std::memory_order_release);
        pendingSegments_.store(0, std::memory_order_release);
        commandedSpsNow_.store(0, std::memory_order_release);
        finishedReason_.store(StopReason::UserStop, std::memory_order_release);
        // The halted transmission's on_trans_done still fires (the encoder was told
        // to COMPLETE) — flag it so that callback is dropped and can't clobber a
        // move armed before it lands.
        ignoreNextDone_.store(true, std::memory_order_release);
        return Status::Ok();
}

// =====================================================================
// Status / position
// =====================================================================

StepSignalStatus RmtStepSignal::status() const
{
        StepSignalStatus s;
        s.running = running_.load(std::memory_order_acquire);
        s.faulted = faulted_.load(std::memory_order_acquire);
        s.finishedReason = finishedReason_.load(std::memory_order_acquire);
        return s;
}

Position RmtStepSignal::commandedPosition() const
{
        // During motion, compute live from baseline + emitted so the
        // host sees the counter advance even on a never-ending jog.
        // When idle, return the last committed value (set by
        // resetPosition or by the on_done / stop commit).
        if (running_.load(std::memory_order_acquire) && encoder_ != nullptr) {
                const auto *stepEnc =
                    __containerof(static_cast<rmt_encoder_t *>(encoder_), StepEncoder, base);
                return computeLivePosition(stepEnc,
                                           baselinePosition_.load(std::memory_order_acquire),
                                           directionSign_.load(std::memory_order_acquire));
        }
        return commandedPosition_.load(std::memory_order_acquire);
}

uint32_t RmtStepSignal::commandedSpsNow() const
{
        return running_.load(std::memory_order_acquire) ?
                   commandedSpsNow_.load(std::memory_order_acquire) :
                   0u;
}

Status RmtStepSignal::resetPosition(Position newSteps)
{
        if (running_.load(std::memory_order_acquire)) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        commandedPosition_.store(newSteps, std::memory_order_release);
        return Status::Ok();
}

Status RmtStepSignal::clearFault()
{
        if (running_.load(std::memory_order_acquire)) {
                return Status::Err(ErrorCode::MotionInProgress);
        }
        faulted_.store(false, std::memory_order_release);
        finishedReason_.store(StopReason::None, std::memory_order_release);
        return Status::Ok();
}

// =====================================================================
// ISR trampoline — runs from the RMT TX-done event in IRAM context
// =====================================================================

bool IRAM_ATTR RmtStepSignal::onTxDoneCallback(void * /*channel*/, const void * /*edata*/,
                                               void *userCtx)
{
        auto *self = static_cast<RmtStepSignal *>(userCtx);
        if (self == nullptr) {
                return false;
        }
        // Drop the completion of a stopped transmission — its `running_`/position
        // were already settled by stop(), and acting here would clobber a move
        // armed in the meantime.
        if (self->ignoreNextDone_.exchange(false, std::memory_order_acq_rel)) {
                return false;
        }
        const int32_t remaining = self->pendingSegments_.fetch_sub(1, std::memory_order_acq_rel);
        if (remaining <= 1) {
                // Last queued segment finished — motion is done.
                // Commit live position into the steady-state counter
                // BEFORE flipping `running_` to false, otherwise a
                // racing reader would briefly see the stale baseline.
                if (self->encoder_ != nullptr) {
                        const auto *stepEnc = __containerof(
                            static_cast<rmt_encoder_t *>(self->encoder_), StepEncoder, base);
                        const int32_t live = computeLivePosition(
                            stepEnc, self->baselinePosition_.load(std::memory_order_acquire),
                            self->directionSign_.load(std::memory_order_acquire));
                        self->commandedPosition_.store(live, std::memory_order_release);
                }
                self->running_.store(false, std::memory_order_release);
                self->commandedSpsNow_.store(0, std::memory_order_release);
                self->finishedReason_.store(StopReason::TargetReached, std::memory_order_release);
        }
        // No task to wake from this ISR — the host polls `status()`
        // each service tick.
        return false;
}

} // namespace ungula::motor

#else // Non-ESP32 host build

namespace ungula::motor
{

// Host stubs — the class declaration must link, but every method
// returns NotInitialized. Host tests use FakeStepSignal, not this
// class, so the stubs only exist to keep the lib compilable on host.

RmtStepSignal::RmtStepSignal()
        : RmtStepSignal(Config{})
{
}
RmtStepSignal::RmtStepSignal(const Config &cfg)
        : cfg_(cfg)
{
}
RmtStepSignal::~RmtStepSignal() = default;

Status RmtStepSignal::begin(uint8_t, uint8_t, bool, uint32_t, uint32_t, uint32_t)
{
        return Status::Err(ErrorCode::Unsupported);
}
void RmtStepSignal::end()
{
}
Status RmtStepSignal::armMove(const PlannedMove &)
{
        return Status::Err(ErrorCode::Unsupported);
}
Status RmtStepSignal::stop(StopMode)
{
        return Status::Err(ErrorCode::Unsupported);
}
StepSignalStatus RmtStepSignal::status() const
{
        return {};
}
Position RmtStepSignal::commandedPosition() const
{
        return 0;
}
uint32_t RmtStepSignal::commandedSpsNow() const
{
        return 0;
}
Status RmtStepSignal::resetPosition(Position)
{
        return Status::Err(ErrorCode::Unsupported);
}
Status RmtStepSignal::clearFault()
{
        return Status::Err(ErrorCode::Unsupported);
}
bool RmtStepSignal::onTxDoneCallback(void *, const void *, void *)
{
        return false;
}
Status RmtStepSignal::sendSegment(const MotionSegment &)
{
        return Status::Err(ErrorCode::Unsupported);
}

} // namespace ungula::motor

#endif // ESP_PLATFORM
