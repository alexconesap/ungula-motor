// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/limits/limit_system.h"

#include "ungula/core/time/time.h"
#include "ungula/hal/core/compiler_attrs.h"
#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;

bool LimitSystem::stopsInIsr(LimitKind k)
{
        // Kinds whose ISR directly halts the engine. TravelLimit does
        // NOT — its ISR only latches a per-slot flag; the actual stop
        // is issued task-side after the direction-aware check, which
        // prevents a noise spike on the "wrong" switch from halting
        // valid motion in the opposite direction.
        return k == LimitKind::EmergencyLimit || k == LimitKind::StallSensor;
}

bool LimitSystem::attachesIsr(LimitKind k)
{
        // Kinds whose pin needs a GPIO interrupt attached. Everything
        // that fires asynchronously qualifies — including TravelLimit
        // since 1.0.2, so the lib can catch hand-tap presses that fall
        // entirely between two FreeRTOS ticks.
        return k == LimitKind::EmergencyLimit || k == LimitKind::StallSensor ||
               k == LimitKind::TravelLimit;
}

bool LimitSystem::readActive(const LimitWiring &cfg)
{
        const bool level = gpio::read(cfg.pin);
        return cfg.polarity == SwitchPolarity::NormallyClosed ? !level : level;
}

UNGULA_ISR_ATTR void LimitSystem::onIsrTrampoline(void *ctx)
{
        auto *binding = static_cast<IsrBinding *>(ctx);
        if (!binding || !binding->bank) {
                return;
        }
        LimitSystem *self = binding->bank;

        switch (binding->kind) {
        case LimitKind::EmergencyLimit:
                if (self->engineForIsr_ != nullptr) {
                        self->engineForIsr_->stop(StopMode::Immediate);
                }
                self->emergencyLatched_.store(true, std::memory_order_release);
                break;
        case LimitKind::TravelLimit:
                // Only latch — direction filtering happens task-side
                // in `pumpLimits`, which issues the actual stop if the
                // limit guards the current motion direction. Stopping
                // here would halt motion that is reversing OFF the
                // switch (the pin is still HIGH, but no rising edge
                // fires; this branch only runs on the ASSERTING edge
                // so we still need the polled deactivation path to
                // clear `stableActive`).
                if (binding->slotIndex < MAX_LIMIT_INPUTS) {
                        self->slots_[binding->slotIndex].isrEdgeLatched.store(
                            true, std::memory_order_release);
                }
                break;
        case LimitKind::StallSensor: {
                // Always count: the debounce counter feeds the
                // task-side noise filter; the cumulative counter is a
                // host-visible diagnostic for "is DIAG firing at all?".
                const uint32_t prevCount =
                    self->stallHitCounter_.fetch_add(1, std::memory_order_acq_rel);
                self->stallHitsTotal_.fetch_add(1, std::memory_order_acq_rel);

                const int64_t armedAt = self->stallArmedAtMs_.load(std::memory_order_acquire);
                if (armedAt == 0) {
                        break; // not armed — service will discard the count
                }
                const int64_t now = ungula::core::time::millis();
                if (now - armedAt < static_cast<int64_t>(self->stallArmDelayMs_)) {
                        break; // inside arm window
                }

                // Once a stall has been latched for this motion the
                // engine is already faulted; re-halting on subsequent
                // edges is noise.
                if (self->stallLatched_.load(std::memory_order_acquire)) {
                        break;
                }
                if (static_cast<uint32_t>(prevCount + 1) <
                    static_cast<uint32_t>(self->stallHitsToTrigger_)) {
                        break;
                }

                if (self->engineForIsr_ != nullptr) {
                        self->engineForIsr_->stop(StopMode::Immediate);
                }
                self->stallLatched_.store(true, std::memory_order_release);
                self->stallHitCounter_.store(0, std::memory_order_release);
                break;
        }
        default:
                break; // non-ISR kinds reach here only via test simulation
        }
}

Status LimitSystem::begin(const LimitWiring (&wirings)[MAX_LIMIT_INPUTS],
                          IStepSignalGenerator *engineForIsr)
{
        // Auto-count: pin == GPIO_NONE is the lib's "unused slot"
        // marker. Walk the array, find the highest index with a pin
        // set, pass count to the explicit-count overload. Hosts no
        // longer track the row count by hand.
        uint8_t count = 0;
        for (uint8_t i = 0; i < MAX_LIMIT_INPUTS; ++i) {
                if (wirings[i].pin != GPIO_NONE) {
                        count = static_cast<uint8_t>(i + 1u);
                }
        }
        return begin(&wirings[0], count, engineForIsr);
}

Status LimitSystem::begin(const LimitWiring *wirings, uint8_t count,
                          IStepSignalGenerator *engineForIsr)
{
        if (begun_) {
                return Status::Err(ErrorCode::AlreadyInitialized);
        }
        if (count > MAX_LIMIT_INPUTS) {
                return Status::Err(ErrorCode::InvalidConfig);
        }

        engineForIsr_ = engineForIsr;

        bool seenHome = false;
        bool seenStall = false;
        bool anyIsrKind = false;

        for (uint8_t i = 0; i < count; ++i) {
                const auto &cfg = wirings[i];
                if (cfg.pin == GPIO_NONE) {
                        continue;
                }

                if (cfg.kind == LimitKind::HomeSensor) {
                        if (seenHome) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        seenHome = true;
                }
                if (cfg.kind == LimitKind::StallSensor) {
                        if (seenStall) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        seenStall = true;
                        stallHitsToTrigger_ = cfg.stallHitsToTrigger;
                        stallArmDelayMs_ = cfg.stallArmDelayMs;
                }
                if (stopsInIsr(cfg.kind)) {
                        // Only kinds whose ISR halts the engine
                        // require an engine reference. TravelLimit
                        // attaches an ISR for fast edge-latching but
                        // does the actual stop task-side, so it can
                        // run with `engineForIsr == nullptr`.
                        if (engineForIsr_ == nullptr) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                }
                if (attachesIsr(cfg.kind)) {
                        anyIsrKind = true;
                }

                Slot &s = slots_[slotCount_++];
                s.cfg = cfg;
                s.inUse = true;
                s.stableActive = false;
                s.candidateActive = false;
                s.candidateSinceMs = 0;
                s.isrEdgeLatched.store(false, std::memory_order_relaxed);

                // Resolve pull mode: Hardware/Input -> NONE, Internal/Polarity -> infer from polarity
                gpio::PullMode pull;
                switch (cfg.pullMode) {
                case LimitPinPullMode::McU:
                case LimitPinPullMode::Polarity:
                        pull = (cfg.polarity == SwitchPolarity::NormallyClosed) ?
                                   gpio::PullMode::UP :
                                   gpio::PullMode::DOWN;
                        break;
                case LimitPinPullMode::InternalPullUp:
                        pull = gpio::PullMode::UP;
                        break;
                case LimitPinPullMode::InternalPullDown:
                        pull = gpio::PullMode::DOWN;
                        break;
                case LimitPinPullMode::HardwareResistors:
                case LimitPinPullMode::Input:
                default:
                        pull = gpio::PullMode::NONE;
                        break;
                }

                if (attachesIsr(cfg.kind)) {
                        const auto edge = (cfg.polarity == SwitchPolarity::NormallyClosed) ?
                                              gpio::InterruptEdge::EDGE_FALLING :
                                              gpio::InterruptEdge::EDGE_RISING;
                        if (!gpio::configInputInterrupt(cfg.pin, edge, pull)) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                } else {
                        bool ok = false;
                        switch (pull) {
                        case gpio::PullMode::UP:
                                ok = gpio::configInputPullup(cfg.pin);
                                break;
                        case gpio::PullMode::DOWN:
                                ok = gpio::configInputPulldown(cfg.pin);
                                break;
                        default:
                                ok = gpio::configInput(cfg.pin);
                                break;
                        }
                        if (!ok) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                }
        }

        if (anyIsrKind) {
                if (gpio::installIsrService() == gpio::IsrServiceInstall::Failed) {
                        end();
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                for (uint8_t i = 0; i < slotCount_; ++i) {
                        const auto &s = slots_[i];
                        if (!s.inUse || !attachesIsr(s.cfg.kind)) {
                                continue;
                        }
                        IsrBinding &b = isrBindings_[isrBindingCount_++];
                        b.bank = this;
                        b.pin = s.cfg.pin;
                        b.kind = s.cfg.kind;
                        b.slotIndex = i;
                        if (!gpio::addIsrHandler(s.cfg.pin, &onIsrTrampoline, &b)) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                }
        }

        // Boot-time level seed for TravelLimit. Edge interrupts miss
        // the case where the switch is ALREADY pressed at the moment
        // `begin()` finishes — the rising edge happened earlier and is
        // gone for good. A one-shot live read after the ISR is armed
        // catches that case, so `armJog` / `armMove` correctly refuse
        // to drive INTO the already-pressed limit. (EmergencyLimit and
        // StallSensor have their own task-side level-trip recovery,
        // documented next to the stall path below.)
        for (uint8_t i = 0; i < slotCount_; ++i) {
                Slot &s = slots_[i];
                if (!s.inUse || s.cfg.kind != LimitKind::TravelLimit) {
                        continue;
                }
                if (readActive(s.cfg)) {
                        s.stableActive = true;
                        s.candidateActive = true;
                        s.candidateSinceMs = 0;
                }
        }

        begun_ = true;
        return Status::Ok();
}

void LimitSystem::end()
{
        for (uint8_t i = 0; i < isrBindingCount_; ++i) {
                const auto &b = isrBindings_[i];
                if (b.pin != GPIO_NONE) {
                        (void)gpio::removeIsrHandler(b.pin);
                }
        }
        isrBindingCount_ = 0;

        // Slot is non-copy-assignable (it owns an std::atomic for the
        // edge-latch flag), so we reset fields explicitly instead of
        // `s = Slot{}`.
        for (auto &s : slots_) {
                s.cfg = LimitWiring{};
                s.inUse = false;
                s.stableActive = false;
                s.candidateActive = false;
                s.candidateSinceMs = 0;
                s.isrEdgeLatched.store(false, std::memory_order_relaxed);
                s.edgeArmed = false;
        }
        slotCount_ = 0;
        engineForIsr_ = nullptr;
        emergencyLatched_.store(false, std::memory_order_release);
        stallLatched_.store(false, std::memory_order_release);
        stallHitCounter_.store(0, std::memory_order_release);
        stallHitsToTrigger_ = 1; // matches LimitWiring default; level-held DIAG
        stallArmDelayMs_ = 200;
        stallArmedAtMs_.store(0, std::memory_order_release);
        stallHitsTotal_.store(0, std::memory_order_release);
        begun_ = false;
}

void LimitSystem::service(int64_t nowMs)
{
        if (!begun_) {
                return;
        }

        for (uint8_t i = 0; i < slotCount_; ++i) {
                Slot &s = slots_[i];
                if (!s.inUse) {
                        continue;
                }

                // StallSensor: edge-counting lives in the ISR (fast),
                // but the ISR can only see RISING transitions after
                // it has been attached. The TMC2209's DIAG is
                // level-held while stalled; if it's already HIGH at
                // the moment `LimitSystem::begin()` finishes attaching
                // the interrupt, the rising edge has already happened
                // and is gone for good. This polled path catches
                // that case: if the stall pin reads active past the
                // arm window and the latch isn't already set, halt
                // the engine the same way the ISR would.
                if (s.cfg.kind == LimitKind::StallSensor) {
                        const bool active = readActive(s.cfg);
                        if (!active) {
                                // Pin dropped - reset the level timer
                                // so a future re-assert needs to hold
                                // continuously for the full debounce.
                                s.candidateActive = false;
                                s.candidateSinceMs = 0;
                                continue;
                        }
                        if (stallLatched_.load(std::memory_order_acquire)) {
                                continue;
                        }
                        const int64_t armedAt = stallArmedAtMs_.load(std::memory_order_acquire);
                        if (armedAt == 0) {
                                // Not armed - motion not in flight, or
                                // motionEnd was called. Reset the timer
                                // so re-arming starts a fresh window.
                                s.candidateActive = false;
                                s.candidateSinceMs = 0;
                                continue;
                        }
                        if (nowMs - armedAt < static_cast<int64_t>(stallArmDelayMs_)) {
                                continue; // inside arm window
                        }
                        // Level-time debounce. DIAG must read active
                        // continuously for `debounceMs` before we
                        // latch. Without this any single tick where
                        // the chip briefly drops SG_RESULT below trip
                        // (a tight spot, a microstep boundary) would
                        // fault the axis. `debounceMs` is the same
                        // field other polled limits use; 20 ms is a
                        // sensible default. Set it to 0 on the wiring
                        // row for instant latch.
                        if (!s.candidateActive) {
                                s.candidateActive = true;
                                s.candidateSinceMs = nowMs;
                        }
                        if (nowMs - s.candidateSinceMs < static_cast<int64_t>(s.cfg.debounceMs)) {
                                continue; // active but not held long enough
                        }
                        if (engineForIsr_ != nullptr) {
                                (void)engineForIsr_->stop(StopMode::Immediate);
                        }
                        stallLatched_.store(true, std::memory_order_release);
                        // Bump the visible hit counter so host logs
                        // and `totalStallHits()` show the level-trip
                        // even when no rising edge ever fired.
                        stallHitsTotal_.fetch_add(1, std::memory_order_acq_rel);
                        s.candidateActive = false;
                        s.candidateSinceMs = 0;
                        continue;
                }

                if (s.cfg.kind == LimitKind::TravelLimit) {
                        // TravelLimit activation/deactivation model
                        // (1.0.3+, after field testing exposed two
                        // failure modes in earlier 1.0.2 iterations).
                        //
                        // ACTIVATION — two-tick confirmation
                        //   The asserting GPIO edge fires the ISR,
                        //   which sets `isrEdgeLatched`. Task-side,
                        //   `service()` copies that flag into the
                        //   per-slot `edgeArmed` so the hint persists
                        //   across multiple ticks. Promotion to
                        //   `stableActive=true` requires:
                        //     - `edgeArmed == true`     (the ISR saw
                        //       an asserting edge at some point), AND
                        //     - polled read at THIS tick is active,
                        //       AND
                        //     - polled read at the PREVIOUS tick was
                        //       also active (mirrored in
                        //       `s.candidateActive` before we update
                        //       it from the live read this tick).
                        //   The two-tick requirement is what gives
                        //   the lib noise immunity. An EMI spike
                        //   that fires the ISR can also happen to
                        //   land HIGH inside a single polled read by
                        //   bad luck — but for the spike to land
                        //   HIGH on TWO consecutive polled reads
                        //   (separated by one FreeRTOS tick ≈ 10 ms)
                        //   it would have to be a sustained
                        //   disturbance, not a transient.
                        //
                        //   `edgeArmed` is disarmed by a single
                        //   inactive read (the edge was a noise
                        //   spike — discard) or by successful
                        //   promotion (motion already halted).
                        //
                        // ACTIVATION — slow polled fallback
                        //   If the ISR is missed entirely (e.g. the
                        //   asserting edge predated `begin()`'s
                        //   `addIsrHandler` call), the polled path
                        //   still latches the limit after the pin
                        //   has read active continuously for
                        //   `debounceMs`. Belt-and-suspenders.
                        //
                        // DEACTIVATION
                        //   `stableActive` clears only after the pin
                        //   has read inactive continuously for
                        //   `debounceMs`. One bounce-high during the
                        //   window restarts the timer.
                        //
                        // LATENCY
                        //   At the typical FreeRTOS tick rate
                        //   (~10 ms), promotion latency for a real
                        //   press is ~one tick (the time between
                        //   the edge firing and the second polled-
                        //   active confirmation). At 25 kSPS that's
                        //   ~250 additional steps before halt. For
                        //   safety-critical axes that need faster
                        //   halt, use `EmergencyLimit` instead —
                        //   it stops the engine directly from the
                        //   ISR with zero polling latency.
                        const bool edge =
                            s.isrEdgeLatched.exchange(false, std::memory_order_acq_rel);
                        const bool active = readActive(s.cfg);
                        const bool prevPolledActive = s.candidateActive;

                        // Update the polled-state transition tracker.
                        // Drives the slow activation fallback and the
                        // deactivation timer.
                        if (active != s.candidateActive) {
                                s.candidateActive = active;
                                s.candidateSinceMs = nowMs;
                        }

                        const int64_t elapsed = nowMs - s.candidateSinceMs;
                        const int64_t debounceMs = static_cast<int64_t>(s.cfg.debounceMs);

                        if (!s.stableActive) {
                                // Fast path - ISR edge + two-tick
                                // active confirmation.
                                if (edge) {
                                        s.edgeArmed = true;
                                }
                                if (s.edgeArmed && active && prevPolledActive) {
                                        s.stableActive = true;
                                        s.edgeArmed = false;
                                } else if (s.edgeArmed && !active) {
                                        // The edge was a noise spike
                                        // — pin is no longer active.
                                        // Disarm so we don't promote
                                        // on a future stale edge.
                                        s.edgeArmed = false;
                                } else if (active && elapsed >= debounceMs) {
                                        // Slow path - polled stable
                                        // for debounceMs. Catches
                                        // the rare case where the
                                        // ISR was missed.
                                        s.stableActive = true;
                                        s.edgeArmed = false;
                                }
                        } else {
                                // Deactivation: pin must read
                                // inactive continuously for
                                // `debounceMs` before we clear.
                                if (!active && elapsed >= debounceMs) {
                                        s.stableActive = false;
                                }
                        }
                        continue;
                }

                if (stopsInIsr(s.cfg.kind)) {
                        continue; // EmergencyLimit: ISR halts + latches; nothing to poll.
                }

                // Remaining polled path: HomeSensor (read by the
                // homing strategy via `isActive`; debounced here so
                // sensor chatter doesn't false-positive the home
                // detection).
                const bool active = readActive(s.cfg);
                if (active != s.candidateActive) {
                        s.candidateActive = active;
                        s.candidateSinceMs = nowMs;
                        continue;
                }
                if (active != s.stableActive) {
                        const int64_t elapsed = nowMs - s.candidateSinceMs;
                        if (elapsed >= static_cast<int64_t>(s.cfg.debounceMs)) {
                                s.stableActive = active;
                        }
                }
        }

        // Stall-counter cleanup: discard counts accumulated while
        // unarmed or inside the arm window. Threshold promotion lives
        // in the ISR.
        const uint32_t hits = stallHitCounter_.load(std::memory_order_acquire);
        if (hits > 0) {
                const int64_t armedAt = stallArmedAtMs_.load(std::memory_order_acquire);
                const bool armed = (armedAt != 0);
                const bool pastWindow = armed &&
                                        (nowMs - armedAt) >= static_cast<int64_t>(stallArmDelayMs_);
                if (!armed || !pastWindow) {
                        stallHitCounter_.store(0, std::memory_order_release);
                }
        }
}

bool LimitSystem::isActive(LimitKind kind) const
{
        if (kind == LimitKind::EmergencyLimit) {
                return emergencyLatched_.load(std::memory_order_acquire);
        }
        if (kind == LimitKind::StallSensor) {
                return stallLatched_.load(std::memory_order_acquire);
        }
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (s.inUse && s.cfg.kind == kind && s.stableActive) {
                        return true;
                }
        }
        return false;
}

bool LimitSystem::isActive(LimitKind kind, Direction dir) const
{
        if (kind == LimitKind::EmergencyLimit) {
                return emergencyLatched_.load(std::memory_order_acquire);
        }
        if (kind == LimitKind::StallSensor) {
                return stallLatched_.load(std::memory_order_acquire);
        }
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (s.inUse && s.cfg.kind == kind && s.cfg.direction == dir && s.stableActive) {
                        return true;
                }
        }
        return false;
}

bool LimitSystem::consumeEmergencyActivation()
{
        return emergencyLatched_.exchange(false, std::memory_order_acq_rel);
}

bool LimitSystem::consumeStallActivation()
{
        return stallLatched_.exchange(false, std::memory_order_acq_rel);
}

void LimitSystem::notifyMotionStart(int64_t nowMs)
{
        stallHitCounter_.store(0, std::memory_order_release);
        stallArmedAtMs_.store(nowMs == 0 ? 1 : nowMs, std::memory_order_release);
}

void LimitSystem::notifyMotionEnd()
{
        stallArmedAtMs_.store(0, std::memory_order_release);
}

uint32_t LimitSystem::totalStallHits() const
{
        return stallHitsTotal_.load(std::memory_order_acquire);
}

void LimitSystem::resetStallHitsTotal()
{
        stallHitsTotal_.store(0, std::memory_order_release);
}

bool LimitSystem::isAssertedLive(LimitKind kind) const
{
        if (!begun_) {
                return false;
        }
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (!s.inUse || s.cfg.kind != kind) {
                        continue;
                }
                if (readActive(s.cfg)) {
                        return true;
                }
        }
        return false;
}

uint8_t LimitSystem::homePin() const
{
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (s.inUse && s.cfg.kind == LimitKind::HomeSensor) {
                        return s.cfg.pin;
                }
        }
        return GPIO_NONE;
}

void LimitSystem::simulateIsrEdgeForTesting(LimitKind kind)
{
        if (!begun_) {
                return;
        }
        for (uint8_t i = 0; i < isrBindingCount_; ++i) {
                if (isrBindings_[i].kind == kind) {
                        onIsrTrampoline(&isrBindings_[i]);
                        return;
                }
        }
}

} // namespace ungula::motor
