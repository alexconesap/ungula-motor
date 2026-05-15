// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/limits/sensor_bank.h"

#include "ungula/core/time/time.h"
#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;

namespace
{

        constexpr bool isIsrRole(SensorRole r)
        {
                return r == SensorRole::CrashLimit || r == SensorRole::EmergencyStop ||
                       r == SensorRole::Stall;
        }

} // namespace

bool SensorBank::readActive(const SensorInputConfig &cfg)
{
        const bool level = gpio::read(cfg.pin);
        // NormallyClosed: pressed reads LOW (active = !level).
        // NormallyOpen:   pressed reads HIGH (active = level).
        return cfg.polarity == SensorPolarity::NormallyClosed ? !level : level;
}

UNGULA_ISR_ATTR void SensorBank::onIsrTrampoline(void *ctx)
{
        auto *binding = static_cast<IsrBinding *>(ctx);
        if (!binding || !binding->bank)
                return;

        SensorBank *self = binding->bank;

        // For E-stop and crash limits: halt FIRST. Latency on these
        // paths is the whole point of wiring an ISR — drop everything
        // else, get the steps to stop, then bookkeep.
        //
        // For Stall: halt only if the arm window has elapsed. Inside
        // the arm window (StealthChop auto-tune transient, etc.) we
        // just count the hit; halting there would latch the engine
        // fault and force the host to `clearFault()` on every motor
        // bring-up. Real stalls past the window still halt instantly.
        switch (binding->role) {
        case SensorRole::EmergencyStop:
                if (self->engineForIsr_) {
                        self->engineForIsr_->haltFromIsr(StopReason::EmergencyStop);
                }
                self->estopLatched_.store(true, std::memory_order_release);
                break;
        case SensorRole::Stall: {
                // Always count: the debounce counter feeds the
                // task-side noise filter; the cumulative counter is a
                // host-visible diagnostic for "is DIAG firing at all?".
                const uint32_t prevCount = self->stallHitCounter_.fetch_add(
                        1, std::memory_order_acq_rel);
                self->stallHitsTotal_.fetch_add(1, std::memory_order_acq_rel);

                const int64_t armedAt =
                        self->stallArmedAtMs_.load(std::memory_order_acquire);
                if (armedAt == 0)
                        break; // not armed — service will discard the count
                const int64_t now = ungula::core::time::millis();
                if (now - armedAt <
                    static_cast<int64_t>(self->stallArmDelayMs_)) {
                        break; // inside arm window
                }

                // Once a stall has been latched for this motion, the
                // engine is already faulted — re-halting on subsequent
                // edges is noise. The latch is reset by
                // `consumeStallActivation()` from the task path.
                if (self->stallLatched_.load(std::memory_order_acquire))
                        break;

                // Gated halt — only fire when the counter reaches the
                // configured threshold. `prevCount` is the value
                // before this hit, so the new count is prevCount + 1.
                if (static_cast<uint32_t>(prevCount + 1) <
                    static_cast<uint32_t>(self->stallHitsToTrigger_))
                        break;

                if (self->engineForIsr_) {
                        self->engineForIsr_->haltFromIsr(
                                StopReason::StallDetected);
                }
                // Promote the count into the latch atomically here.
                // The task-side service() path observes `stallLatched_`
                // (via `consumeStallActivation`) and routes the event
                // through the proper stall fault path; the engine is
                // already halted so this is bookkeeping only.
                self->stallLatched_.store(true, std::memory_order_release);
                self->stallHitCounter_.store(0, std::memory_order_release);
                break;
        }
        case SensorRole::CrashLimit:
        default:
                if (self->engineForIsr_) {
                        self->engineForIsr_->haltFromIsr(StopReason::LimitSwitch);
                }
                self->crashLatched_.store(true, std::memory_order_release);
                break;
        }
}

Status SensorBank::begin(const SensorInputConfig *sensors, uint8_t count,
                         IPulseEngine *engineForIsr)
{
        if (begun_)
                return Status::Err(ErrorCode::AlreadyInitialized);
        if (count > MAX_SENSOR_INPUTS)
                return Status::Err(ErrorCode::InvalidConfig);

        engineForIsr_ = engineForIsr;

        // First pass: copy configs, validate, configure pins as inputs.
        bool seenHome = false;
        bool seenStall = false;
        bool anyIsrRole = false;
        for (uint8_t i = 0; i < count; ++i) {
                const auto &cfg = sensors[i];
                if (cfg.pin == GPIO_NONE)
                        continue; // ignore unset slots

                if (cfg.role == SensorRole::Home) {
                        if (seenHome) {
                                // Two home sensors don't make sense — homing routines
                                // can't target one over the other unambiguously.
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        seenHome = true;
                }
                if (cfg.role == SensorRole::Stall) {
                        if (seenStall) {
                                // Multiple stall inputs share one counter today;
                                // configuring two would silently lose tuning data.
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                        seenStall = true;
                        // Pick up the stall-specific tuning knobs from the first
                        // (and only) Stall sensor's config.
                        stallHitsToTrigger_ = cfg.stallHitsToTrigger;
                        stallArmDelayMs_ = cfg.stallArmDelayMs;
                }
                if (isIsrRole(cfg.role)) {
                        anyIsrRole = true;
                        if (!engineForIsr_) {
                                // ISR-driven sensors need an engine to halt. Asking
                                // for a CrashLimit / Stall without one is a config
                                // error, not silent fallback to polling.
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                }

                Slot &s = slots_[slotCount_++];
                s.cfg = cfg;
                s.inUse = true;
                s.stableActive = false;
                s.candidateActive = false;
                s.candidateSinceMs = 0;

                // Pull direction matches polarity: NC switches pull HIGH when
                // open (so the line floats safely high when the wire is intact).
                const auto pull = (cfg.polarity == SensorPolarity::NormallyClosed) ?
                                      gpio::PullMode::UP :
                                      gpio::PullMode::DOWN;

                if (isIsrRole(cfg.role)) {
                        // Edge: for NC the "pressed/cut" transition is HIGH→LOW,
                        // i.e. falling edge. For NO it's rising. Either way we
                        // only care about activation, not release.
                        const auto edge = (cfg.polarity == SensorPolarity::NormallyClosed) ?
                                              gpio::InterruptEdge::EDGE_FALLING :
                                              gpio::InterruptEdge::EDGE_RISING;
                        if (!gpio::configInputInterrupt(cfg.pin, edge, pull)) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                } else {
                        // Polled sensors get the SAME polarity-derived pull
                        // the ISR branch uses. Without this, a configured-but-
                        // not-wired NC pin floats LOW on ESP32 → `readActive`
                        // returns true → `pumpSensors` halts every tick on
                        // TravelLimit, never reaching a real stall. The
                        // earlier polled path called `gpio::configInput`
                        // which disables both pulls.
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

        // Second pass: wire ISR handlers AFTER pin config so the ISR
        // service is installed exactly once if any ISR sensor is present.
        if (anyIsrRole) {
                if (!gpio::installIsrService()) {
                        end();
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                for (uint8_t i = 0; i < slotCount_; ++i) {
                        const auto &s = slots_[i];
                        if (!s.inUse)
                                continue;
                        if (!isIsrRole(s.cfg.role))
                                continue;

                        IsrBinding &b = isrBindings_[isrBindingCount_++];
                        b.bank = this;
                        b.pin = s.cfg.pin;
                        b.role = s.cfg.role;

                        if (!gpio::addIsrHandler(s.cfg.pin, &onIsrTrampoline, &b)) {
                                end();
                                return Status::Err(ErrorCode::InvalidConfig);
                        }
                }
        }

        begun_ = true;
        return Status::Ok();
}

void SensorBank::end()
{
        for (uint8_t i = 0; i < isrBindingCount_; ++i) {
                const auto &b = isrBindings_[i];
                if (b.pin != GPIO_NONE) {
                        (void)gpio::removeIsrHandler(b.pin);
                }
        }
        isrBindingCount_ = 0;

        for (auto &s : slots_) {
                s = Slot{};
        }
        slotCount_ = 0;
        engineForIsr_ = nullptr;
        crashLatched_.store(false, std::memory_order_release);
        estopLatched_.store(false, std::memory_order_release);
        stallLatched_.store(false, std::memory_order_release);
        stallHitCounter_.store(0, std::memory_order_release);
        stallHitsToTrigger_ = 4;
        stallArmDelayMs_ = 200;
        stallArmedAtMs_.store(0, std::memory_order_release);
        stallHitsTotal_.store(0, std::memory_order_release);
        begun_ = false;
}

void SensorBank::service(int64_t nowMs)
{
        if (!begun_)
                return;

        for (uint8_t i = 0; i < slotCount_; ++i) {
                Slot &s = slots_[i];
                if (!s.inUse)
                        continue;
                // ISR-driven roles are managed by the trampoline + atomic
                // latches; polling them would be redundant and might race
                // with a fresh ISR.
                if (isIsrRole(s.cfg.role))
                        continue;

                const bool active = readActive(s.cfg);

                if (active != s.candidateActive) {
                        s.candidateActive = active;
                        s.candidateSinceMs = nowMs;
                        continue;
                }
                // Same candidate as last tick — has it been stable long enough?
                if (active != s.stableActive) {
                        const int64_t elapsed = nowMs - s.candidateSinceMs;
                        if (elapsed >= static_cast<int64_t>(s.cfg.debounceMs)) {
                                s.stableActive = active;
                        }
                }
        }

        // Stall counter / arm-window evaluation. Two states:
        //   1) Bank not armed (stallArmedAtMs_ == 0): no motion has
        //      started since the last reset → discard hits.
        //   2) Inside the arm-delay window: discard hits (StealthChop
        //      auto-tune is still settling; readings are noise).
        //   3) Past the arm window: latch stall when count crosses
        //      threshold. The latch is what `consumeStallActivation()`
        //      returns.
        // Service-side noise filter. The ISR is now responsible for the
        // "past arm window AND hits >= threshold" promotion — it halts
        // the engine and sets `stallLatched_` atomically when the
        // counter crosses the threshold. Service() only cleans up
        // counts that the ISR couldn't legitimately use (unarmed, or
        // inside the arm window).
        const uint32_t hits = stallHitCounter_.load(std::memory_order_acquire);
        if (hits > 0) {
                const int64_t armedAt = stallArmedAtMs_.load(std::memory_order_acquire);
                const bool armed = (armedAt != 0);
                const bool pastWindow =
                        armed && (nowMs - armedAt) >=
                                         static_cast<int64_t>(stallArmDelayMs_);
                if (!armed || !pastWindow) {
                        // Discard noise outside the arm window. The ISR
                        // left the counter alone; service clears it.
                        stallHitCounter_.store(0, std::memory_order_release);
                }
                // Past the arm window: leave sub-threshold counts in
                // place so the next ISR edge can observe their
                // progression. The ISR resets the counter when it
                // promotes to a latched halt.
        }
}

bool SensorBank::isActive(SensorRole role) const
{
        if (role == SensorRole::CrashLimit) {
                return crashLatched_.load(std::memory_order_acquire);
        }
        if (role == SensorRole::EmergencyStop) {
                return estopLatched_.load(std::memory_order_acquire);
        }
        if (role == SensorRole::Stall) {
                return stallLatched_.load(std::memory_order_acquire);
        }
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (s.inUse && s.cfg.role == role && s.stableActive) {
                        return true;
                }
        }
        return false;
}

bool SensorBank::isActive(SensorRole role, Direction direction) const
{
        if (role == SensorRole::CrashLimit) {
                return crashLatched_.load(std::memory_order_acquire);
        }
        if (role == SensorRole::EmergencyStop) {
                return estopLatched_.load(std::memory_order_acquire);
        }
        if (role == SensorRole::Stall) {
                return stallLatched_.load(std::memory_order_acquire);
        }
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (s.inUse && s.cfg.role == role && s.cfg.direction == direction &&
                    s.stableActive) {
                        return true;
                }
        }
        return false;
}

bool SensorBank::consumeCrashActivation()
{
        return crashLatched_.exchange(false, std::memory_order_acq_rel);
}

bool SensorBank::consumeEstopActivation()
{
        return estopLatched_.exchange(false, std::memory_order_acq_rel);
}

bool SensorBank::consumeStallActivation()
{
        return stallLatched_.exchange(false, std::memory_order_acq_rel);
}

void SensorBank::notifyMotionStart(int64_t nowMs)
{
        // Each new motion gets a fresh stall window. Clearing the
        // counter is intentional: stale hits from a prior motion (e.g.
        // a deceleration glitch right before stop) must not survive
        // into the next jog and trip stallHitsToTrigger artificially
        // early.
        stallHitCounter_.store(0, std::memory_order_release);
        // Use release ordering so the ISR (which loads with acquire)
        // never observes the new armed-at timestamp before the counter
        // reset becomes visible. If nowMs happens to be 0 — only
        // possible on the host backend in the very first millisecond
        // of process life — bump to 1 so the ISR's "armedAt == 0
        // means not armed" sentinel still works.
        stallArmedAtMs_.store(nowMs == 0 ? 1 : nowMs, std::memory_order_release);
}

void SensorBank::notifyMotionEnd()
{
        // Disarm the stall window so a DIAG glitch on the trailing edge
        // of the just-finished motion (current-decay transient, mechanical
        // bounce) doesn't reach `haltFromIsr` and trip the engine fault.
        // The counter is left alone — if the service tick hasn't drained
        // it yet, it still needs to do so.
        stallArmedAtMs_.store(0, std::memory_order_release);
}

bool SensorBank::isAssertedLive(SensorRole role) const
{
        if (!begun_)
                return false;
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (!s.inUse || s.cfg.role != role)
                        continue;
                if (readActive(s.cfg))
                        return true;
        }
        return false;
}

uint32_t SensorBank::totalStallHits() const
{
        return stallHitsTotal_.load(std::memory_order_acquire);
}

bool SensorBank::isStallArmed() const
{
        return stallArmedAtMs_.load(std::memory_order_acquire) != 0;
}

void SensorBank::simulateIsrEdgeForTesting(SensorRole role)
{
        if (!begun_)
                return;
        for (uint8_t i = 0; i < isrBindingCount_; ++i) {
                if (isrBindings_[i].role == role) {
                        onIsrTrampoline(&isrBindings_[i]);
                        return;
                }
        }
}

uint8_t SensorBank::homePin() const
{
        for (uint8_t i = 0; i < slotCount_; ++i) {
                const Slot &s = slots_[i];
                if (s.inUse && s.cfg.role == SensorRole::Home) {
                        return s.cfg.pin;
                }
        }
        return GPIO_NONE;
}

} // namespace ungula::motor
