// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/limits/sensor_bank.h"

#include "ungula/hal/gpio/gpio.h"

namespace ungula::motor
{

namespace gpio = ungula::hal::gpio;

namespace
{

        constexpr bool isIsrRole(SensorRole r)
        {
                return r == SensorRole::CrashLimit || r == SensorRole::EmergencyStop;
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

        // Halt the pulse engine FIRST. Latency on this path is the whole
        // point of wiring crash/estop to an interrupt — drop everything
        // else, get the steps to stop, then bookkeep.
        if (self->engineForIsr_) {
                const StopReason reason = (binding->role == SensorRole::EmergencyStop) ?
                                              StopReason::EmergencyStop :
                                              StopReason::LimitSwitch;
                self->engineForIsr_->haltFromIsr(reason);
        }

        if (binding->role == SensorRole::EmergencyStop) {
                self->estopLatched_.store(true, std::memory_order_release);
        } else {
                self->crashLatched_.store(true, std::memory_order_release);
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
                if (isIsrRole(cfg.role)) {
                        anyIsrRole = true;
                        if (!engineForIsr_) {
                                // ISR-driven sensors need an engine to halt. Asking
                                // for a CrashLimit without one is a config error,
                                // not silent fallback to polling.
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
                        if (!gpio::configInput(cfg.pin)) {
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
}

bool SensorBank::isActive(SensorRole role) const
{
        if (role == SensorRole::CrashLimit) {
                return crashLatched_.load(std::memory_order_acquire);
        }
        if (role == SensorRole::EmergencyStop) {
                return estopLatched_.load(std::memory_order_acquire);
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
