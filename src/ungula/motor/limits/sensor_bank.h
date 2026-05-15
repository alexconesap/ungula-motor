// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <atomic>
#include <cstdint>

#include "ungula/motor/axis_config.h" // MAX_SENSOR_INPUTS
#include "ungula/motor/limits/sensor_input.h"
#include "ungula/motor/pulse/i_pulse_engine.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Runtime sensor service. Owns up to `MAX_SENSOR_INPUTS` sensor
/// configurations and applies the right policy per role:
///
///   - **Home / TravelLimit**: polled. `service(nowMs)` reads the pin,
///     applies a `debounceMs` filter, and exposes a stable
///     `isActive(role[, dir])` query. Travel-limit activation matching
///     the in-flight direction is what the Axis uses to halt motion.
///
///   - **CrashLimit / EmergencyStop**: ISR-driven. `begin()` registers
///     a GPIO interrupt; when it fires, the ISR calls
///     `engine.haltFromIsr(...)` directly (sub-µs halt latency) AND
///     latches an atomic flag. The Axis service path consumes the
///     latch on its next tick and dispatches the corresponding event
///     from task context.
///
///   - **Stall**: ISR-driven like CrashLimit, with two extra knobs to
///     filter false positives. ISR increments an atomic counter; the
///     service tick latches the stall only when the counter exceeds
///     `stallHitsToTrigger` AND we are past the `stallArmDelayMs`
///     window that begins at each motion start. Reports
///     `StopReason::StallDetected` to distinguish from a generic
///     crash limit.
///
/// ## Concurrency
///
///   - `begin` / `end` / `service` are TASK-context only.
///   - `isActive*` / `consume*Activation` are safe from any task
///     context (they read atomics).
///   - The ISR target (`onIsr`) is the only ISR-context entry — it
///     never touches the polled-sensor state.
class SensorBank {
    public:
        SensorBank() = default;

        SensorBank(const SensorBank &) = delete;
        SensorBank &operator=(const SensorBank &) = delete;

        /// Configure the bank. `sensors` is copied — the caller's storage
        /// does not need to outlive the bank. `engineForIsr` may be null
        /// if no crash/estop sensors are configured; otherwise the bank
        /// keeps the reference for ISR use.
        ///
        /// Pins set to `GPIO_NONE` are ignored. Duplicate roles where they
        /// don't make sense (two estops on different pins is fine, two
        /// homes on different pins is rejected) return `InvalidConfig`.
        Status begin(const SensorInputConfig *sensors, uint8_t count, IPulseEngine *engineForIsr);

        /// Disarm ISRs and release pins. Safe to call before `begin()`.
        void end();

        /// Polled service tick. Reads each polled sensor, applies the
        /// `debounceMs` filter, and updates the cached active state.
        /// `nowMs` is monotonic milliseconds (from `ungula::core::time`).
        /// Hosts call this from the Axis `service()` path; calling it
        /// from an ISR is undefined.
        void service(int64_t nowMs);

        /// True if at least one sensor of `role` is currently latched
        /// active (debounced for polled roles, ISR-latched for crash/
        /// estop).
        bool isActive(SensorRole role) const;

        /// Role + direction lookup. Used by Axis to decide whether a
        /// TravelLimit activation should halt the in-flight move (only
        /// when the limit's direction matches the move's direction).
        bool isActive(SensorRole role, Direction direction) const;

        /// Consume the crash-limit ISR latch. Returns true exactly once
        /// per ISR event; resets the latch atomically. Use this from the
        /// service tick to know when to emit a `LimitActivated` event for
        /// CrashLimit.
        bool consumeCrashActivation();

        /// Consume the E-stop ISR latch. Same semantics as
        /// `consumeCrashActivation()`.
        bool consumeEstopActivation();

        /// Consume the stall latch. Returns true exactly once after the
        /// ISR-hit counter exceeds `stallHitsToTrigger` outside the arm
        /// window. Same semantics as the other `consume*Activation`
        /// methods.
        bool consumeStallActivation();

        /// Notify the bank that motion has just started. Resets the
        /// stall hit counter and arms the `stallArmDelayMs` window so
        /// auto-tune transients in the first ms of motion do not get
        /// counted as stall hits. Idempotent; cheap; safe to call from
        /// task context only.
        void notifyMotionStart(int64_t nowMs);

        /// Pin number of the first home sensor configured, or `GPIO_NONE`
        /// if none. Convenience for homing strategies.
        uint8_t homePin() const;

    private:
        /// Per-sensor runtime state. Polled sensors track debounce
        /// history; ISR sensors carry a pointer to the bank's atomic
        /// latch which the static ISR callback writes.
        struct Slot {
                SensorInputConfig cfg{};
                bool inUse = false;
                // ---- Polled-only state ----
                bool stableActive = false;
                bool candidateActive = false;
                int64_t candidateSinceMs = 0;
        };

        /// Static ISR trampoline. `ctx` is the bank pointer; the slot
        /// index is encoded in the high byte of the context word — but
        /// since we register one handler per pin we just stash the slot
        /// pointer through. ESP32 / STM32 ISR conventions covered by
        /// `UNGULA_ISR_ATTR` in the definition.
        struct IsrBinding {
                SensorBank *bank = nullptr;
                uint8_t pin = GPIO_NONE;
                SensorRole role = SensorRole::Home;
        };
        static void onIsrTrampoline(void *ctx);

        /// Read the raw pin and apply polarity to derive "is the switch
        /// considered pressed/active right now".
        static bool readActive(const SensorInputConfig &cfg);

        Slot slots_[MAX_SENSOR_INPUTS]{};
        uint8_t slotCount_ = 0;

        IsrBinding isrBindings_[MAX_SENSOR_INPUTS]{};
        uint8_t isrBindingCount_ = 0;

        IPulseEngine *engineForIsr_ = nullptr;

        std::atomic<bool> crashLatched_{ false };
        std::atomic<bool> estopLatched_{ false };

        // Stall debounce state. Counter is incremented from ISR context;
        // service() reads + resets it from task context. Threshold and
        // arm window come from the first Stall sensor's config.
        std::atomic<uint32_t> stallHitCounter_{ 0 };
        std::atomic<bool> stallLatched_{ false };
        uint8_t stallHitsToTrigger_ = 4;
        uint16_t stallArmDelayMs_ = 200;
        int64_t stallArmedAtMs_ = 0; // start-of-motion timestamp; 0 = not armed

        bool begun_ = false;
};

} // namespace ungula::motor
