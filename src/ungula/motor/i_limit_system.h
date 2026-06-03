// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motor_units.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

class IStepSignalGenerator; // forward decl — limits halt the engine via ISR

/// What a limit input represents for the FSM. Mapped to behaviour at
/// activation:
///
///   - `TravelLimit`     polled — when active in the active motion
///                       direction, the axis halts under controlled
///                       deceleration and reports
///                       `StopReason::TravelLimit`. Used at both ends
///                       of a normally-bounded axis.
///   - `EmergencyLimit`  ISR-driven — immediate halt the moment the
///                       signal asserts. Reports `EmergencyStop`
///                       reason; the FSM transitions to
///                       `EmergencyStopped`. Use for hard mechanical
///                       end-stops behind the normal limit (Nicky-RBB1
///                       safety backstops).
///   - `HomeSensor`      polled — same plumbing as `TravelLimit` but
///                       only consulted by `IHomingStrategy`. Outside
///                       homing, activations are ignored by the axis.
///   - `StallSensor`     ISR-driven — driver-side stall input (TMC
///                       DIAG, BEMF detector, etc.) with hit-count and
///                       arm-window debounce.
enum class LimitKind : uint8_t {
        TravelLimit,
        EmergencyLimit,
        HomeSensor,
        StallSensor,
};

/// Switch wiring polarity. NO = switch closes a circuit when activated
/// (active = pin reads HIGH given a pulldown); NC = switch opens it
/// (active = pin reads LOW given a pullup). The limit system configures
/// pull mode automatically per polarity.
enum class SwitchPolarity : uint8_t {
        NormallyOpen,
        NormallyClosed,
};

/// Pin pull up / pull down mode. Determines how the GPIO pin is configured: 'Input', 'Input with PullUp', or 'Input with PullDown'.
/// - Set it as `HardwareResistors` if the pin is pulled up/down by hardware. It is equivalent to set the GPIO as `Input` but explicitly indicating that there are external pull resistors.
/// - Set it as `McU` to use the microcontroller's internal pull resistors. Note that certain GPIO pins may not support internal pull-ups or pull-downs like EPS32 34-39, so check your microcontroller's datasheet to ensure compatibility.
/// - Set it as `Polarity` to let the limit system decide based on the polarity (NO = pull down, NC = pull up).
/// - Set it as `Input` to leave the pin floating (not recommended if no hardware pull resistors are present).
enum class LimitPinPullMode : uint8_t {
        HardwareResistors,
        McU,
        Polarity,
        Input,
};

/// One row of the host's `MotorAxisConfig::limits_wiring[]` array.
/// Fixed-size struct, copied by value into the limit system at
/// `begin()`.
struct LimitWiring {
        // GPIO pin number. `GPIO_NONE` (0xFF) means this slot is unused and the limit system ignores it.
        uint8_t pin = GPIO_NONE;

        // What this limit is? A limit switch? An emergency stop? A homing sensor? A stall input?
        LimitKind kind = LimitKind::TravelLimit;

        // Which motion 'direction' this limit guards
        // For linear axes, Forward = away from the home/back limit, Backward = toward it.
        // For rotary axes, the mapping is up to the host and has no effect on the lib's internal logic.
        Direction direction = Direction::Forward;

        // Switch wiring `polarity` (NO vs NC).
        // The limit system configures pull mode automatically per polarity, unless explicitly overridden
        // by setting `pullMode`
        SwitchPolarity polarity = SwitchPolarity::NormallyOpen;

        // `pullMode`
        // Pin pull up / pull down mode. Determines how the GPIO pin is configured: 'Input', 'Input with PullUp', or 'Input with PullDown'.
        // Set it as `HardwareResistors` if the pin is pulled up/down by hardware. It is equivalent to set the GPIO as `Input` but explicitly indicating that there are external pull resistors.
        // Set it as `McU` to use the microcontroller's internal pull resistors. Note that certain GPIO pins may not support internal pull-ups or pull-downs like EPS32 34-39, so check your microcontroller's datasheet to ensure compatibility.
        // Set it as `Polarity` to let the limit system decide based on the polarity (NO = pull down, NC = pull up).
        // Set it as `Input` to leave the pin floating (not recommended if no hardware pull resistors are present).
        LimitPinPullMode pullMode = LimitPinPullMode::Polarity;

        // Debounce time in ms. For polled limits, this is the debounce time for the raw input reading.
        // For ISR limits, this is the duration of the "arm window"
        uint16_t debounceMs = 20;

        // Stall-only knobs (ignored for non-Stall kinds). The
        // sensitivity itself is a chip-side concern (TMC2209: SGTHRS,
        // configured via `Tmc2209Config::stallSensitivity`). The lib
        // does not duplicate it here - the limit-system slot only
        // owns the host-side debounce + arm-window timing.
        uint16_t stallArmDelayMs = 200;

        // `stallHitsToTrigger` counts rising edges on the stall pin.
        // Default 1 because the typical source (TMC2209 DIAG) is
        // LEVEL-HELD when stalled: the pin goes HIGH and stays HIGH,
        // so the lib sees a single LOW->HIGH edge per stall event.
        // Raise this only for pulsed stall sources (some BEMF
        // detectors, strain-gauge-with-debounce add-ons).
        uint8_t stallHitsToTrigger = 1;
};

/// Hard cap on how many limit inputs a single axis can carry. Sized for
/// the worst documented scenario in our projects at Ungula:
/// (Nicky-RBB1: 4 limit switches + optional home + optional stall = 6).
/// Bumped to 8 for headroom; no runtime cost when fewer are used.
constexpr uint8_t MAX_LIMIT_INPUTS = 8;

/// Default debounced limit + ISR-stall system. Polled limits read in
/// `service()` with per-row `debounceMs`; ISR limits halt the engine
/// via `IStepSignalGenerator::stop(Immediate)` from interrupt context
/// and latch an atomic flag that the axis consumes on the next service
/// tick.
class ILimitSystem {
    public:
        virtual ~ILimitSystem() = default;

        /// Configure pins + ISR handlers. `engineForIsr` may be null if
        /// no ISR limits (Emergency / Stall) are wired; otherwise the
        /// limit system holds a reference for ISR-side halts.
        virtual Status begin(const LimitWiring *wirings, uint8_t count,
                             IStepSignalGenerator *engineForIsr) = 0;
        virtual void end() = 0;

        /// Polled tick. Hosts call from the axis service path. NOT for
        /// ISR context.
        virtual void service(int64_t nowMs) = 0;

        /// Direction-agnostic active query (any sensor of this kind
        /// reads active).
        virtual bool isActive(LimitKind kind) const = 0;

        /// Direction-aware active query (sensor of this kind whose
        /// `direction` matches reads active). Used to pre-flight-gate
        /// motion in a given direction.
        virtual bool isActive(LimitKind kind, Direction dir) const = 0;

        // ISR-latch consumers — each returns true exactly once per
        // observed activation, then resets atomically.
        virtual bool consumeEmergencyActivation() = 0;
        virtual bool consumeStallActivation() = 0;

        // Motion bookends. The axis calls these so stall windows arm /
        // disarm correctly.
        virtual void notifyMotionStart(int64_t nowMs) = 0;
        virtual void notifyMotionEnd() = 0;

        // Diagnostics.
        virtual uint32_t totalStallHits() const = 0;
        virtual void resetStallHitsTotal() = 0;
};

} // namespace ungula::motor
