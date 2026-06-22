// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_units.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Snapshot of step-signal generator status. The generator is the only
/// piece that knows in real time whether pulses are still being issued;
/// the axis polls this each `service()` tick to decide motion-done /
/// fault transitions.
struct StepSignalStatus {
        bool running = false;
        bool faulted = false;
        StopReason finishedReason = StopReason::None;
};

/// Abstraction over STEP/DIR pulse generation. Concrete generators in
/// Stage 1:
///   - `GptimerStepSignal`: ESP32 GPTimer + GPIO toggle ISR. Comfortable
///     up to ~150 kSPS, edge of usable around ~200 kSPS.
///   - `RmtStepSignal` (planned, Stage 2+): ESP32 RMT, past 500 kSPS.
///
/// Drivers that issue STEP pulses (TMC2209, YPMC, generic STEP/DIR) hold
/// a REFERENCE to one of these. They do not own it. That lets two
/// drivers share one generator (Wendy-Tensy: twin YPMC on one STEP pin)
/// without complicating ownership.
///
/// The generator consumes pre-planned `PlannedMove` segments; planning
/// happens above this interface (in the driver, via an
/// `IMotionPlanner`). The generator is just the "execute these
/// segments" runtime.
class IStepSignalGenerator {
    public:
        virtual ~IStepSignalGenerator() = default;

        /// Allocate hardware (timer, GPIO ISR) and configure pulse-width
        /// and DIR setup parameters. Idempotent for the same pin
        /// configuration; reconfiguring pins requires `end()` first.
        virtual Status begin(uint8_t stepPin, uint8_t dirPin, bool dirActiveHigh,
                             uint32_t dirSetupUs, uint32_t minPulseHighUs,
                             uint32_t minPulseLowUs) = 0;

        /// Release hardware. After `end()` the generator can be
        /// `begin()`-ed again with new pin assignments.
        virtual void end() = 0;

        /// Arm a pre-planned move. The move's `direction` decides DIR
        /// pin level; segments are consumed in order. The generator
        /// keeps a private copy or reference of the move for the
        /// duration — caller MUST keep the `PlannedMove` valid until
        /// `status().running` reads false.
        virtual Status armMove(const PlannedMove &move) = 0;

        /// Halt motion. `Decelerate` honours any decel ramp embedded in
        /// the planned move's remaining segments by truncating; `Immediate`
        /// drops pulses on the next ISR fire (sub-µs latency).
        virtual Status stop(StopMode mode) = 0;

        /// Splice a precomputed decel ramp onto the in-flight move so the
        /// motor coasts to a controlled stop instead of cutting pulses
        /// dead. `decelMove` MUST start near the current step rate for a
        /// seamless (velocity-continuous) handoff. The move is consumed
        /// like any other plan; motion finishes when the ramp runs out.
        /// Generators that can't repoint a running transmission fall back
        /// to an immediate halt — callers get a hard stop, never a no-op.
        virtual Status armDecelStop(const PlannedMove & /*decelMove*/)
        {
                return stop(StopMode::Immediate);
        }

        /// Current motion status. Polled by the axis every service tick.
        virtual StepSignalStatus status() const = 0;

        /// Position the generator believes the motor is at. Open-loop —
        /// equals "commanded position" since the engine assumes every
        /// pulse landed.
        virtual Position commandedPosition() const = 0;

        /// Live SPS being issued (post-ramp). Zero when not running.
        virtual uint32_t commandedSpsNow() const = 0;

        /// Move the position counter without issuing pulses. Used by
        /// homing strategies to declare "this is now position 0".
        virtual Status resetPosition(Position newSteps) = 0;

        /// Clear an ISR-latched generator fault (e.g. a rearm failure
        /// from the timer ISR). The axis calls this from `clearFault()`
        /// after the driver clears any chip-side fault.
        virtual Status clearFault() = 0;

        /// Generator timer resolution in Hz — the host needs this when
        /// asking the planner to express segment half-periods in ticks
        /// that match this specific generator.
        virtual uint32_t timerResolutionHz() const = 0;

        /// Minimum timer ticks per half-period the generator can
        /// honour. Planner uses this as a floor to clamp peak velocity.
        virtual uint32_t minTimerTicks() const = 0;
};

} // namespace ungula::motor
