// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/hal/core/compiler_attrs.h"
#include "ungula/motor/axis_state.h"   // StopReason
#include "ungula/motor/axis_types.h"
#include "ungula/motor/planning/planned_move.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Who owns step pulse timing.
///   - Internal: the pulse engine owns a hardware timer + ISR and emits
///     pulses autonomously. This is the default and recommended mode.
///   - External: the host calls `tick(nowMicros)` from its own loop /
///     timer task. Useful for tests and for hosts that already have a
///     deterministic 1 ms tick they want to reuse. Less precise than
///     Internal because the worst-case latency is the host's tick period.
enum class PulseMode : uint8_t {
        Internal = 0,
        External = 1,
};

struct PulseEngineStatus {
        bool running = false;
        bool faulted = false;
        uint32_t emittedSteps = 0; // steps emitted in the current move
        uint32_t totalSteps = 0; // totalSteps from the loaded move
        uint8_t segmentIndex = 0;
        StopReason finishedReason = StopReason::None; // valid once running==false
};

/// Abstract step pulse generator. Implementations:
///   - `HalPulseEngine`     — production, drives an `IHwTimer` + STEP/DIR
///                            GPIO. Internal mode runs in the timer ISR.
///   - `FakePulseEngine`    — host tests; deterministic, no hardware.
///
/// Concurrency:
///   - `loadMove`, `start`, `stop`, `emergencyStop`, `begin`, `tick`
///     are called from task context.
///   - `status`, `isRunning`, `commandedPositionSteps` are safe from any
///     context (atomic reads).
///   - Internal-mode implementations run the alarm callback from ISR
///     context; the implementation is responsible for keeping it safe.
class IPulseEngine {
    public:
        virtual ~IPulseEngine() = default;

        /// Initialise the engine. The HAL pulse engine takes ownership of
        /// the step pin and a hardware timer here. Returns InvalidConfig if
        /// `stepPin`/`dirPin` are unset or `mode` is not supported by the
        /// concrete engine.
        virtual Status begin(PulseMode mode) = 0;

        /// Load a precomputed move. Allowed only when `isRunning()` is false
        /// (or, for soft re-plan, by stopping first). Validates the move is
        /// non-empty and that segment half-periods don't violate driver
        /// pulse-width minimums.
        virtual Status loadMove(const PlannedMove &move) = 0;

        /// Start emitting pulses for the currently loaded move. Returns
        /// `InvalidState` if no move is loaded or the engine is already
        /// running.
        virtual Status start() = 0;

        /// Stop pulses.
        ///
        ///   - `Immediate` — halt the hardware counter on the next alarm
        ///     boundary. Latches `StopReason::UserStop`.
        ///   - `Emergency` — same hardware path, latches
        ///     `StopReason::EmergencyStop`. `emergencyStop()` additionally
        ///     latches `faulted_` so subsequent `start()` returns
        ///     `DriverFault` until `clearFault()`.
        ///   - `Decelerate` — **NOT implemented at the engine layer**.
        ///     Returns `ErrorCode::Unsupported`. A real decel ramp requires
        ///     mid-flight replacement of the segment queue; the Axis
        ///     facade is responsible for synthesising it via
        ///     `MotionPlanner::planStop` and a future
        ///     `loadMoveInFlight` engine call. **Do NOT silently treat
        ///     Decelerate as Immediate** — motion control depends on
        ///     accurate stop semantics; lying here is a safety bug.
        virtual Status stop(StopMode mode) = 0;

        /// Halt immediately, latch a `PulseEngineFault` flag.
        virtual Status emergencyStop() = 0;

        /// **ISR-context immediate halt.** Called by GPIO ISRs wired to
        /// CrashLimit / EmergencyStop sensors when sub-millisecond stop
        /// latency matters. Implementations MUST:
        ///   - Disarm the timer (or the equivalent ISR-safe halt).
        ///   - Set the running flag false atomically.
        ///   - Latch `faulted_` and `finishedReason_ = reason`.
        ///   - NOT call task-context APIs (no UART, no logging, no heap).
        ///
        /// The Axis service path picks up the latched state on its next
        /// `service()` tick and emits the appropriate event. `reason` is
        /// typically `LimitSwitch` (crash) or `EmergencyStop` (e-stop).
        ///
        /// **Attribute placement note:** the declaration here is NOT
        /// tagged `UNGULA_ISR_ATTR` for the same reason as `IHwTimer`
        /// — `IRAM_ATTR` uses `__COUNTER__` in its section name and a
        /// second expansion at the definition site conflicts. Concrete
        /// implementations apply the attribute to the DEFINITION only.
        virtual void haltFromIsr(StopReason reason) = 0;

        /// True between `start()` and the natural end / any stop.
        virtual bool isRunning() const = 0;

        /// Snapshot of internal state. Safe from any context.
        virtual PulseEngineStatus status() const = 0;

        /// Commanded position in steps, ISR-precise.
        virtual int32_t commandedPositionSteps() const = 0;

        /// Reset the commanded position counter. Allowed only when stopped;
        /// returns `MotionInProgress` otherwise.
        virtual Status resetPosition(int32_t positionSteps) = 0;

        /// Clear a latched fault. Required after `emergencyStop()` or any
        /// `DriverFault` raised from the ISR (e.g. `rearmFromIsr` returning
        /// non-Ok). Returns `MotionInProgress` if motion is somehow still
        /// running. After a successful `clearFault()`, the engine accepts
        /// `loadMove` + `start` again.
        virtual Status clearFault() = 0;

        /// External-mode only. `due(now)` returns true if there is at least
        /// one pending edge to emit; `tick(now)` emits all pending edges up
        /// to and including `now`. Internal-mode engines may return
        /// `Unsupported` here.
        virtual bool due(int64_t /*nowMicros*/) const
        {
                return false;
        }
        virtual Status tick(int64_t /*nowMicros*/)
        {
                return Status::Err(ErrorCode::Unsupported);
        }
};

} // namespace ungula::motor
