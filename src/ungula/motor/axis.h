// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <memory>

#include "ungula/hal/timer/i_hwtimer.h"
#include "ungula/motor/result.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/motion_units.h"
#include "ungula/motor/planning/motion_planner.h"
#include "ungula/motor/limits/sensor_bank.h"
#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/homing/i_homing_axis.h"
#include "ungula/motor/events/axis_event.h"
#include "ungula/motor/events/axis_event_queue.h"

namespace ungula::motor
{

class IAxisActuator;
class IPulseEngine;
class IHomingStrategy;

/// User-facing axis facade.
///
/// Created via one of the three `create*()` factories. The factory
/// validates the config, heap-allocates the underlying components
/// (timer + pulse engine + actuator), wires them, and returns a fully
/// owning `unique_ptr<Axis>`. Heap allocation happens only in the
/// factory (boot path); steady-state runtime is allocation-free.
///
/// ## Ownership model
///
/// Three component slots, each held as `unique_ptr`. Declaration order
/// is `timer_` → `engine_` → `actuator_` so destruction runs in
/// reverse: actuator first (it borrows the engine), then engine (it
/// borrows the timer), then timer last.
///
/// For CAN servo axes, `timer_` and `engine_` are null; only
/// `actuator_` (a `CanServoActuator`) is allocated.
///
/// ## Service model
///
/// The host calls `service(nowMs)` from a regular task tick (1 ms is
/// typical). That single call:
///   - Polls debounced sensors and consumes ISR-latched ones.
///   - Halts motion if a TravelLimit in the move's direction activates
///     or if a Home sensor fires while a homing cycle is in progress.
///   - Detects engine motion-complete transitions and emits
///     `MotionCompleted` / `MotionStopped` / `EmergencyStopped`.
///   - Ticks the homing controller, which steps the strategy through
///     its FSM (Idle → FastApproach → Backoff → SlowApproach →
///     SetHomePosition → Complete / Failed).
///
/// Event listeners are dispatched out of the same call, AFTER the
/// state has been brought up to date, so a listener observing
/// `MotionCompleted` can safely call `moveBy()` for the next move.
///
/// ## Public command API
///
///   - begin() / enable() / disable()
///   - moveTo / moveBy / jog            — non-blocking; plans + arms + starts.
///   - stop(mode)                       — Immediate / Emergency.
///                                        Decelerate returns Unsupported.
///   - emergencyStop()                  — latches a fault.
///   - home()                           — runs the homing controller.
///   - clearFault()                     — leave Faulted.
///
/// ## Status queries
///
///   - state(), feedback(), faultStatus(), lastStopReason()
///   - isHoming(), isHomed(), homingPhase()
class Axis : public IHomingAxis {
    public:
        // --- Factories ---------------------------------------------------

        static Result<std::unique_ptr<Axis>>
        createStepDirStepper(const StepDirStepperAxisConfig &cfg);

        static Result<std::unique_ptr<Axis>> createStepDirServo(const StepDirServoAxisConfig &cfg);

        static Result<std::unique_ptr<Axis>> createCanServo(const CanServoAxisConfig &cfg);

        /// Advanced / test composition path. Hosts that want to swap one
        /// of the underlying components (e.g. a custom timer backend, a
        /// `FakePulseEngine` in a host integration test, an alternative
        /// actuator) build the three pieces by hand and hand them over.
        ///
        /// Requirements:
        ///   - `actuator` is mandatory.
        ///   - `engine` is required for STEP/DIR axes (the Axis service
        ///     path reads engine status to detect motion completion);
        ///     CAN-servo style axes leave it null.
        ///   - `timer` may be null when `engine` does not need one
        ///     (the fake engine doesn't; the production engine does).
        ///
        /// `sensors` / `sensorCount` are copied into the Axis the same
        /// way the production factories do.
        struct ComposedComponents {
                std::unique_ptr<ungula::hal::timer::IHwTimer> timer;
                std::unique_ptr<IPulseEngine> engine;
                std::unique_ptr<IAxisActuator> actuator;
                AxisCommonConfig common{};
                uint32_t timerResolutionHz = 1'000'000;
                uint32_t timerMinTicks = 5;
        };
        static Result<std::unique_ptr<Axis>>
        createComposed(ComposedComponents components, const SensorInputConfig *sensors = nullptr,
                       uint8_t sensorCount = 0);

        // --- Lifecycle ---------------------------------------------------

        Status begin();
        Status enable();
        Status disable();

        /// Live-level safety check across the configured ISR-role
        /// safety sensors (E-stop, crash limit). Returns true if any
        /// is currently asserted according to its polarity, bypassing
        /// the edge-driven ISR latch.
        ///
        /// Use this after `clearFault()` to decide whether re-enabling
        /// the axis is safe — the latch only fires on transitions, so
        /// a button held continuously across a `clearFault()` /
        /// `enable()` cycle is invisible to `lastStopReason()` /
        /// `consumeEstopActivation()` but visible here.
        ///
        /// Not wired into `enable()` automatically — the right gating
        /// policy depends on the host's wiring. A genuine fail-safe
        /// wire (line floats HIGH at rest, sinks to GND on press /
        /// cut) reads as "not asserted" here; an inverted wiring may
        /// read asserted at idle. Hosts call this where the wiring
        /// supports it.
        bool isSafetyInterlockActive() const;

        // --- Motion ------------------------------------------------------

        Status moveTo(Position target, DistanceUnit unit = DistanceUnit::Steps);
        Status moveBy(Distance delta, DistanceUnit unit = DistanceUnit::Steps);
        Status jog(Direction direction);

        /// Per-jog feed-rate override in user units. Resolves `s`
        /// against the axis's `UnitScaling`; returns `InvalidConfig`
        /// if the unit requires a scaling field that isn't configured.
        /// Speed is a magnitude — direction is supplied separately.
        Status jog(Direction direction, Speed s);

        Status stop(StopMode mode = StopMode::Immediate);
        Status emergencyStop();

        // --- Trajectory tuning (runtime) ---------------------------------
        //
        // Mutate the live `TrajectoryLimits` in user units. None of these
        // refuse during in-flight motion; the semantics are:
        //
        //   - `setMaxVelocity` while JOGGING: stops the engine and re-arms
        //     the jog at the new speed (brief glitch, no fault). Use this
        //     to change feed live during an indefinite jog.
        //   - `setMaxVelocity` while MOVING (moveTo/moveBy in flight): the
        //     in-flight profile is locked (segments already precomputed);
        //     the new value applies on the next motion.
        //   - `setAcceleration` / `setDeceleration` / `setRampProfile`:
        //     always next-motion only — changing ramp coefficients
        //     mid-flight would require re-planning the segment queue, and
        //     the engine doesn't support that today.

        Status setMaxVelocity(Speed s);
        Status setAcceleration(Acceleration a);
        Status setDeceleration(Acceleration a);
        /// Sets accel == decel in one call. Common for symmetric profiles.
        Status setRampProfile(Acceleration a);

        // --- Homing ------------------------------------------------------

        Status setHomingStrategy(IHomingStrategy *strategy, uint32_t timeoutMs = 0);
        Status home();
        bool isHoming() const;

        /// Idle-policy: what the axis does automatically when motion
        /// completes. `HoldCurrent` (default) leaves the chip to its
        /// own IHOLD-after-iHoldDelay path; `AutoDisable` calls
        /// `disable()` internally on every MotionCompleted /
        /// MotionStopped event. See `IdlePolicy` doc for trade-offs.
        ///
        /// This coexists with custom listeners — if you `subscribe()`
        /// your own `IAxisEventListener` it runs alongside the
        /// built-in policy. Use the policy for the common case, drop
        /// down to a listener if you want to do more (light the
        /// pillar, log the cycle, etc.).
        void setDefaultIdlePolicy(IdlePolicy policy);
        IdlePolicy defaultIdlePolicy() const;
        bool isHomed() const;
        HomingPhase homingPhase() const
        {
                return homing_.phase();
        }

        // --- Faults ------------------------------------------------------

        Status clearFault();
        FaultStatus faultStatus() const;

        // --- Driver identity ---------------------------------------------

        /// Universal "what's on the other end?" query. Delegates to the
        /// actuator, which in turn delegates to a chip-specific
        /// `IDriverIdentityProvider` if one was wired (kits wire it
        /// automatically). Returns `ErrorCode::Unsupported` only when no
        /// provider was wired (compose-by-hand omission). Returns
        /// `ErrorCode::TransportError` if the read failed (UART / CAN
        /// wiring or slave-address problem).
        ///
        /// May block on UART / CAN traffic. NOT for the motion-timing
        /// path — call from setup() or a diagnostics task.
        Result<DriverIdentity> readDriverIdentity();

        // --- Queries -----------------------------------------------------

        AxisState state() const;
        AxisFeedback feedback() const;
        StopReason lastStopReason() const;
        AxisId id() const;

        // --- Convenience state predicates -------------------------------
        //
        // Sugar over `state()` / `faultStatus()` for the common
        // call-site patterns. Use these instead of raw state-enum
        // comparisons so the semantics of "running", "idle" and
        // "faulted" stay defined in one place.

        /// True while the engine is actively emitting steps —
        /// `Moving`, `Jogging`, or `Homing`. False everywhere else,
        /// including `Faulted` and `EmergencyStopped`. Suitable for
        /// the classic blocking-wait loop
        /// `while (axis.isRunning()) { axis.service(now); yield(); }`.
        bool isRunning() const;

        /// True only when the axis is `Idle`. NOT a generalised "is
        /// the motor stopped" — `Disabled`, `Faulted`, and
        /// `EmergencyStopped` are all "not moving" but require very
        /// different host handling. If you need to gate the next
        /// motion command, combine with `hasFault()` and the
        /// `enable()` lifecycle yourself.
        bool isIdle() const;

        /// True if the axis is in any fault condition. ORs three
        /// signals so the host doesn't have to:
        ///   - `state() == AxisState::Faulted`
        ///   - `state() == AxisState::EmergencyStopped`
        ///   - `faultStatus().active()`
        /// The third clause matters because the pulse engine can
        /// latch a fault from an ISR (e.g. `haltFromIsr` on a
        /// `CrashLimit` edge) BEFORE the next `service()` tick has
        /// consumed that latch and pushed `state_` into `Faulted`.
        /// During that window `state()` still reports `Moving` /
        /// `Jogging`, but `hasFault()` returns true.
        bool hasFault() const;

        /// Cumulative count of DIAG / stall-sensor ISR hits since
        /// `begin()`. Includes hits discarded by the arm window or
        /// `stallHitsToTrigger` debounce; this is the raw edge count.
        /// 0 if no Stall sensor is configured.
        uint32_t totalStallHits() const;

        /// Test-only passthrough that fires one synthetic ISR edge on
        /// the configured sensor of `role`. Used by host tests to
        /// exercise E-stop precedence, stall debounce, and similar
        /// paths without a real GPIO. Production code MUST NOT call
        /// this.
        void simulateSensorIsrForTesting(SensorRole role)
        {
                sensors_.simulateIsrEdgeForTesting(role);
        }

        // --- Service path ------------------------------------------------

        /// Single periodic update. `nowMs` is monotonic milliseconds. Host
        /// calls from a regular tick (1 ms typical, 10 ms acceptable for
        /// low-speed work). NOT for ISR / timer-callback context.
        void service(int64_t nowMs);

        // --- Events ------------------------------------------------------

        Status subscribe(IAxisEventListener *listener);
        /// Drain pending events and dispatch to listeners. Returns the
        /// number drained. Called automatically at the tail of `service()`
        /// — hosts can also call it explicitly if they want to drain
        /// outside the regular tick (e.g. from a higher-priority task).
        uint32_t serviceEvents();

        // --- IHomingAxis (private contract, public methods) --------------

        Status commandMove(Distance deltaSteps, Velocity feedSps) override;
        Status commandJog(Direction direction, Velocity feedSps) override;
        Status stopMove() override;
        bool isHomeActive() const override;
        bool isStallActive() const override;
        bool isMotionIdle() const override;
        Status resetPosition(Position positionSteps) override;

        // --- Non-copyable, non-movable -----------------------------------

        Axis(const Axis &) = delete;
        Axis &operator=(const Axis &) = delete;
        Axis(Axis &&) = delete;
        Axis &operator=(Axis &&) = delete;
        ~Axis() override;

    private:
        Axis();

        Result<Distance> resolveToSteps(int32_t value, DistanceUnit unit) const;

        /// Common arming path. Updates `state_` to `onStart` on success
        /// and emits `MotionStarted`.
        Status armAndStart(const PlannedMove &move, AxisState onStart);

        /// Build a one-off `TrajectoryLimits` clone with a feed-rate cap
        /// for homing moves. Strategies pass their own feed in SPS;
        /// configurations' `maxVelocitySps` may be lower than that, in
        /// which case the configured cap wins (the planner enforces it).
        TrajectoryLimits limitsForFeed(Velocity feedSps) const;

        /// Emit an `AxisEvent`. Fills the common fields (timestamp,
        /// sequence, axisId, state, position snapshot) and enqueues.
        void emitEvent(AxisEventType type, StopReason stopReason = StopReason::None,
                       FaultCode faultCode = FaultCode::None);

        /// Internal motion-progress probe — used by `service()` to detect
        /// engine completion transitions.
        void pumpMotionState();

        /// Internal sensor pump — polls SensorBank, halts on
        /// limit/crash/estop, emits LimitActivated.
        void pumpSensors(int64_t nowMs);

        /// Component slots. Order matters; see "Ownership model" above.
        std::unique_ptr<ungula::hal::timer::IHwTimer> timer_; // may be null (CAN)
        std::unique_ptr<IPulseEngine> engine_; // may be null (CAN)
        std::unique_ptr<IAxisActuator> actuator_;

        MotionPlanner planner_{};
        SensorBank sensors_{};
        HomingController homing_{};

        AxisCommonConfig common_{};
        AxisId axisId_ = AxisId(0xFF);
        uint32_t timerResolutionHz_ = 1'000'000;
        uint32_t timerMinTicks_ = 5;

        AxisState state_ = AxisState::Uninitialized;
        StopReason lastStopReason_ = StopReason::None;
        bool isHomed_ = false;
        Direction lastMoveDirection_ = Direction::Forward;
        bool motionInFlight_ = false;
        uint32_t eventSequence_ = 0;
        int64_t lastServiceMs_ = 0;
        IdlePolicy idlePolicy_ = IdlePolicy::HoldCurrent;

        /// Set by `pumpSensors` when stall fires during a homing
        /// cycle. The HomingController/strategy reads this via
        /// `isStallActive()` to know the jog ended because of a
        /// stall. Cleared when the strategy issues the next motion
        /// (commandJog / commandMove).
        bool stallObservedDuringHoming_ = false;

        /// Sensor configurations captured at factory time and applied to
        /// `sensors_` in `begin()`. Storing them lets the caller hand us a
        /// stack-allocated config and walk away.
        SensorInputConfig sensorConfigs_[MAX_SENSOR_INPUTS]{};
        uint8_t sensorConfigCount_ = 0;

        AxisEventQueue<32> events_{};
};

} // namespace ungula::motor
