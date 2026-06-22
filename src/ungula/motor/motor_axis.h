// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_homing_strategy.h"
#include "ungula/motor/i_limit_system.h"
#include "ungula/motor/i_motor_driver.h"
#include "ungula/motor/motor_axis_config.h"
#include "ungula/motor/motor_diagnostics.h"
#include "ungula/motor/motor_event.h"
#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_units.h"
#include "ungula/motor/result.h"

namespace ungula::motor
{

/// Black-box motor / axis controller. The host instantiates ONE
/// `MotorAxis` per physical motor, hands it a config + a driver
/// reference, and afterwards interacts only through the verbs below.
///
/// What the host does NOT see:
///   - chopper modes, register layouts, microstep table values
///   - SPS, half-period ticks, segment ramps
///   - GPIO ISR plumbing for limit switches
///   - identity provider indirections
///
/// What the host DOES see:
///   - "move", "stop", "home", "set speed"
///   - state, position (in any configured unit), diagnostics
///
/// Threading: every public method except the FSM-internal ISR-latch
/// drains is callable from any task context, but not from ISR.
/// `service()` must be called from exactly one task at any given time;
/// the lib does NOT spawn its own task.
class MotorAxis final {
    public:
        /// Construct an axis. `driver` MUST outlive the axis — the
        /// axis holds a reference, not ownership. Limit system and
        /// homing strategy are optional; pass nullptr for axes that
        /// don't need them (FS-UV's stall-only setup wants a limit
        /// system but no homing; an indefinite-jog test sketch can
        /// skip both).
        MotorAxis(MotorAxisConfig cfg, IMotorDriver &driver, ILimitSystem *limits = nullptr,
                  IHomingStrategy *homing = nullptr);

        ~MotorAxis();

        MotorAxis(const MotorAxis &) = delete;
        MotorAxis &operator=(const MotorAxis &) = delete;
        MotorAxis(MotorAxis &&) = delete;
        MotorAxis &operator=(MotorAxis &&) = delete;

        // ---- Lifecycle ---------------------------------------------------
        Status begin();
        Status enable();
        Status disable();
        Status clearFault();

        // ---- Motion ------------------------------------------------------
        Status moveForward();
        Status moveBackward();
        Status moveTo(DistanceValue target);
        Status moveBy(DistanceValue delta);
        Status home();
        Status stop();
        /// Decelerate the in-flight motion to a controlled stop at HALF the
        /// configured acceleration — a gentler rampdown than the move's own
        /// decel, meant for cancelling a jog without slamming the mechanics.
        /// Falls back to a hard stop when no ramp is configured or the
        /// generator can't splice a rampdown. Use `stop()` for a forced halt.
        Status softStop();
        Status emergencyStop();

        // ---- Runtime tuning ---------------------------------------------
        Status setSpeed(Speed s);
        Status setAcceleration(Acceleration a);
        Status setProfile(MotionProfile p);
        Status setIntent(MotorIntent intent);

        // ---- Service tick (host-driven, no internal task) ---------------
        void service(int64_t nowMs);

        // ---- Queries -----------------------------------------------------
        MotorState state() const
        {
                return state_;
        }
        StopReason lastStopReason() const
        {
                return lastStopReason_;
        }
        Position positionSteps() const;
        float position(DistanceUnit unit) const;
        uint32_t currentSps() const;
        /// Actual step rate the planner will emit at the configured
        /// cruise speed, given the driver's pulse-generator resolution.
        /// `currentSps()` reports live SPS (post-ramp, real motion);
        /// this getter reports the **steady-state** rate the next
        /// `armMove` / `armJog` will produce — `setSpeed()`'s argument
        /// quantized to the nearest rung the timer can hit. Hosts use
        /// this for honest "commanded speed" displays instead of
        /// echoing the un-quantized request. Returns the un-quantized
        /// resolved cruise when the driver does not expose a timer
        /// resolution (e.g. RMD CAN servos).
        uint32_t actualCruiseSps() const;
        /// True between `armMove` / `armJog` and the moment the
        /// driver reports motion finished (via the service tick).
        /// Lets homing / sequencing strategies detect "the last
        /// motion just ended" without needing the axis to flip
        /// out of `Homing` into `Idle` first.
        bool isMotionInFlight() const
        {
                return motionInFlight_;
        }
        MotorDiagnostics diagnostics() const;
        DriverIdentity identity();
        bool isHomed() const
        {
                return homed_;
        }
        const MotorAxisConfig &config() const
        {
                return cfg_;
        }

        // ---- Events -----------------------------------------------------
        /// Single-slot listener. Subscribing a second listener replaces
        /// the first.
        void subscribe(IMotorEventListener &listener);
        void unsubscribe();

    private:
        // Helpers used by the FSM + verbs (defined in .cpp).
        Status resolveSpeedToSps(Speed s, uint32_t &outSps) const;
        Status resolveDistanceToSteps(DistanceValue d, int32_t &outSteps) const;
        // `cruiseSps` is the resolved cruise speed used to ground unit
        // conversions that need a reference speed (currently
        // `AccelUnit::RampMs`). All rate-based units (SPS², RPM/s,
        // deg/s², mm/s²) ignore it.
        Status resolveAccelToSps2(Acceleration a, uint32_t cruiseSps, uint32_t &outSps2) const;

        // Resolved (and clamped) motion knobs the planner / driver use.
        // Filled at begin() from cfg_; mutated by setSpeed /
        // setAcceleration / setProfile.
        uint32_t resolvedCruiseSps_ = 0;
        uint32_t resolvedAccelSps2_ = 0;
        uint32_t resolvedDecelSps2_ = 0;
        uint32_t cachedHardCeiling_ = 0;
        MotionProfile activeProfile_;
        MotorIntent activeIntent_ = MotorIntent::Default;

        Status armMove(Direction dir, uint32_t targetSteps);
        Status armJog(Direction dir);

        void applyIntentToDriver();
        void emit(MotorEventType type, StopReason reason = StopReason::None,
                  FaultCode fault = FaultCode::None);
        void transition(MotorState newState);

        // Pulled from service() — separated for readability.
        void pumpLimits(int64_t nowMs);
        void pumpMotion();

        // Configuration & references (immutable across the axis lifetime
        // except for `cfg_.profile` / `cfg_.intent` which mirror the
        // runtime-mutable activeProfile_ / activeIntent_).
        MotorAxisConfig cfg_;
        IMotorDriver &driver_;
        ILimitSystem *limits_ = nullptr;
        IHomingStrategy *homing_ = nullptr;

        // FSM state.
        MotorState state_ = MotorState::Uninitialized;
        StopReason lastStopReason_ = StopReason::None;
        FaultCode lastFault_ = FaultCode::None;
        Direction activeDirection_ = Direction::Forward;
        bool motionInFlight_ = false;
        bool homed_ = false;

        IMotorEventListener *listener_ = nullptr;
};

} // namespace ungula::motor
