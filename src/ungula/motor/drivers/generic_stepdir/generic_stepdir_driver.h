// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include <memory>

#include "ungula/motor/drivers/generic_stepdir/generic_stepdir_config.h"
#include "ungula/motor/i_motion_planner.h"
#include "ungula/motor/i_motor_driver.h"
#include "ungula/motor/i_step_signal_generator.h"
#include "ungula/motor/motion_segment.h"

namespace ungula::motor::stepdir
{

/// Minimal `IMotorDriver` for plain STEP/DIR drives that have no bus.
/// Used directly for DM542 / TB6600 / similar; also serves as the base
/// for chip-specific specialisations (`YpmcStepDirDriver`).
///
/// What it owns:
///   - the EN pin (configures + writes it on enable/disable).
///
/// What it does NOT own:
///   - STEP / DIR pins (held by the `IStepSignalGenerator`).
///   - No transport — no UART, no CAN, no SPI.
///
/// Identity is statically "Unknown / Generic STEP/DIR" — there's no
/// chip side to query. Subclasses override `identity()` to return a
/// known brand.
///
/// `MotorIntent::*` flags are mostly no-ops here. A plain STEP/DIR
/// drive can't change chopper mode at runtime; the host's intent does
/// not reach the silicon. `applyIntent()` returns
/// `IntentSupport::Unsupported` for any non-default intent — drivers
/// that DO support an intent (YPMC's brake-on-fault, for example)
/// override the method.
class GenericStepDirDriver : public IMotorDriver {
    public:
        /// Self-owns constructor. Allocates an `RmtStepSignal` and a
        /// `TrapezoidalPlanner` internally and drives them. STEP / DIR
        /// / EN pins come from `cfg`. `begin()` calls
        /// `RmtStepSignal::begin(stepPin, dirPin, ...)` using cfg's
        /// pin / timing fields.
        explicit GenericStepDirDriver(GenericStepDirConfig cfg);

        /// Pluggable constructor: the host passes a pre-initialised
        /// `IStepSignalGenerator` + `IMotionPlanner`. Use this in
        /// tests (with `FakeStepSignal`), when sharing a planner
        /// across axes, or when you need a non-default generator
        /// (e.g. `GptimerStepSignal` because all RMT channels are
        /// taken). The driver does NOT call `stepSignal.begin()` in
        /// this mode.
        GenericStepDirDriver(GenericStepDirConfig cfg, IStepSignalGenerator &stepSignal,
                             IMotionPlanner &planner);

        GenericStepDirDriver(const GenericStepDirDriver &) = delete;
        GenericStepDirDriver &operator=(const GenericStepDirDriver &) = delete;

        // ---- IMotorDriver -----------------------------------------------
        Status begin() override;
        Status enable() override;
        Status disable() override;
        Status clearFault() override;

        Status armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps, uint32_t accelSps2,
                       uint32_t decelSps2) override;
        Status armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2) override;
        Status stop(StopMode mode) override;
        Status decelStop(uint32_t decelSps2) override;

        DriverMotionStatus motionStatus() const override;
        Position commandedPositionSteps() const override;
        uint32_t commandedSpsNow() const override;
        uint32_t timerResolutionHz() const override
        {
                return stepSignal_.timerResolutionHz();
        }
        Status resetPosition(Position newSteps) override;

        DriverIdentity identity() override;
        IntentSupport applyIntent(MotorIntent intent) override;
        void fillDriverDiagnostics(MotorDiagnostics &out) const override;

        /// Accessor for the active step signal generator, owned or
        /// pluggable. Hosts using `LimitSystem` need this pointer
        /// to wire the ISR-context emergency-stop path.
        IStepSignalGenerator *stepSignalForLimits()
        {
                return &stepSignal_;
        }

    protected:
        /// Hook for subclasses (e.g. `YpmcStepDirDriver`) that need to
        /// write extra pins (secondary DIR, etc.) before the step
        /// signal generator's `armMove`. Default impl is a no-op.
        virtual void onBeforeArm(Direction /*dir*/)
        {
        }

        /// Hook for subclasses to react to motion-state transitions
        /// (e.g. brake engage/release). Default no-op. Called from the
        /// driver's arm / stop paths after the underlying generator
        /// has been commanded.
        virtual void onMotionStarted()
        {
        }
        virtual void onMotionStopped()
        {
        }

        // Subclasses can override identity() to report their brand.

        IStepSignalGenerator &stepSignal()
        {
                return stepSignal_;
        }
        IMotionPlanner &planner()
        {
                return planner_;
        }
        const GenericStepDirConfig &config() const
        {
                return cfg_;
        }

    private:
        Status setEnablePinLevel(bool enabled);

        // The two `owned*` smart pointers are non-null only in
        // self-owns mode; pluggable mode leaves them null and the
        // references below bind to host-owned objects.
        GenericStepDirConfig cfg_;
        std::unique_ptr<IStepSignalGenerator> ownedStepSignal_;
        std::unique_ptr<IMotionPlanner> ownedPlanner_;
        IStepSignalGenerator &stepSignal_;
        IMotionPlanner &planner_;
        mutable PlannedMove lastPlannedMove_{};
        // Direction of the move currently in flight — the decel ramp for a
        // soft stop must continue in the same direction.
        Direction lastDir_ = Direction::Forward;
        bool begun_ = false;
        bool motionInFlight_ = false;
};

} // namespace ungula::motor::stepdir
