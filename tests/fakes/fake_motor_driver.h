// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_motor_driver.h"

namespace ungula::motor::tests
{

/// In-memory `IMotorDriver` that exercises the `MotorAxis` FSM without
/// any hardware. Motion completes when `simulateMotionComplete()` is
/// called by the test; faults latch via `simulateFault()`. Identity is
/// configurable so tests covering the identity surface have somewhere
/// to read from.
class FakeMotorDriver final : public IMotorDriver {
    public:
        FakeMotorDriver() = default;

        // ---- Lifecycle ---------------------------------------------------
        Status begin() override
        {
                beginCalls++;
                return Status::Ok();
        }
        Status enable() override
        {
                enableCalls++;
                return Status::Ok();
        }
        Status disable() override
        {
                disableCalls++;
                return Status::Ok();
        }
        Status clearFault() override
        {
                clearFaultCalls++;
                faulted = false;
                return Status::Ok();
        }

        // ---- Motion ------------------------------------------------------
        Status armMove(Direction dir, uint32_t targetSteps, uint32_t cruiseSps,
                       uint32_t accelSps2, uint32_t decelSps2) override
        {
                lastArmedDirection = dir;
                lastArmedTargetSteps = targetSteps;
                lastArmedCruiseSps = cruiseSps;
                lastArmedAccelSps2 = accelSps2;
                lastArmedDecelSps2 = decelSps2;
                armMoveCalls++;
                running = true;
                finishedReason = StopReason::None;
                return Status::Ok();
        }
        Status armJog(Direction dir, uint32_t cruiseSps, uint32_t accelSps2) override
        {
                lastArmedDirection = dir;
                lastArmedCruiseSps = cruiseSps;
                lastArmedAccelSps2 = accelSps2;
                lastArmedTargetSteps = 0;
                armJogCalls++;
                running = true;
                finishedReason = StopReason::None;
                return Status::Ok();
        }
        Status stop(StopMode mode) override
        {
                lastStopMode = mode;
                stopCalls++;
                running = false;
                finishedReason = (mode == StopMode::Immediate) ? StopReason::EmergencyStop :
                                                                  StopReason::UserStop;
                return Status::Ok();
        }

        // ---- Status ------------------------------------------------------
        DriverMotionStatus motionStatus() const override
        {
                DriverMotionStatus s;
                s.running = running;
                s.faulted = faulted;
                s.finishedReason = finishedReason;
                return s;
        }
        Position commandedPositionSteps() const override { return commandedPosition; }
        uint32_t commandedSpsNow() const override
        {
                return running ? lastArmedCruiseSps : 0u;
        }
        Status resetPosition(Position newSteps) override
        {
                commandedPosition = newSteps;
                resetPositionCalls++;
                return Status::Ok();
        }

        // ---- Identity / intent / diagnostics ----------------------------
        DriverIdentity identity() override { return id; }
        IntentSupport applyIntent(MotorIntent intent) override
        {
                lastIntent = intent;
                applyIntentCalls++;
                return IntentSupport::Supported;
        }
        void fillDriverDiagnostics(MotorDiagnostics &out) const override
        {
                out.identity = id;
                out.totalStepsIssued = totalStepsIssued;
        }

        // ---- Test knobs -------------------------------------------------
        void simulateMotionComplete(uint32_t emittedSteps, StopReason why = StopReason::TargetReached)
        {
                running = false;
                finishedReason = why;
                if (lastArmedDirection == Direction::Forward) {
                        commandedPosition += static_cast<int32_t>(emittedSteps);
                } else {
                        commandedPosition -= static_cast<int32_t>(emittedSteps);
                }
                totalStepsIssued += emittedSteps;
        }
        void simulateFault(StopReason why = StopReason::DriverFault)
        {
                running = false;
                faulted = true;
                finishedReason = why;
        }

        // ---- Observable state -------------------------------------------
        DriverIdentity id{ "TestVendor", "TestModel", 0, 0, 0 };
        bool   running = false;
        bool   faulted = false;
        StopReason finishedReason = StopReason::None;
        Position   commandedPosition = 0;
        uint32_t   totalStepsIssued = 0;

        Direction  lastArmedDirection = Direction::Forward;
        uint32_t   lastArmedTargetSteps = 0;
        uint32_t   lastArmedCruiseSps = 0;
        uint32_t   lastArmedAccelSps2 = 0;
        uint32_t   lastArmedDecelSps2 = 0;
        StopMode   lastStopMode = StopMode::Decelerate;
        MotorIntent lastIntent = MotorIntent::Default;

        // Call counters — useful for verifying call order in tests.
        uint32_t beginCalls = 0;
        uint32_t enableCalls = 0;
        uint32_t disableCalls = 0;
        uint32_t clearFaultCalls = 0;
        uint32_t armMoveCalls = 0;
        uint32_t armJogCalls = 0;
        uint32_t stopCalls = 0;
        uint32_t resetPositionCalls = 0;
        uint32_t applyIntentCalls = 0;
};

} // namespace ungula::motor::tests
