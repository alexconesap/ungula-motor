// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/motor/pulse/i_pulse_engine.h"

namespace ungula::motor
{

/// Header-only test fake. Models the externally-visible contract of
/// `IPulseEngine` without touching hardware: tracks running state,
/// position counter, segment progress, and the finished reason that
/// callers query via `status()`.
///
/// Tests advance simulated motion via:
///   - `runMove()`       — completes the loaded move "instantly".
///   - `tickSteps(n)`    — emits `n` steps; transitions to completed
///                          if those exhaust the move's totalSteps.
///   - `injectFault()`   — latches a `PulseEngineFault` state.
///
/// All counters mirror what the real engine would produce so the Axis
/// layer can be tested end-to-end without an `IHwTimer`.
class FakePulseEngine final : public IPulseEngine {
    public:
        Status begin(PulseMode mode) override
        {
                beginCallCount++;
                if (begun_)
                        return Status::Err(ErrorCode::AlreadyInitialized);
                mode_ = mode;
                begun_ = true;
                return Status::Ok();
        }

        Status loadMove(const PlannedMove &move) override
        {
                loadMoveCallCount++;
                if (!begun_)
                        return Status::Err(ErrorCode::NotInitialized);
                if (running_)
                        return Status::Err(ErrorCode::MotionInProgress);
                if (move.totalSteps == 0 || move.segmentCount == 0) {
                        return Status::Err(ErrorCode::InvalidConfig);
                }
                move_ = move;
                moveLoaded_ = true;
                emittedSteps_ = 0;
                currentSegmentIdx_ = 0;
                finishedReason_ = StopReason::None;
                return Status::Ok();
        }

        Status start() override
        {
                startCallCount++;
                if (!begun_)
                        return Status::Err(ErrorCode::NotInitialized);
                if (!moveLoaded_)
                        return Status::Err(ErrorCode::InvalidState);
                if (running_)
                        return Status::Err(ErrorCode::MotionInProgress);
                if (faulted_)
                        return Status::Err(ErrorCode::DriverFault);
                running_ = true;
                return Status::Ok();
        }

        Status stop(StopMode mode) override
        {
                stopCallCount++;
                if (!begun_)
                        return Status::Err(ErrorCode::NotInitialized);

                // Match the production engine: Decelerate is not implemented
                // at the engine layer. Refuse rather than silently downgrading.
                if (mode == StopMode::Decelerate) {
                        return Status::Err(ErrorCode::Unsupported);
                }

                if (!running_)
                        return Status::Ok();
                running_ = false;
                finishedReason_ = (mode == StopMode::Emergency) ? StopReason::EmergencyStop :
                                                                  StopReason::UserStop;
                return Status::Ok();
        }

        Status emergencyStop() override
        {
                emergencyStopCallCount++;
                const auto s = stop(StopMode::Emergency);
                if (!s.ok())
                        return s;
                faulted_ = true;
                return Status::Ok();
        }

        void haltFromIsr(StopReason reason) override
        {
                // Models the ISR-context immediate halt. The real engine
                // disarms the timer + latches faulted_; the fake just sets
                // the visible state. Counter increments so tests can verify
                // the path was exercised.
                haltFromIsrCallCount++;
                running_ = false;
                faulted_ = true;
                finishedReason_ = reason;
        }

        bool isRunning() const override
        {
                return running_;
        }

        PulseEngineStatus status() const override
        {
                PulseEngineStatus s;
                s.running = running_;
                s.faulted = faulted_;
                s.emittedSteps = emittedSteps_;
                s.totalSteps = move_.totalSteps;
                s.segmentIndex = currentSegmentIdx_;
                s.finishedReason = finishedReason_;
                return s;
        }

        int32_t commandedPositionSteps() const override
        {
                return commandedPosition_;
        }

        Status resetPosition(int32_t positionSteps) override
        {
                if (running_)
                        return Status::Err(ErrorCode::MotionInProgress);
                commandedPosition_ = positionSteps;
                return Status::Ok();
        }

        Status clearFault() override
        {
                clearFaultCallCount++;
                if (running_)
                        return Status::Err(ErrorCode::MotionInProgress);
                faulted_ = false;
                finishedReason_ = StopReason::None;
                return Status::Ok();
        }

        // ---- Test knobs --------------------------------------------------

        /// Advance simulated motion by `n` steps. Honours direction (the
        /// position counter increments for Forward, decrements for
        /// Backward). Transitions to "completed" if this drains the
        /// move. No-op if not running.
        void tickSteps(uint32_t n)
        {
                if (!running_ || n == 0)
                        return;

                const uint32_t remaining = move_.totalSteps - emittedSteps_;
                const uint32_t emit = (n < remaining) ? n : remaining;

                if (move_.direction == Direction::Forward) {
                        commandedPosition_ += static_cast<int32_t>(emit);
                } else {
                        commandedPosition_ -= static_cast<int32_t>(emit);
                }
                emittedSteps_ += emit;

                // Advance segment index in lockstep with emittedSteps.
                uint32_t cumul = 0;
                for (uint8_t i = 0; i < move_.segmentCount; ++i) {
                        cumul += move_.segments[i].stepCount;
                        if (emittedSteps_ < cumul) {
                                currentSegmentIdx_ = i;
                                break;
                        }
                        if (i + 1 == move_.segmentCount) {
                                currentSegmentIdx_ = i;
                        }
                }

                if (emittedSteps_ >= move_.totalSteps) {
                        running_ = false;
                        finishedReason_ = StopReason::TargetReached;
                }
        }

        /// Run the loaded move to completion in one call.
        void runMove()
        {
                tickSteps(move_.totalSteps);
        }

        /// Inject a hardware-fault scenario (e.g. simulated pulse engine
        /// fault). Latches the same state `emergencyStop()` would produce
        /// from inside the ISR after a backend error.
        void injectFault()
        {
                running_ = false;
                faulted_ = true;
                finishedReason_ = StopReason::DriverFault;
        }

        // ---- Counters ----------------------------------------------------

        uint32_t beginCallCount = 0;
        uint32_t loadMoveCallCount = 0;
        uint32_t startCallCount = 0;
        uint32_t stopCallCount = 0;
        uint32_t emergencyStopCallCount = 0;
        uint32_t clearFaultCallCount = 0;
        uint32_t haltFromIsrCallCount = 0;

    private:
        PlannedMove move_{};
        PulseMode mode_ = PulseMode::Internal;
        bool begun_ = false;
        bool moveLoaded_ = false;
        bool running_ = false;
        bool faulted_ = false;
        uint32_t emittedSteps_ = 0;
        uint8_t currentSegmentIdx_ = 0;
        int32_t commandedPosition_ = 0;
        StopReason finishedReason_ = StopReason::None;
};

} // namespace ungula::motor
