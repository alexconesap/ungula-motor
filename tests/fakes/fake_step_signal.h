// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_step_signal_generator.h"

namespace ungula::motor::tests
{

/// Host-side step-signal generator that records what the driver hands
/// it without running any pulse train. Tests inspect the last armed
/// move (direction, total steps, segment count) and drive completion
/// via `setStatus()`.
class FakeStepSignal final : public IStepSignalGenerator {
    public:
        Status begin(uint8_t stepPin, uint8_t dirPin, bool dirActiveHigh,
                     uint32_t dirSetupUs, uint32_t minPulseHighUs,
                     uint32_t minPulseLowUs) override
        {
                stepPin_ = stepPin;
                dirPin_ = dirPin;
                dirActiveHigh_ = dirActiveHigh;
                dirSetupUs_ = dirSetupUs;
                minPulseHighUs_ = minPulseHighUs;
                minPulseLowUs_ = minPulseLowUs;
                begun_ = true;
                return Status::Ok();
        }
        void end() override { begun_ = false; }
        Status armMove(const PlannedMove &move) override
        {
                lastMove_ = move;
                armCalls_++;
                status_.running = true;
                status_.faulted = false;
                status_.finishedReason = StopReason::None;
                return Status::Ok();
        }
        Status stop(StopMode mode) override
        {
                lastStopMode_ = mode;
                status_.running = false;
                status_.finishedReason = (mode == StopMode::Immediate) ?
                                              StopReason::EmergencyStop :
                                              StopReason::UserStop;
                return Status::Ok();
        }
        StepSignalStatus status() const override { return status_; }
        Position commandedPosition() const override { return position_; }
        uint32_t commandedSpsNow() const override
        {
                return status_.running ? lastMove_.cruiseSps : 0u;
        }
        Status resetPosition(Position newSteps) override
        {
                position_ = newSteps;
                return Status::Ok();
        }
        Status clearFault() override
        {
                status_.faulted = false;
                status_.finishedReason = StopReason::None;
                return Status::Ok();
        }
        uint32_t timerResolutionHz() const override { return 1'000'000u; }
        uint32_t minTimerTicks() const override { return 5u; }

        // Test knobs.
        void setStatus(StepSignalStatus s) { status_ = s; }
        void completeMove()
        {
                status_.running = false;
                status_.finishedReason = StopReason::TargetReached;
                if (lastMove_.direction == Direction::Forward) {
                        position_ += static_cast<int32_t>(lastMove_.totalSteps);
                } else {
                        position_ -= static_cast<int32_t>(lastMove_.totalSteps);
                }
        }

        // Observable.
        PlannedMove lastMove_{};
        uint32_t    armCalls_ = 0;
        StopMode    lastStopMode_ = StopMode::Decelerate;
        StepSignalStatus status_{};
        Position    position_ = 0;
        bool        begun_ = false;
        uint8_t     stepPin_ = 0xFF;
        uint8_t     dirPin_ = 0xFF;
        bool        dirActiveHigh_ = true;
        uint32_t    dirSetupUs_ = 0;
        uint32_t    minPulseHighUs_ = 0;
        uint32_t    minPulseLowUs_ = 0;
};

} // namespace ungula::motor::tests
