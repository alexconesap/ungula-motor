// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "homing_runner.h"

#include <time/time_control.h>

#include "i_homeable_motor.h"
#include "i_homing_strategy.h"

namespace motor {

    HomingRunner::HomingRunner(IHomeableMotor& motor, IHomingStrategy& strategy, uint32_t timeoutMs)
        : motor_(motor), strategy_(strategy), timeoutMs_(timeoutMs) {}

    void HomingRunner::start() {
        if (state_ == State::Running) {
            return;
        }
        startMs_ = ungula::TimeControl::millis();
        state_ = State::Running;
        strategy_.begin(motor_);
    }

    bool HomingRunner::step() {
        if (state_ != State::Running) {
            return true;
        }

        // Wall-clock guard. 0 disables the timeout so strategies with their own
        // internal deadline can opt out.
        if (timeoutMs_ != 0U && (ungula::TimeControl::millis() - startMs_) >= timeoutMs_) {
            strategy_.finish(motor_, false);
            state_ = State::DoneFail;
            return true;
        }

        if (strategy_.tick(motor_)) {
            const bool didSucceed = strategy_.succeeded();
            strategy_.finish(motor_, didSucceed);
            state_ = didSucceed ? State::DoneSuccess : State::DoneFail;
            return true;
        }

        return false;
    }

    void HomingRunner::abort() {
        if (state_ != State::Running) {
            return;
        }
        strategy_.finish(motor_, false);
        state_ = State::DoneFail;
    }

    uint32_t HomingRunner::elapsedMs() const {
        if (state_ == State::Idle) {
            return 0U;
        }
        return ungula::TimeControl::millis() - startMs_;
    }

}  // namespace motor
