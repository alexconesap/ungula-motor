// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace motor {

    class IHomeableMotor;
    class IHomingStrategy;

    /// @brief Non-blocking driver for an IHomingStrategy.
    ///
    /// Owns no motion logic of its own — just the lifecycle of a homing
    /// operation (start → tick → finish) plus an optional wall-clock timeout.
    ///
    /// Designed so the caller's main loop can home several motors in parallel
    /// (Rig-style X+Y homing) by ticking one runner per motor:
    ///
    ///   HomingRunner xRunner(xMotor, xStrategy, 10000);
    ///   HomingRunner yRunner(yMotor, yStrategy, 10000);
    ///   xRunner.start();
    ///   yRunner.start();
    ///   while (xRunner.isRunning() || yRunner.isRunning()) {
    ///     xRunner.step();
    ///     yRunner.step();
    ///     ungula::TimeControl::delayMs(5);
    ///   }
    ///
    /// A timeout of 0 disables the wall-clock limit — the strategy decides
    /// when to give up.
    class HomingRunner {
        public:
            HomingRunner(IHomeableMotor& motor, IHomingStrategy& strategy, uint32_t timeoutMs = 0U);

            /// @brief Kick off the homing sequence. Safe to call only from Idle.
            /// Calls strategy.begin() internally.
            void start();

            /// @brief Poll the strategy. Returns true when the runner has stopped
            /// (success, failure or abort). Cheap to call from a loop.
            bool step();

            /// @brief Abort an in-progress homing. Calls strategy.finish(motor, false).
            /// Safe to call at any time.
            void abort();

            bool isRunning() const {
                return state_ == State::Running;
            }

            bool succeeded() const {
                return state_ == State::DoneSuccess;
            }

            /// @brief Milliseconds elapsed since start() was called. Zero when idle.
            uint32_t elapsedMs() const;

        private:
            enum class State : uint8_t { Idle, Running, DoneSuccess, DoneFail };

            IHomeableMotor& motor_;
            IHomingStrategy& strategy_;
            uint32_t timeoutMs_;
            uint32_t startMs_ = 0U;
            State state_ = State::Idle;
    };

}  // namespace motor
