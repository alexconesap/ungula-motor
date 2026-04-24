// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <motor/homing/homing_runner.h>
#include <motor/homing/i_homing_strategy.h>

#include "mock_homeable_motor.h"

namespace {

    using motor::HomingRunner;
    using motor::IHomeableMotor;
    using motor::IHomingStrategy;
    using motor::test::MockHomeableMotor;

    // Scripted strategy so we can drive the runner deterministically.
    class ScriptedStrategy : public IHomingStrategy {
        public:
            int beginCount = 0;
            int tickCount = 0;
            int finishCount = 0;
            bool lastFinishSucceeded = false;

            // The test can program how many ticks to run before reporting done,
            // and what the final outcome should be.
            int ticksUntilDone = 1;
            bool succeedOnDone = true;

            void begin(IHomeableMotor& /*motor*/) override {
                ++beginCount;
            }

            bool tick(IHomeableMotor& /*motor*/) override {
                ++tickCount;
                if (tickCount >= ticksUntilDone) {
                    succeeded_ = succeedOnDone;
                    return true;
                }
                return false;
            }

            void finish(IHomeableMotor& /*motor*/, bool succeeded) override {
                ++finishCount;
                lastFinishSucceeded = succeeded;
            }

            bool succeeded() const override {
                return succeeded_;
            }

            bool isAtHomeReference(const IHomeableMotor& /*motor*/) const override {
                return scriptedAtHome;
            }

            bool scriptedAtHome = false;

        private:
            bool succeeded_ = false;
    };

    TEST(HomingRunnerTest, StartThenStepReportsSuccess) {
        MockHomeableMotor mock;
        ScriptedStrategy strategy;
        strategy.ticksUntilDone = 3;
        HomingRunner runner(mock, strategy);

        runner.start();
        EXPECT_TRUE(runner.isRunning());
        EXPECT_EQ(strategy.beginCount, 1);

        EXPECT_FALSE(runner.step());
        EXPECT_FALSE(runner.step());
        EXPECT_TRUE(runner.step());

        EXPECT_FALSE(runner.isRunning());
        EXPECT_TRUE(runner.succeeded());
        EXPECT_EQ(strategy.finishCount, 1);
        EXPECT_TRUE(strategy.lastFinishSucceeded);
    }

    TEST(HomingRunnerTest, FailurePropagates) {
        MockHomeableMotor mock;
        ScriptedStrategy strategy;
        strategy.ticksUntilDone = 1;
        strategy.succeedOnDone = false;
        HomingRunner runner(mock, strategy);

        runner.start();
        EXPECT_TRUE(runner.step());
        EXPECT_FALSE(runner.succeeded());
        EXPECT_FALSE(strategy.lastFinishSucceeded);
    }

    TEST(HomingRunnerTest, AbortStopsAndCallsFinish) {
        MockHomeableMotor mock;
        ScriptedStrategy strategy;
        strategy.ticksUntilDone = 100;
        HomingRunner runner(mock, strategy);

        runner.start();
        runner.step();
        runner.abort();

        EXPECT_FALSE(runner.isRunning());
        EXPECT_FALSE(runner.succeeded());
        EXPECT_EQ(strategy.finishCount, 1);
        EXPECT_FALSE(strategy.lastFinishSucceeded);
    }

    TEST(HomingRunnerTest, StepAfterDoneIsNoOp) {
        MockHomeableMotor mock;
        ScriptedStrategy strategy;
        strategy.ticksUntilDone = 1;
        HomingRunner runner(mock, strategy);

        runner.start();
        EXPECT_TRUE(runner.step());
        const int finishCallsAfterFirst = strategy.finishCount;

        EXPECT_TRUE(runner.step());
        EXPECT_EQ(strategy.finishCount, finishCallsAfterFirst);
    }

}  // namespace
