// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include <vector>

#include <motor/homing/i_homeable_motor.h>
#include <motor/motion_profile.h>
#include <motor/motor_state.h>
#include <motor/motor_types.h>

namespace motor {
    namespace test {

        /// @brief In-memory IHomeableMotor stand-in for strategy unit tests.
        ///
        /// The mock records every call the strategy makes, lets the test script
        /// the reported FSM state from tick to tick, and tracks position/running
        /// flags so finish() behaves like a real motor. No hardware, no timers.
        class MockHomeableMotor : public IHomeableMotor {
            public:
                // ---- scripted inputs -----------------------------------------------

                MotorFsmState scriptedState = MotorFsmState::Idle;
                int32_t scriptedPosition = 0;

                // ---- recorded outputs ----------------------------------------------

                struct ProfileWrite {
                        MotionProfile profile;
                        int32_t speedSps;
                        uint32_t accelMs;
                        uint32_t decelMs;
                };

                std::vector<ProfileWrite> profileWrites;
                std::vector<MotionProfile> activeProfiles;
                std::vector<float> moveByCalls;
                int moveForwardCount = 0;
                int moveBackwardCount = 0;
                int emergencyStopCount = 0;
                int stopCount = 0;
                int clearStallCount = 0;
                int clearFaultCount = 0;
                int resetPositionCount = 0;
                int enableCount = 0;
                int disableCount = 0;
                bool autoStopOnStall = false;

                // ---- IMotor --------------------------------------------------------

                void enable() override {
                    ++enableCount;
                }
                void disable() override {
                    ++disableCount;
                }

                void moveForward() override {
                    ++moveForwardCount;
                    running_ = true;
                }
                void moveBackward() override {
                    ++moveBackwardCount;
                    running_ = true;
                }
                void moveTo(float target, DistanceUnit /*unit*/ = DistanceUnit::STEPS) override {
                    moveByCalls.push_back(target);
                    running_ = true;
                }
                void moveBy(float delta, DistanceUnit /*unit*/ = DistanceUnit::STEPS) override {
                    moveByCalls.push_back(delta);
                    running_ = true;
                }
                void executeProfile(const MotionProfileSpec& /*profile*/) override {
                    running_ = true;
                }

                void stop() override {
                    ++stopCount;
                    running_ = false;
                }
                void emergencyStop() override {
                    ++emergencyStopCount;
                    running_ = false;
                    // A real motor drops back to Idle shortly after e-stop. Mirror that
                    // here so subsequent moves in the same tick are plausible.
                    scriptedState = MotorFsmState::Idle;
                }

                MotorFsmState state() const override {
                    return scriptedState;
                }
                int32_t positionSteps() const override {
                    return scriptedPosition;
                }
                bool isMoving() const override {
                    return running_;
                }

                // ---- IHomeableMotor extras ----------------------------------------

                void setAutoStopOnStall(bool enabled) override {
                    autoStopOnStall = enabled;
                }

                void setProfileSpeed(MotionProfile profile, int32_t speedSps) override {
                    profileWrites.push_back({profile, speedSps, lastAccel_, lastDecel_});
                    lastSpeed_ = speedSps;
                }
                void setProfileAccel(MotionProfile profile, uint32_t accelMs) override {
                    lastAccel_ = accelMs;
                    if (!profileWrites.empty() && profileWrites.back().profile == profile) {
                        profileWrites.back().accelMs = accelMs;
                    }
                }
                void setProfileDecel(MotionProfile profile, uint32_t decelMs) override {
                    lastDecel_ = decelMs;
                    if (!profileWrites.empty() && profileWrites.back().profile == profile) {
                        profileWrites.back().decelMs = decelMs;
                    }
                }
                void setActiveProfile(MotionProfile profile) override {
                    activeProfiles.push_back(profile);
                }

                void clearStall() override {
                    ++clearStallCount;
                    // Real FSM: Stall → Idle on ack.
                    if (scriptedState == MotorFsmState::Stall) {
                        scriptedState = MotorFsmState::Idle;
                    }
                }
                void clearFault() override {
                    ++clearFaultCount;
                    if (scriptedState == MotorFsmState::Fault) {
                        scriptedState = MotorFsmState::Idle;
                    }
                }

                void resetPosition() override {
                    ++resetPositionCount;
                    scriptedPosition = 0;
                }

            private:
                bool running_ = false;
                int32_t lastSpeed_ = 0;
                uint32_t lastAccel_ = 0U;
                uint32_t lastDecel_ = 0U;
        };

    }  // namespace test
}  // namespace motor
