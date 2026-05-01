// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

// Recording IMotorCommandSink shared by test_remote_motor.cpp and
// test_motor_coordinator.cpp — captures every typed message a host
// transport would have to deliver, so tests can assert sequencing and
// payloads without involving real networking.

#include <motor/i_motor_command_sink.h>
#include <motor/motion_profile.h>

#include <vector>

namespace test_helpers {

    struct RecordingSink final : public motor::IMotorCommandSink {
            struct SimpleSent {
                    uint8_t id;
                    motor::MotorCommandType cmd;
            };
            struct MoveSent {
                    uint8_t id;
                    motor::MotorCommandType cmd;
                    motor::MotorMoveParams params;
            };
            struct ProfileSent {
                    uint8_t id;
                    motor::MotionProfileSpec profile;
            };

            std::vector<SimpleSent> simples;
            std::vector<MoveSent> moves;
            std::vector<ProfileSent> profiles;

            bool send(uint8_t motorId, motor::MotorCommandType command) override {
                simples.push_back({motorId, command});
                return true;
            }
            bool sendMove(uint8_t motorId, motor::MotorCommandType command,
                          const motor::MotorMoveParams& params) override {
                moves.push_back({motorId, command, params});
                return true;
            }
            bool sendProfile(uint8_t motorId, const motor::MotionProfileSpec& profile) override {
                profiles.push_back({motorId, profile});
                return true;
            }
    };

}  // namespace test_helpers
