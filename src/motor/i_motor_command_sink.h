// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "motion_profile.h"
#include "motor_types.h"

/// @brief Typed command interface for remote motor control.
///
/// RemoteMotor sends typed commands through this interface. The application
/// implements it by routing commands through whatever shared transport it
/// uses (ESP-NOW, MQTT, serial, etc.). The motor never touches the transport
/// directly and never serializes — that's the application's job.
///
/// ## Why not a raw byte transport?
///
/// The transport channel is shared across the whole system — motors, thermal
/// controllers, actions, telemetry all use the same bus. The motor shouldn't
/// own or filter a shared resource. Instead, it expresses intent ("move
/// forward") and the application decides how to deliver it.

namespace motor {

    /// @brief Motor command types for remote control.
    enum class MotorCommandType : uint8_t {
        ENABLE = 0,
        DISABLE,
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_TO,
        MOVE_BY,
        EXECUTE_PROFILE,
        STOP,  // this will use the acceleration/deceleration parameters from the current profile,
               // if any
        EMERGENCY_STOP  // hard stop with no ramping
    };

    /// @brief Payload for commands that carry a distance value.
    struct MotorMoveParams {
            float value = 0.0F;
            DistanceUnit unit = DistanceUnit::STEPS;
    };

    /// @brief Accepts typed motor commands for delivery to a remote node.
    ///
    /// Implemented by the application's message routing layer, not by the
    /// motor subsystem. The implementation serializes the command into the
    /// project's wire protocol and sends it over the shared transport.
    class IMotorCommandSink {
        public:
            virtual ~IMotorCommandSink() = default;

            /// @brief Send a simple command (enable, disable, stop, etc.).
            virtual bool send(uint8_t motorId, MotorCommandType command) = 0;

            /// @brief Send a move command with distance parameters.
            virtual bool sendMove(uint8_t motorId, MotorCommandType command,
                                  const MotorMoveParams& params) = 0;

            /// @brief Send a full motion profile for autonomous execution.
            virtual bool sendProfile(uint8_t motorId, const MotionProfileSpec& profile) = 0;
    };

}  // namespace motor
