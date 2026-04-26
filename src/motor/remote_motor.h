// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "i_motor.h"
#include "i_motor_command_sink.h"
#include "motor_state.h"

/// @brief Proxy IMotor for a motor running on another node.
///
/// Sends typed commands through IMotorCommandSink (implemented by the
/// application's message router) and caches state received from the
/// remote node. The motor knows nothing about transport — it just
/// expresses intent through the command sink.
///
/// ## State updates
///
/// The application receives state events from the remote node through
/// its shared transport. When a motor-related event arrives, it calls
/// updateState() on the corresponding RemoteMotor to refresh the cache.
/// This keeps the data flow unidirectional and explicit:
///
///   Coordinator → RemoteMotor → IMotorCommandSink → [app] → transport
///   transport → [app] → RemoteMotor.updateState()

namespace motor {

    class RemoteMotor : public IMotor {
        public:
            /// @param commandSink  Application-provided command delivery.
            /// @param motorId  Identifier of the motor on the remote node.
            RemoteMotor(IMotorCommandSink& commandSink, uint8_t motorId);

            // ---- IMotor interface ----

            void enable() override;
            void disable() override;
            void moveForward() override;
            void moveBackward() override;
            void moveTo(float target, DistanceUnit unit = DistanceUnit::STEPS) override;
            void moveBy(float delta, DistanceUnit unit = DistanceUnit::STEPS) override;
            void executeProfile(const MotionProfileSpec& profile) override;
            void stop() override;
            void emergencyStop() override;
            MotorFsmState state() const override;
            int32_t positionSteps() const override;
            bool isMoving() const override;

            // RemoteMotor is a proxy — the black-box status flags would
            // need to ride in the cached state messages from the far end.
            // For now, report conservative defaults. Wire these up when the
            // protocol grows the needed fields.
            bool isIdle() const override {
                return state() == MotorFsmState::Idle;
            }
            bool isStalling() const override {
                return state() == MotorFsmState::Stall;
            }
            StopReason lastStopReason() const override {
                return StopReason::None;
            }
            bool wasLimitHit() const override {
                return false;
            }
            bool isLimitActive(Direction /*dir*/) const override {
                return false;
            }
            bool isLimitActive(Direction /*dir*/, int32_t /*index*/) const override {
                return false;
            }
            int32_t limitCount(Direction /*dir*/) const override {
                return 0;
            }
            bool isHoming() const override {
                return false;
            }
            bool isHomed() const override {
                return false;
            }

            // ---- State updates (called by application message router) ----

            /// @brief Update cached state from a remote event.
            /// Called by the application when it receives a motor state message
            /// from the remote node through the shared transport.
            void updateState(MotorFsmState newState, int32_t position);

        private:
            IMotorCommandSink& commandSink_;
            uint8_t motorId_;

            // Cached state from last received event
            MotorFsmState cachedState_ = MotorFsmState::Disabled;
            int32_t cachedPosition_ = 0;
    };

}  // namespace motor
