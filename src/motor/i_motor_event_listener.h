// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include "motor_event.h"

/// @brief Interface for consuming motor events.

namespace motor {

    class IMotorEventListener {
        public:
            virtual ~IMotorEventListener() = default;

            /// @brief Called when a motor emits an event.
            virtual void onMotorEvent(const MotorEvent& event) = 0;
    };

}  // namespace motor
