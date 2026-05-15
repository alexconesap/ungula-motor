// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor
{

/// What an actuator can do. The Axis facade inspects this to decide
/// whether a query (e.g. "what is actualPosition?") can be honoured or
/// has to return `Unsupported` / a sentinel.
struct ActuatorCapabilities {
        bool hasEnablePin = false;
        bool hasDirectionPin = true; // both step/dir actuators
        bool hasPulseEngine = true; // false for CAN drives
        bool hasActualPosition = false; // encoder or drive feedback
        bool hasInPositionInput = false;
        bool hasAlarmInput = false;
        bool hasFollowingError = false;
        bool hasNativeHoming = false; // drive does its own homing (CAN servo)
        bool hasStallDetection = false;
};

} // namespace ungula::motor
