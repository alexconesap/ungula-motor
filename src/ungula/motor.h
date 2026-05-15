// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

// Public umbrella header. Application code that just wants the facade
// should include this and nothing else.

#include "ungula/motor/result.h"
#include "ungula/motor/axis_types.h"
#include "ungula/motor/axis_state.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/motion_units.h"
#include "ungula/motor/axis.h"
#include "ungula/motor/events/axis_event.h"

// Planner, pulse engine, and actuator implementations. Application
// code typically reaches only for `axis.h` and uses the factories;
// these headers are exposed for tests and for advanced hosts that
// compose components by hand.
#include "ungula/motor/planning/motion_planner.h"
#include "ungula/motor/pulse/hal_pulse_engine.h"
#include "ungula/motor/actuator/i_axis_actuator.h"
#include "ungula/motor/actuator/step_dir_actuator.h"
#include "ungula/motor/actuator/can_servo_actuator.h"
#include "ungula/motor/actuator/i_can_servo_protocol.h"

// Sensors, homing, and the service path. The Axis facade already
// pulls these in via its own header; hosts that compose components
// manually can reach for them here.
#include "ungula/motor/limits/sensor_bank.h"
#include "ungula/motor/homing/i_homing_strategy.h"
#include "ungula/motor/homing/i_homing_axis.h"
#include "ungula/motor/homing/homing_controller.h"
#include "ungula/motor/homing/limit_switch_homing_strategy.h"
#include "ungula/motor/homing/stall_homing_strategy.h"
