// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#ifndef __cplusplus
#error UngulaMotor requires a C++ compiler
#endif

// UngulaMotor — high-level motor / axis control. Include this header
// from host code for the common case (a `MotorAxis` driven by one of
// the shipped concrete drivers). Driver-specific headers live under
// `motor/drivers/<chip>/` and are included only by hosts that need
// direct access to chip-specific configuration types.

// Dependency: lib (string + time utilities). Must come first so Arduino
// CLI discovers it.
#include <ungula/core.h>

// Foundation types + result.
#include "ungula/motor/result.h"
#include "ungula/motor/motor_units.h"
#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_intent.h"
#include "ungula/motor/driver_identity.h"
#include "ungula/motor/motion_segment.h"
#include "ungula/motor/motion_profile.h"
#include "ungula/motor/motor_event.h"
#include "ungula/motor/motor_diagnostics.h"

// Internal contracts (exposed so hosts can compose drivers directly).
#include "ungula/motor/i_device_transport.h"
#include "ungula/motor/i_step_signal_generator.h"
#include "ungula/motor/i_motion_planner.h"
#include "ungula/motor/i_limit_system.h"
#include "ungula/motor/i_homing_strategy.h"
#include "ungula/motor/i_motor_driver.h"

// Config + the black-box class.
#include "ungula/motor/motor_axis_config.h"
#include "ungula/motor/motor_axis.h"

// Default implementations shipped with the library.
#include "ungula/motor/limits/limit_system.h"
#include "ungula/motor/planners/trapezoidal_planner.h"
#include "ungula/motor/step_signal/gptimer_step_signal.h"
#include "ungula/motor/homing/home_to_limit_strategy.h"
