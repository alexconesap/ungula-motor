// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#ifndef __cplusplus
#error UngulaMotor requires a C++ compiler
#endif

// Ungula Motor Library — stepper motor control for embedded projects.
//
// Include this header to activate the library in Arduino CLI builds.
// It pulls in platform-independent types, interfaces, and utilities.
//
// Platform-dependent components must be included explicitly:
//   #include <ungula/motor/motor_fsm.h>                          // needs time/time.h
//   #include <ungula/motor/limit_switch.h>                       // needs hal/gpio/gpio_access.h
//   #include <ungula/motor/step_generator.h>                     // needs ESP-IDF gptimer
//   #include <ungula/motor/local_motor.h>                        // needs ESP-IDF esp_timer
//   #include <ungula/motor/drivers/tmc2209.h>                    // needs hal/uart, hal/gpio
//   #include <ungula/motor/homing/homing_runner.h>               // needs LocalMotor
//   #include <ungula/motor/homing/stall_homing_strategy.h>       // needs LocalMotor
//   #include <ungula/motor/homing/limit_switch_homing_strategy.h>

// Depend on UngulaCore — must be included first so Arduino CLI
// discovers lib/ include paths before our headers reference them.
#include <ungula/core.h>

// ---- Motor system (motor:: namespace) ----

// Types and enums
#include "ungula/motor/motion_profile.h"
#include "ungula/motor/motor_event.h"
#include "ungula/motor/motor_state.h"
#include "ungula/motor/motor_types.h"

// Interfaces
#include "ungula/motor/i_motor.h"
#include "ungula/motor/i_motor_command_sink.h"
#include "ungula/motor/i_motor_driver.h"
#include "ungula/motor/i_motor_event_listener.h"

// Utilities
#include "ungula/motor/motor_event_publisher.h"
#include "ungula/motor/stall_detector.h"

// Remote motor (no platform dependencies)
#include "ungula/motor/motor_coordinator.h"
#include "ungula/motor/remote_motor.h"
