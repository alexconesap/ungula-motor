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
//   #include <motor/motor_fsm.h>                          // needs time/time_control.h
//   #include <motor/limit_switch.h>                       // needs hal/gpio/gpio_access.h
//   #include <motor/step_generator.h>                     // needs ESP-IDF gptimer
//   #include <motor/local_motor.h>                        // needs ESP-IDF esp_timer
//   #include <motor/drivers/tmc2209.h>                    // needs hal/uart, hal/gpio
//   #include <motor/homing/homing_runner.h>               // needs LocalMotor
//   #include <motor/homing/stall_homing_strategy.h>       // needs LocalMotor
//   #include <motor/homing/limit_switch_homing_strategy.h>

// Depend on UngulaCore — must be included first so Arduino CLI
// discovers lib/ include paths before our headers reference them.
#include <ungula_core.h>

// ---- Motor system (motor:: namespace) ----

// Types and enums
#include "motor/motion_profile.h"
#include "motor/motor_event.h"
#include "motor/motor_state.h"
#include "motor/motor_types.h"

// Interfaces
#include "motor/i_motor.h"
#include "motor/i_motor_command_sink.h"
#include "motor/i_motor_driver.h"
#include "motor/i_motor_event_listener.h"

// Utilities
#include "motor/motor_event_publisher.h"
#include "motor/stall_detector.h"

// Remote motor (no platform dependencies)
#include "motor/motor_coordinator.h"
#include "motor/remote_motor.h"

// ---- Legacy motor HAL (ungula::motor:: namespace) ----
// Retained for RBB1/RBB2 compatibility. Will be removed.
#include "basic_motor/i_motor_driver.h"
#include "basic_motor/stepper_config.h"
#include "basic_motor/stepper_controller.h"

#ifdef MOTOR_DRIVER_TMC2209
#include "basic_motor/tmc_stepper.h"
#endif
