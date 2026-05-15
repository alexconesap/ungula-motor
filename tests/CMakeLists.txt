cmake_minimum_required(VERSION 3.16)
project(ungula_motor_lib_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# Sibling libraries during local development.
set(LIB_MOTOR_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)
set(LIB_HAL_SRC   ${CMAKE_CURRENT_LIST_DIR}/../../lib_hal/src)
set(LIB_CORE_SRC  ${CMAKE_CURRENT_LIST_DIR}/../../lib/src)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${LIB_MOTOR_SRC}
    ${LIB_HAL_SRC}
    ${LIB_CORE_SRC}
)

# ---------------------------------------------------------------------------
# MotionPlanner tests — pure logic; no HAL dependency at all.
# ---------------------------------------------------------------------------
add_executable(test_motion_planner
    test_motion_planner.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/planning/motion_planner.cpp
)
target_link_libraries(test_motion_planner gtest_main)

# ---------------------------------------------------------------------------
# HalPulseEngine tests — uses the HwTimerFake from lib_hal, the host GPIO
# stub from lib_hal (default backend), and the host time stub from lib.
# No real hardware involved.
# ---------------------------------------------------------------------------
add_executable(test_hal_pulse_engine
    test_hal_pulse_engine.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/pulse/hal_pulse_engine.cpp
    ${LIB_HAL_SRC}/ungula/hal/timer/platforms/hwtimer_default.cpp
)
target_link_libraries(test_hal_pulse_engine gtest_main)

# ---------------------------------------------------------------------------
# FakePulseEngine tests — header-only fake; no .cpp to compile in.
# ---------------------------------------------------------------------------
add_executable(test_fake_pulse_engine
    test_fake_pulse_engine.cpp
)
target_link_libraries(test_fake_pulse_engine gtest_main)

# ---------------------------------------------------------------------------
# StepDirActuator tests — use FakePulseEngine; no HAL hookup needed.
# ---------------------------------------------------------------------------
add_executable(test_step_dir_actuator
    test_step_dir_actuator.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/actuator/step_dir_actuator.cpp
)
target_link_libraries(test_step_dir_actuator gtest_main)

# ---------------------------------------------------------------------------
# Axis-facade-dependent tests share a common source list. The host
# timer backend has no autonomous ISR so motion never completes
# spontaneously — tests cover everything up to "armed".
# ---------------------------------------------------------------------------
set(AXIS_SRC_FILES
    ${LIB_MOTOR_SRC}/ungula/motor/axis.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/actuator/step_dir_actuator.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/planning/motion_planner.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/pulse/hal_pulse_engine.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/limits/sensor_bank.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/homing/homing_controller.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/homing/limit_switch_homing_strategy.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/homing/stall_homing_strategy.cpp
    ${LIB_HAL_SRC}/ungula/hal/timer/platforms/hwtimer_default.cpp
)

add_executable(test_axis_step_dir_stepper
    test_axis_step_dir_stepper.cpp
    ${AXIS_SRC_FILES}
)
target_link_libraries(test_axis_step_dir_stepper gtest_main)

# ---------------------------------------------------------------------------
# Speed / Acceleration unit conversions — pure-function tests, no
# Axis / engine dependencies. Header-only; nothing extra to compile.
# ---------------------------------------------------------------------------
add_executable(test_motion_units
    test_motion_units.cpp
)
target_link_libraries(test_motion_units gtest_main)

# ---------------------------------------------------------------------------
# Sensors, homing controller, axis event emission.
# ---------------------------------------------------------------------------
add_executable(test_sensor_bank
    test_sensor_bank.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/limits/sensor_bank.cpp
)
target_link_libraries(test_sensor_bank gtest_main)

add_executable(test_homing_controller
    test_homing_controller.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/homing/homing_controller.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/homing/limit_switch_homing_strategy.cpp
)
target_link_libraries(test_homing_controller gtest_main)

add_executable(test_axis_events
    test_axis_events.cpp
    ${AXIS_SRC_FILES}
)
target_link_libraries(test_axis_events gtest_main)

# ---------------------------------------------------------------------------
# End-to-end smoke test. Uses FakePulseEngine through the
# Axis::createComposed factory so the full pipeline can be exercised
# deterministically (no GPIO, no real timer ISR, no real hardware).
# ---------------------------------------------------------------------------
add_executable(test_axis_e2e_smoke
    test_axis_e2e_smoke.cpp
    ${AXIS_SRC_FILES}
)
target_link_libraries(test_axis_e2e_smoke gtest_main)

# ---------------------------------------------------------------------------
# TMC2209 driver components — all use the FakeTmcUart, no real UART
# required. Each is independent of the others.
# ---------------------------------------------------------------------------
add_executable(test_tmc2209_configurator
    test_tmc2209_configurator.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/tmc2209/tmc2209_configurator.cpp
)
target_link_libraries(test_tmc2209_configurator gtest_main)

add_executable(test_tmc2209_diagnostics
    test_tmc2209_diagnostics.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/tmc2209/tmc2209_diagnostics.cpp
)
target_link_libraries(test_tmc2209_diagnostics gtest_main)

add_executable(test_tmc2209_stallguard
    test_tmc2209_stallguard.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/tmc2209/tmc2209_stallguard.cpp
)
target_link_libraries(test_tmc2209_stallguard gtest_main)

add_executable(test_tmc2209_coolstep
    test_tmc2209_coolstep.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/tmc2209/tmc2209_coolstep.cpp
)
target_link_libraries(test_tmc2209_coolstep gtest_main)

# CRC self-consistency test only compiles the static CRC method, which
# lives in the .cpp — pull in the source so the static method links.
# No HAL UART dependency, no lib_hal source needed.
add_executable(test_tmc2209_hal_uart_crc
    test_tmc2209_hal_uart_crc.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/tmc2209/tmc2209_hal_uart.cpp
    ${LIB_HAL_SRC}/ungula/hal/uart/platforms/uart_default.cpp
)
target_link_libraries(test_tmc2209_hal_uart_crc gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_motion_planner)
gtest_discover_tests(test_hal_pulse_engine)
gtest_discover_tests(test_fake_pulse_engine)
gtest_discover_tests(test_step_dir_actuator)
gtest_discover_tests(test_axis_step_dir_stepper)
gtest_discover_tests(test_motion_units)
gtest_discover_tests(test_sensor_bank)
gtest_discover_tests(test_homing_controller)
gtest_discover_tests(test_axis_events)
gtest_discover_tests(test_axis_e2e_smoke)
gtest_discover_tests(test_tmc2209_configurator)
gtest_discover_tests(test_tmc2209_diagnostics)
gtest_discover_tests(test_tmc2209_stallguard)
gtest_discover_tests(test_tmc2209_coolstep)
gtest_discover_tests(test_tmc2209_hal_uart_crc)
