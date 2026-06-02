cmake_minimum_required(VERSION 3.16)
project(ungula_motor_lib_tests)

set(CMAKE_CXX_STANDARD 17)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

set(LIB_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib/src)
set(LIB_HAL_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib_hal/src)
set(LIB_CANBUS_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib_canbus/src)
set(LIB_CANBUS_TESTS ${CMAKE_CURRENT_LIST_DIR}/../../lib_canbus/tests)
set(LIB_MOTOR_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${LIB_SRC}
    ${LIB_HAL_SRC}
    ${LIB_CANBUS_SRC}
    ${LIB_CANBUS_TESTS}
    ${LIB_MOTOR_SRC}
)

# Host tests link lib_canbus too, so the optional RMD CAN driver in
# lib_motor must be enabled.
add_compile_definitions(UNGULA_USE_CANBUS)

# ---------------------------------------------------------------------------
# Stage 1 + Stage 2 host tests. Pure host code: MotorAxis FSM driven by
# fakes, the trapezoidal planner (pure math), unit conversions, and the
# TMC2209 driver against a fake byte-level transport + fake step signal
# generator. No GPIO / timer / UART hardware is touched here — the
# lib_hal::gpio calls inside the driver land on the no-op host stub.
# ---------------------------------------------------------------------------
add_executable(test_lib_motor
    test_motor_axis_fsm.cpp
    test_units_conversion.cpp
    test_trapezoidal_planner.cpp
    test_tmc2209_driver.cpp
    test_stepdir_drivers.cpp
    test_rmd_can_driver.cpp
    test_limit_system.cpp
    test_step_symbol_plan.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/motor_axis.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/limits/limit_system.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/motor_diagnostics.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/planners/trapezoidal_planner.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/tmc2209/tmc2209_driver.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/generic_stepdir/generic_stepdir_driver.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/ypmc/ypmc_driver.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/drivers/rmd/rmd_can_driver.cpp
    ${LIB_CANBUS_SRC}/ungula/canbus/bus_helpers.cpp
    ${LIB_CANBUS_SRC}/ungula/canbus/devices/rmd/motor.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/step_signal/rmt_step_signal.cpp
    ${LIB_MOTOR_SRC}/ungula/motor/homing/home_to_limit_strategy.cpp
)
target_link_libraries(test_lib_motor gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_lib_motor)
