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

# ---------------------------------------------------------------------------
# Dependencies: ungula-lib (headers for TimeControl, GpioAccess) and emblogx
# (tmc_stepper.cpp uses log_error via <emblogx/logger.h>).
#
# By default we use sibling directories (../../lib, ../../lib_emblogx) which
# is the layout during local development. External consumers who cloned only
# lib_motor can set -DUSE_LOCAL_DEPS=OFF to fetch them from GitHub instead.
# ---------------------------------------------------------------------------
option(USE_LOCAL_DEPS "Use local sibling lib/ and lib_emblogx/ directories" ON)

if(USE_LOCAL_DEPS)
    set(LIB_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib/src)
    set(LIB_EMBLOGX_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib_emblogx/src)
else()
    FetchContent_Declare(
        ungula_lib
        GIT_REPOSITORY https://github.com/alexconesap/ungula-core.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(ungula_lib)
    set(LIB_SRC ${ungula_lib_SOURCE_DIR}/src)

    FetchContent_Declare(
        emblogx
        GIT_REPOSITORY https://github.com/alexconesap/emblogx.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(emblogx)
    set(LIB_EMBLOGX_SRC ${emblogx_SOURCE_DIR}/src)
endif()

set(LIB_MOTOR_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/ArduinoStub
    ${LIB_SRC}
    ${LIB_EMBLOGX_SRC}
    ${LIB_MOTOR_SRC}
)

add_compile_definitions(
    EMBLOGX_BACKEND_STDIO
    EMBLOGX_LOG_PREFIX="[TEST] "
    EMBLOGX_ENABLE_SINK_CONSOLE=1
    EMBLOGX_ENABLE_SINK_MEMORY=0
    EMBLOGX_ENABLE_SINK_HTTP=0
    EMBLOGX_ENABLE_SINK_SD=0
)

add_executable(test_ungula_motor
    test_stepper_config.cpp
    test_stepper_controller.cpp
    test_motor_driver_interface.cpp
    test_current_curve.cpp
    ArduinoStub/system_stub.cpp
    ${LIB_MOTOR_SRC}/basic_motor/stepper_controller.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/logger_core.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/async_dispatcher.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/sinks/console_sink.cpp
)
target_link_libraries(test_ungula_motor gtest_main)

# ---------------------------------------------------------------------------
# Homing strategy tests — pure host tests, no hardware. Strategies depend
# only on IHomeableMotor, so MockHomeableMotor gives us full coverage of
# stall/limit-switch logic without esp_timer, gptimer or a real driver.
#
# The HomingRunner source pulls in <time/time_control.h> from ungula-lib.
# We already add LIB_SRC to the include paths above, so it resolves.
# ---------------------------------------------------------------------------
add_executable(test_ungula_motor_homing
    test_stall_homing_strategy.cpp
    test_limit_switch_homing_strategy.cpp
    test_homing_runner.cpp
    ArduinoStub/system_stub.cpp
    ${LIB_MOTOR_SRC}/motor/homing/stall_homing_strategy.cpp
    ${LIB_MOTOR_SRC}/motor/homing/limit_switch_homing_strategy.cpp
    ${LIB_MOTOR_SRC}/motor/homing/homing_runner.cpp
)
target_link_libraries(test_ungula_motor_homing gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_ungula_motor)
gtest_discover_tests(test_ungula_motor_homing)
