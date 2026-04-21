#include <basic_motor/stepper_config.h>
#include <gtest/gtest.h>

using namespace ungula::motor;

// --- MotorTimer constants ---

TEST(MotorTimer, TimerFrequency) {
    EXPECT_EQ(MotorTimer::TIMER_FREQ_HZ, 1000000u);
}

TEST(MotorTimer, MinAlarmTicks) {
    EXPECT_EQ(MotorTimer::MIN_ALARM_TICKS, 2u);
}

TEST(MotorTimer, TogglesPerStep) {
    EXPECT_FLOAT_EQ(MotorTimer::TOGGLES_PER_STEP, 2.0f);
}

TEST(MotorTimer, MinRunningSps) {
    EXPECT_FLOAT_EQ(MotorTimer::MIN_RUNNING_SPS, 1.0f);
}

// --- StepperConfig default ---

TEST(StepperConfig, DefaultValues) {
    StepperConfig config{};
    EXPECT_EQ(config.stepPin, 0);
    EXPECT_EQ(config.enablePin, 0);
    EXPECT_EQ(config.directionPin, 0);
    EXPECT_EQ(config.rampTimeMs, 0u);
}
