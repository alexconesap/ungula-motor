#include <basic_motor/stepper_controller.h>
#include <gtest/gtest.h>

using namespace ungula::motor;

class StepperControllerTest : public ::testing::Test {
    protected:
        StepperController stepper;
        StepperConfig config;

        void SetUp() override {
            memset(&config, 0, sizeof(config));
            config.stepPin = 1;
            config.enablePin = 2;
            config.directionPin = 3;
            config.rampTimeMs = 100;
            stepper.configure(config);
        }
};

TEST_F(StepperControllerTest, InitialStateNotRunning) {
    EXPECT_FALSE(stepper.isRunning());
    EXPECT_FLOAT_EQ(stepper.getCurrentSPS(), 0.0f);
    EXPECT_EQ(stepper.getPositionSteps(), 0);
}

TEST_F(StepperControllerTest, SetSpeed) {
    stepper.setSpeed(2000);
    stepper.start();
    auto& state = stepper.getState();
    EXPECT_FLOAT_EQ(state.targetSPS, 2000.0f);
}

TEST_F(StepperControllerTest, Stop) {
    stepper.setSpeed(1000);
    stepper.start();
    stepper.stop();
    auto& state = stepper.getState();
    EXPECT_FLOAT_EQ(state.targetSPS, 0.0f);
}

TEST_F(StepperControllerTest, HardStop) {
    stepper.setSpeed(1000);
    stepper.start();
    stepper.hardStop();
    EXPECT_FALSE(stepper.isRunning());
    EXPECT_FLOAT_EQ(stepper.getCurrentSPS(), 0.0f);
}

TEST_F(StepperControllerTest, SetDirection) {
    stepper.setDirection(1);
    EXPECT_EQ(stepper.getDirection(), 1);

    stepper.setDirection(0);
    EXPECT_EQ(stepper.getDirection(), 0);
}

TEST_F(StepperControllerTest, RequestHardStopSetsFlag) {
    stepper.requestHardStop();
    auto& state = stepper.getState();
    EXPECT_TRUE(state.hardStopReq);
}

TEST_F(StepperControllerTest, ResetPosition) {
    stepper.resetPosition();
    EXPECT_EQ(stepper.getPositionSteps(), 0);
}

TEST_F(StepperControllerTest, ServiceRespectInterval) {
    // First call initializes lastServiceMs
    bool stopped = stepper.service(0);
    EXPECT_FALSE(stopped);

    // Call within SERVICE_INTERVAL_MS (10ms) should be skipped
    stopped = stepper.service(5);
    EXPECT_FALSE(stopped);
}

TEST_F(StepperControllerTest, ServiceHardStopRequest) {
    stepper.setSpeed(1000);
    stepper.start();
    stepper.requestHardStop();

    // Initialize with non-zero time
    stepper.service(100);
    bool stopped = stepper.service(111);
    EXPECT_TRUE(stopped);
    EXPECT_FALSE(stepper.isRunning());
}

TEST_F(StepperControllerTest, RampTimeZeroGivesInstantSpeed) {
    stepper.setRampTime(0);
    stepper.setSpeed(5000);
    stepper.start();

    // After enough service calls, speed should reach target
    for (uint32_t t = 0; t <= 100; t += 11) {
        stepper.service(t);
    }
    // With rampTime=0, acceleration is effectively infinite
    EXPECT_NEAR(stepper.getCurrentSPS(), 5000.0f, 100.0f);
}

TEST_F(StepperControllerTest, ISRPointersNotNull) {
    EXPECT_NE(stepper.getRunningPtr(), nullptr);
    EXPECT_NE(stepper.getAlarmTicksPtr(), nullptr);
}
