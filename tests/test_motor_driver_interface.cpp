#include <basic_motor/stepper_controller.h>
#include <gtest/gtest.h>

#include "mock_motor_driver.h"

using namespace ungula::motor;

class DriverInterfaceTest : public ::testing::Test {
    protected:
        StepperController stepper;
        MockMotorDriver driver;
        StepperConfig config;

        void SetUp() override {
            memset(&config, 0, sizeof(config));
            config.stepPin = 1;
            config.enablePin = 2;
            config.directionPin = 3;
            config.rampTimeMs = 100;
            stepper.configure(config);
            stepper.setDriver(&driver);
        }
};

TEST_F(DriverInterfaceTest, StartCallsSetEnable) {
    stepper.setSpeed(1000);
    stepper.start();
    EXPECT_EQ(driver.setEnableCount, 1);
    EXPECT_TRUE(driver.lastEnable);
}

TEST_F(DriverInterfaceTest, HardStopDisablesDriver) {
    stepper.setSpeed(1000);
    stepper.start();
    driver.reset();

    stepper.hardStop();
    EXPECT_EQ(driver.setEnableCount, 1);
    EXPECT_FALSE(driver.lastEnable);
}

TEST_F(DriverInterfaceTest, SetDirectionForwardsToDriver) {
    stepper.setDirection(1);
    EXPECT_EQ(driver.setDirectionCount, 1);
    EXPECT_TRUE(driver.lastDirection);

    stepper.setDirection(0);
    EXPECT_EQ(driver.setDirectionCount, 2);
    EXPECT_FALSE(driver.lastDirection);
}

TEST_F(DriverInterfaceTest, SetCurrentBySpsCanBeCalled) {
    // Verify the interface method works through the mock
    driver.setCurrentBySps(1500);
    EXPECT_EQ(driver.setCurrentBySpsCount, 1);
    EXPECT_EQ(driver.lastSps, 1500);
}

TEST_F(DriverInterfaceTest, SetCurrentBySpsDefaultIsNoOp) {
    // A driver that does NOT override setCurrentBySps should not crash
    class MinimalDriver : public IMotorDriver {
        public:
            void begin() override {}
            void setDirection(bool) override {}
            void setEnable(bool) override {}
            // setCurrentBySps not overridden — uses default no-op
    };

    MinimalDriver minimal;
    minimal.setCurrentBySps(5000);  // should not crash
}

TEST_F(DriverInterfaceTest, RequestHardStopDisablesDriverOnService) {
    stepper.setSpeed(1000);
    stepper.start();
    stepper.requestHardStop();
    driver.reset();

    stepper.service(100);
    bool stopped = stepper.service(111);
    EXPECT_TRUE(stopped);
    EXPECT_EQ(driver.setEnableCount, 1);
    EXPECT_FALSE(driver.lastEnable);
}

TEST_F(DriverInterfaceTest, MultipleStartsCallEnableEachTime) {
    stepper.setSpeed(1000);
    stepper.start();
    stepper.start();
    stepper.start();
    EXPECT_EQ(driver.setEnableCount, 3);
}

TEST_F(DriverInterfaceTest, NullDriverDoesNotCrash) {
    StepperController bare;
    StepperConfig cfg{};
    cfg.stepPin = 1;
    cfg.enablePin = 2;
    cfg.directionPin = 3;
    bare.configure(cfg);
    // No driver set — should not crash
    bare.setSpeed(1000);
    bare.start();
    bare.setDirection(1);
    bare.hardStop();
    bare.service(0);
    bare.service(11);
}
