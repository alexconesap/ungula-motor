#pragma once

#include <basic_motor/i_motor_driver.h>

namespace ungula {
    namespace motor {

        /// Mock motor driver for testing. Records all calls so tests can verify
        /// that StepperController interacts with the driver interface correctly.
        class MockMotorDriver : public IMotorDriver {
            public:
                // Call counters
                int beginCount = 0;
                int setDirectionCount = 0;
                int setEnableCount = 0;
                int setCurrentBySpsCount = 0;

                // Last values received
                bool lastDirection = false;
                bool lastEnable = false;
                int lastSps = 0;

                void begin() override {
                    beginCount++;
                }

                void setDirection(bool clockwise) override {
                    setDirectionCount++;
                    lastDirection = clockwise;
                }

                void setEnable(bool enable) override {
                    setEnableCount++;
                    lastEnable = enable;
                }

                void setCurrentBySps(int sps) override {
                    setCurrentBySpsCount++;
                    lastSps = sps;
                }

                void reset() {
                    beginCount = 0;
                    setDirectionCount = 0;
                    setEnableCount = 0;
                    setCurrentBySpsCount = 0;
                    lastDirection = false;
                    lastEnable = false;
                    lastSps = 0;
                }
        };

    }  // namespace motor
}  // namespace ungula
