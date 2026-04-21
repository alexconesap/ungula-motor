// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#ifndef MOTOR_DRIVER_TMC2209
#error "tmc_stepper.h requires -DMOTOR_DRIVER_TMC2209"
#endif

#include <Arduino.h>
#include <TMCStepper.h>
#include <basic_motor/i_motor_driver.h>

namespace ungula {
    namespace motor {

        /// TMC2209 stepper driver wrapper, implements IMotorDriver
        class TmcStepper : public IMotorDriver {
            public:
                TmcStepper(int stepPin, int enablePin, int dirPin, HardwareSerial& serialPort,
                           float rSense, uint8_t driverAddress, int rxPin, int txPin);

                void begin() override;

                // Basic control (IMotorDriver interface)
                void setDirection(bool clockwise) override;
                void setEnable(bool enable) override;

                // Current control
                void setRunCurrent(uint16_t run_mA, float hold_frac = 0.30F);
                void setCurrentBySps(int sps) override;

                // Debug
                void debugPoll();
                void dumpBanner(const char* reason = nullptr);

            private:
                int stepPin_;
                int enablePin_;
                int dirPin_;

                HardwareSerial& serialPort_;
                float rSense_;
                uint8_t driverAddress_;
                int rxPin_;
                int txPin_;
                TMC2209Stepper driver_;

                // S-curve tunables
                static constexpr int SPS_LO = 300;
                static constexpr int SPS_HI = 2000;
                static constexpr int I_LO_MA = 900;
                static constexpr int I_HI_MA = 1300;
                static constexpr float HOLD_FRAC = 0.30F;
        };

    }  // namespace motor
}  // namespace ungula
