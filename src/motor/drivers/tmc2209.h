// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <hal/gpio/gpio_access.h>
#include <hal/uart/uart.h>
#include <motor/i_motor_driver.h>
#include <motor/stall_detector.h>
#include <cstdint>

/// @brief TMC2209 stepper driver over UART.
///
/// Replaces the TMCStepper Arduino library — direct register access,
/// no external dependencies beyond our own UART HAL.
///
/// Owns all GPIO pins (STEP, EN, DIR, DIAG) and the complete stall
/// detection pipeline: DIAG pin polling, SG_RESULT register reads,
/// speed-based thresholds, blanking during acceleration, and scoring.

namespace tmc {

    // ---- TMC2209 identification ----
    constexpr uint8_t TMC2209_VERSION_ID = 0x21;

    // ---- UART protocol constants ----
    constexpr uint8_t SYNC_BYTE = 0x05;
    constexpr uint8_t WRITE_FLAG = 0x80;
    constexpr uint8_t REPLY_MASTER_ADDR = 0xFF;
    constexpr uint8_t CRC_POLYNOMIAL = 0x07;
    constexpr uint8_t BITS_PER_BYTE = 8;
    constexpr uint8_t WRITE_DATAGRAM_LEN = 8;
    constexpr uint8_t READ_REQUEST_LEN = 4;
    constexpr uint8_t READ_REPLY_LEN = 8;
    constexpr uint8_t READ_BUF_SIZE = 16;
    constexpr uint32_t ECHO_TIMEOUT_MS = 10;
    constexpr uint32_t REPLY_TIMEOUT_MS = 20;
    constexpr uint32_t INTER_DATAGRAM_MS = 2;

    // ---- GSTAT clear mask ----
    constexpr uint8_t GSTAT_CLEAR_ALL = 0x07;

    // ---- Current scale limits (5-bit field in IHOLD_IRUN) ----
    constexpr uint8_t CS_MAX = 31;
    constexpr uint8_t CS_MIN = 0;
    constexpr uint8_t CS_VSENSE_THRESHOLD = 16;

    // ---- Current calculation (from TMC2209 datasheet) ----
    constexpr float CS_STEPS = 32.0F;
    constexpr float VFS_STANDARD = 0.325F;
    constexpr float VFS_HIGH_SENS = 0.180F;
    constexpr float RSENSE_INTERNAL = 0.02F;
    constexpr float SQRT2 = 1.41421356F;
    constexpr float MA_PER_AMP = 1000.0F;

    // ---- Register addresses ----
    namespace reg {
        constexpr uint8_t GCONF = 0x00;
        constexpr uint8_t GSTAT = 0x01;
        constexpr uint8_t IFCNT = 0x02;
        constexpr uint8_t IOIN = 0x06;
        constexpr uint8_t IHOLD_IRUN = 0x10;
        constexpr uint8_t TPOWERDOWN = 0x11;
        constexpr uint8_t TPWMTHRS = 0x13;
        constexpr uint8_t TCOOLTHRS = 0x14;
        constexpr uint8_t SGTHRS = 0x40;
        constexpr uint8_t SG_RESULT = 0x41;
        constexpr uint8_t COOLCONF = 0x42;
        constexpr uint8_t MSCNT = 0x6A;
        constexpr uint8_t CHOPCONF = 0x6C;
        constexpr uint8_t DRV_STATUS = 0x6F;
        constexpr uint8_t PWMCONF = 0x70;
    }  // namespace reg

    // ---- GCONF bits ----
    namespace gconf {
        constexpr uint32_t I_SCALE_ANALOG = 1U << 0;
        constexpr uint32_t INTERNAL_RSENSE = 1U << 1;
        constexpr uint32_t EN_SPREADCYCLE = 1U << 2;
        constexpr uint32_t SHAFT = 1U << 3;
        constexpr uint32_t INDEX_OTPW = 1U << 4;
        constexpr uint32_t INDEX_STEP = 1U << 5;
        constexpr uint32_t PDN_DISABLE = 1U << 6;
        constexpr uint32_t MSTEP_REG_SEL = 1U << 7;
        constexpr uint32_t MULTISTEP_FILT = 1U << 8;
    }  // namespace gconf

    // ---- CHOPCONF bits/masks ----
    namespace chop {
        constexpr uint32_t TOFF_MASK = 0x0FU;
        constexpr int32_t HSTRT_SHIFT = 4;
        constexpr uint32_t HSTRT_MASK = 0x07U << 4;
        constexpr int32_t HEND_SHIFT = 7;
        constexpr uint32_t HEND_MASK = 0x0FU << 7;
        constexpr int32_t TBL_SHIFT = 15;
        constexpr uint32_t TBL_MASK = 0x03U << 15;
        constexpr uint32_t VSENSE = 1U << 17;
        constexpr int32_t MRES_SHIFT = 24;
        constexpr uint32_t MRES_MASK = 0x0FU << 24;
        constexpr uint32_t INTPOL = 1U << 28;
        constexpr uint32_t DEDGE = 1U << 29;
        constexpr uint32_t DISS2G = 1U << 30;
        constexpr uint32_t DISS2VS = 1U << 31;
    }  // namespace chop

    // ---- PWMCONF bits/masks ----
    namespace pwm {
        constexpr uint32_t OFS_MASK = 0xFFU;
        constexpr int32_t GRAD_SHIFT = 8;
        constexpr uint32_t GRAD_MASK = 0xFFU << 8;
        constexpr int32_t FREQ_SHIFT = 16;
        constexpr uint32_t FREQ_MASK = 0x03U << 16;
        constexpr uint32_t AUTOSCALE = 1U << 18;
        constexpr uint32_t AUTOGRAD = 1U << 19;
        constexpr int32_t FREEWHEEL_SHIFT = 20;
        constexpr uint32_t FREEWHEEL_MASK = 0x03U << 20;
        constexpr int32_t REG_SHIFT = 24;
        constexpr uint32_t REG_MASK = 0x0FU << 24;
        constexpr int32_t LIM_SHIFT = 28;
        constexpr uint32_t LIM_MASK = 0x0FU << 28;
    }  // namespace pwm

    // ---- IHOLD_IRUN bits/masks ----
    namespace ihr {
        constexpr uint32_t IHOLD_MASK = 0x1FU;
        constexpr int32_t IRUN_SHIFT = 8;
        constexpr uint32_t IRUN_MASK = 0x1FU << 8;
        constexpr int32_t IHOLDDELAY_SHIFT = 16;
        constexpr uint32_t IHOLDDELAY_MASK = 0x0FU << 16;
    }  // namespace ihr

    // ---- IOIN bits (read-only) ----
    namespace ioin {
        constexpr int32_t VERSION_SHIFT = 24;
        constexpr uint32_t VERSION_MASK = 0xFFU << 24;
    }  // namespace ioin

    // ---- DRV_STATUS bits (read-only) ----
    namespace drv {
        constexpr uint32_t OVERTEMP_WARN = 1U << 0;
        constexpr uint32_t OVERTEMP = 1U << 1;
        constexpr uint32_t SHORT_GND_A = 1U << 2;
        constexpr uint32_t SHORT_GND_B = 1U << 3;
        constexpr uint32_t SHORT_VS_A = 1U << 4;
        constexpr uint32_t SHORT_VS_B = 1U << 5;
        constexpr uint32_t OPEN_LOAD_A = 1U << 6;
        constexpr uint32_t OPEN_LOAD_B = 1U << 7;
        constexpr uint32_t TEMP_120C = 1U << 8;
        constexpr uint32_t TEMP_143C = 1U << 9;
        constexpr uint32_t TEMP_150C = 1U << 10;
        constexpr uint32_t TEMP_157C = 1U << 11;
        constexpr int32_t CS_ACTUAL_SHIFT = 16;
        constexpr uint32_t CS_ACTUAL_MASK = 0x1FU << 16;
        constexpr uint32_t STEALTH_MODE = 1U << 30;
        constexpr uint32_t STANDSTILL = 1U << 31;
    }  // namespace drv

    // ---- Sensible defaults applied by begin() ----
    namespace defaults {
        constexpr uint8_t TOFF = 5;
        constexpr uint16_t MICROSTEPS = 16;
        constexpr uint16_t RUN_CURRENT_MA = 1000;
        constexpr float HOLD_FRACTION = 0.5F;
        constexpr bool PWM_AUTOSCALE = true;
        constexpr bool SPREAD_CYCLE = false;
        constexpr bool PDN_DISABLE = true;
        constexpr bool ISCALE_ANALOG = false;
        constexpr bool INTERNAL_RSENSE = false;
        constexpr uint8_t BLANK_TIME = 1;  // TBL=1 → 24 clocks (matches TMCStepper blank_time(24))
        constexpr bool INTERPOL = true;
        constexpr uint8_t IHOLDDELAY = 10;
        constexpr uint8_t TPOWERDOWN = 20;
    }  // namespace defaults

    // ---- TCOOLTHRS calculation constants ----
    constexpr float TMC_FCLK = 12000000.0F;
    constexpr float TMC_INTERNAL_USTEPS = 256.0F;
    constexpr float TCOOLTHRS_MARGIN = 1.5F;
    constexpr uint32_t STALL_SETTLE_MARGIN_MS = 50;

    // ---- Stall detection configuration ----

    /// @brief All stall detection parameters in one place.
    ///
    /// Sensible defaults are provided for every field — only override
    /// what differs for your motor. At minimum, set `diagPin` and
    /// `sensitivity`. The rest works out of the box for most setups.
    ///
    /// Pass to Tmc2209::configureStall() before begin().
    struct StallConfig {
            /// DIAG output pin on the driver (GPIO_NONE = not wired, no stall detection).
            uint8_t diagPin = motor::GPIO_NONE;

            /// StallGuard sensitivity (0-255). Higher = more sensitive.
            /// Stall triggers when SG_RESULT < 2 × sensitivity.
            uint8_t sensitivity = 0;

            /// DIAG pin path: consecutive HIGH readings needed to confirm a stall.
            int32_t diagConfirmCount = motor::stall::DEFAULT_DIAG_SCORE_LIMIT;

            /// SG_RESULT per step-per-second — the motor's load characteristic slope.
            /// Measured once: run at known speed with no load, read SG_RESULT, divide
            /// by speed. Example: SG=110 at 600 sps → 0.18.
            float sgSlope = motor::stall::DEFAULT_SG_PER_SPS;

            /// Maximum baseline value — SG_RESULT saturates at high speeds.
            /// Set to ~80% of observed SG at max operating speed.
            uint16_t sgMaxBaseline = motor::stall::DEFAULT_SG_BASELINE_CAP;

            /// How far SG must drop from the speed-based baseline to count as
            /// a stall reading (0.0–1.0). Lower = less sensitive.
            float sgDropFraction = motor::stall::DEFAULT_STALL_FRACTION;

            /// Net stall-readings needed to confirm a stall via SG_RESULT path.
            int32_t sgConfirmCount = motor::stall::DEFAULT_SG_SCORE_LIMIT;
    };

    // ============================================================

    /// @brief TMC2209 stepper motor driver — UART register-level control.
    ///
    /// Caches GCONF, CHOPCONF, PWMCONF, and IHOLD_IRUN locally to avoid
    /// read-modify-write round-trips on every field change. Call begin()
    /// once to sync the cache and apply sensible defaults.
    ///
    /// Owns stall detection: DIAG pin polling, SG_RESULT register reads,
    /// speed-based thresholds, and blanking during acceleration.
    class Tmc2209 : public motor::IMotorDriver {
        public:
            /// @param uart      UART port connected to TMC2209 PDN_UART.
            /// @param rSense    External sense resistor value in ohms.
            /// @param stepPin   GPIO for step pulse input on the driver.
            /// @param enablePin GPIO for enable (active LOW).
            /// @param dirPin    GPIO for direction control.
            /// @param address   UART slave address (0-3, set by MS1/MS2 at boot).
            Tmc2209(ungula::uart::Uart& uart, float rSense, uint8_t stepPin, uint8_t enablePin,
                    uint8_t dirPin, uint8_t address = 0);

            // ---- IMotorDriver interface ----

            const char* module() const override;
            const char* info() const override;
            void begin() override;
            void enable() override;
            void disable() override;
            void setDirection(motor::Direction dir) override;
            uint8_t stepPin() const override;
            void setDirectionInverted(bool inverted) override;
            void setMicrosteps(uint16_t microsteps) override;
            void setRunCurrent(uint16_t milliAmps) override;
            uint32_t drvStatus() override;
            uint8_t version() override;

            // Stall detection lifecycle (IMotorDriver)
            void prepareStallDetection(int32_t speedSps, uint32_t accelMs) override;
            void updateStallDetectionSpeed(int32_t speedSps) override;
            void serviceStallDetection() override;
            bool isStalling() const override;
            void clearStall() override;

            // ---- Stall detection configuration (TMC2209-specific) ----

            /// @brief Configure all stall detection parameters at once.
            /// Call before begin(). Fields not set use sensible defaults.
            void configureStall(const StallConfig& config);

            /// @brief Change StallGuard sensitivity at runtime (0-255).
            /// For initial setup, use configureStall() instead.
            void setStallSensitivity(uint8_t sensitivity);

            // ---- Stall diagnostics (for HTTP API and logging) ----

            /// @brief Last SG_RESULT value read via UART.
            uint16_t lastStallGuardResult() const {
                return lastSgResult_;
            }

            /// @brief Current DIAG score (how close to triggering).
            int32_t diagScore() const {
                return stallDetector_.diagScoreNow();
            }

            /// @brief Current SG_RESULT score (how close to triggering).
            int32_t sgScore() const {
                return stallDetector_.sgScoreNow();
            }

            /// @brief Active SG threshold (speed-based).
            uint16_t sgThreshold() const {
                return stallDetector_.sgThresholdNow();
            }

            /// @brief Expected SG baseline at current speed.
            uint16_t sgBaseline() const {
                return stallDetector_.sgBaselineNow();
            }

            /// @brief Current SGTHRS value (0-255).
            uint8_t stallSensitivity() const {
                return stallSensitivity_;
            }

            /// @brief Current DIAG score limit.
            int32_t diagScoreLimit() const {
                return stallDetector_.diagScoreLimitNow();
            }

            // ---- TMC2209-specific register access ----

            void writeRegister(uint8_t regAddr, uint32_t value);
            uint32_t readRegister(uint8_t regAddr);
            static uint8_t calcCrc(const uint8_t* data, uint8_t length);

            /// @brief Read StallGuard result from dedicated register 0x41.
            uint16_t readStallGuardResult();

            uint32_t clearGstat();

            // ---- TMC2209-specific configuration ----

            void setRunCurrent(uint16_t milliAmps, float holdFrac);
            void setHoldFraction(float fraction) {
                holdFraction_ = fraction;
            }
            void setInternalRsense(bool enable);
            void setToff(uint8_t offTime);
            void setBlankTime(uint8_t blankTime);
            void setSpreadCycle(bool enable);
            void setPdnDisable(bool disable);
            void setIScaleAnalog(bool enable);
            void setShaft(bool reverse);
            void setInterpol(bool enable);
            void setPwmAutoscale(bool enable);
            void setPwmAutograd(bool enable);
            void setIholddelay(uint8_t holdDelay);
            void setTpowerdown(uint8_t powerDelay);
            void setTpwmthrs(uint32_t threshold);
            void setTcoolthrs(uint32_t threshold);
            void setStallGuardThreshold(uint8_t threshold);

        private:
            static constexpr uint8_t INFO_BUF_SIZE = 48;

            // Hardware connections
            ungula::uart::Uart& uart_;
            float rSense_;
            const uint8_t stepPin_;
            const uint8_t enablePin_;
            const uint8_t dirPin_;
            uint8_t address_;
            char infoBuf_[INFO_BUF_SIZE] = {};
            float holdFraction_ = defaults::HOLD_FRACTION;

            // Cached register values — avoids read-modify-write round-trips
            uint32_t gconf_ = 0;
            uint32_t chopconf_ = 0;
            uint32_t pwmconf_ = 0;
            uint32_t iholdrun_ = 0;

            // Stall detection state
            uint8_t diagPin_ = motor::GPIO_NONE;
            motor::StallDetector stallDetector_;
            uint8_t stallSensitivity_ = 0;
            uint16_t microsteps_ = defaults::MICROSTEPS;
            uint32_t stallBlankUntilMs_ = 0;
            uint16_t lastSgResult_ = 0xFFFF;
            uint32_t lastSgPollMs_ = 0;
            int32_t targetSpeedSps_ = 0;

            // Helpers
            /// Sensitivity is the gate for all stall detection. DIAG pin is optional
            /// (adds a fast hardware path when wired) but without sensitivity > 0
            /// neither path can trigger.
            bool isStallDetectionEnabled() const {
                return stallSensitivity_ != 0;
            }
            void setBit(uint32_t& cache, uint8_t regAddr, uint32_t mask, bool value);
            void setField(uint32_t& cache, uint8_t regAddr, uint32_t mask, uint32_t value);
            void enableStallDetection();
            void reconfigureStallForSpeed(int32_t speedSps);
            static uint32_t computeTcoolthrs(int32_t speedSps, uint16_t microsteps);
            static uint8_t mresFromMicrosteps(uint16_t microsteps);
    };

}  // namespace tmc
