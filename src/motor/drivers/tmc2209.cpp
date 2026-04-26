// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "tmc2209.h"
#include <hal/gpio/gpio_access.h>
#include <time/time_control.h>
#include <cassert>
#include <cstdio>

namespace tmc {

    // ============================================================
    // CRC8 — polynomial 0x07, LSB-first per byte
    // ============================================================
    uint8_t Tmc2209::calcCrc(const uint8_t* data, uint8_t length) {
        uint8_t crc = 0;
        for (uint8_t idx = 0; idx < length; idx++) {
            uint8_t currentByte = data[idx];
            for (uint8_t bit = 0; bit < BITS_PER_BYTE; bit++) {
                if (((crc >> 7) ^ (currentByte & 0x01)) != 0) {
                    crc = (crc << 1) ^ CRC_POLYNOMIAL;
                } else {
                    crc = crc << 1;
                }
                currentByte >>= 1;
            }
        }
        return crc;
    }

    // ============================================================
    // Raw register write — 8-byte datagram
    // [sync] [addr] [reg|WRITE_FLAG] [d3] [d2] [d1] [d0] [crc]
    // ============================================================
    void Tmc2209::writeRegister(uint8_t regAddr, uint32_t value) {
        uint8_t datagram[WRITE_DATAGRAM_LEN];
        datagram[0] = SYNC_BYTE;
        datagram[1] = address_;
        datagram[2] = regAddr | WRITE_FLAG;
        datagram[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        datagram[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
        datagram[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
        datagram[6] = static_cast<uint8_t>((value >> 0) & 0xFF);
        datagram[7] = calcCrc(datagram, WRITE_DATAGRAM_LEN - 1);

        uart_.flushInput();
        uart_.write(datagram, WRITE_DATAGRAM_LEN);
        uart_.flush();

        // Drain echo on single-wire (our own bytes come back)
        uint8_t echo[WRITE_DATAGRAM_LEN];
        uart_.read(echo, WRITE_DATAGRAM_LEN, ECHO_TIMEOUT_MS);

        ungula::TimeControl::delayMs(INTER_DATAGRAM_MS);
    }

    // ============================================================
    // Raw register read — send 4-byte request, receive 8-byte reply
    // Request: [sync] [addr] [reg] [crc]
    // Reply:   [sync] [0xFF] [reg] [d3] [d2] [d1] [d0] [crc]
    // On single-wire we also see our own 4-byte echo before the reply.
    // ============================================================
    uint32_t Tmc2209::readRegister(uint8_t regAddr) {
        uint8_t request[READ_REQUEST_LEN];
        request[0] = SYNC_BYTE;
        request[1] = address_;
        request[2] = regAddr;
        request[3] = calcCrc(request, READ_REQUEST_LEN - 1);

        uart_.flushInput();
        uart_.write(request, READ_REQUEST_LEN);
        uart_.flush();

        // Read echo (4) + reply (8) with margin
        uint8_t buffer[READ_BUF_SIZE];
        int32_t bytesRead = uart_.read(buffer, sizeof(buffer), REPLY_TIMEOUT_MS);
        if (bytesRead < READ_REPLY_LEN) {
            return 0;
        }

        // Scan for reply: sync + master addr (0xFF) + matching register
        for (int32_t pos = 0; pos <= bytesRead - READ_REPLY_LEN; pos++) {
            if (buffer[pos] != SYNC_BYTE || buffer[pos + 1] != REPLY_MASTER_ADDR ||
                buffer[pos + 2] != regAddr) {
                continue;
            }

            if (calcCrc(&buffer[pos], READ_REPLY_LEN - 1) != buffer[pos + READ_REPLY_LEN - 1]) {
                continue;
            }

            return (static_cast<uint32_t>(buffer[pos + 3]) << 24) |
                   (static_cast<uint32_t>(buffer[pos + 4]) << 16) |
                   (static_cast<uint32_t>(buffer[pos + 5]) << 8) |
                   (static_cast<uint32_t>(buffer[pos + 6]));
        }

        return 0;
    }

    // ============================================================
    // Construction + initialization
    // ============================================================
    Tmc2209::Tmc2209(ungula::uart::Uart& uart, float rSense, uint8_t stepPin, uint8_t enablePin,
                     uint8_t dirPin, uint8_t address)
        : uart_(uart),
          rSense_(rSense),
          stepPin_(stepPin),
          enablePin_(enablePin),
          dirPin_(dirPin),
          address_(address) {
        assert(stepPin_ != motor::GPIO_NONE);
        assert(enablePin_ != motor::GPIO_NONE);
        assert(dirPin_ != motor::GPIO_NONE);
    }

    const char* Tmc2209::module() const {
        return "motor_driver";
    }

    const char* Tmc2209::info() const {
        return infoBuf_;
    }

    void Tmc2209::enable() {
        ungula::gpio::setLow(enablePin_);
    }

    void Tmc2209::disable() {
        ungula::gpio::setHigh(enablePin_);
    }

    void Tmc2209::setDirection(motor::Direction dir) {
        if (dir == motor::Direction::FORWARD) {
            ungula::gpio::setHigh(dirPin_);
        } else {
            ungula::gpio::setLow(dirPin_);
        }
    }

    uint8_t Tmc2209::stepPin() const {
        return stepPin_;
    }

    void Tmc2209::setDirectionInverted(bool inverted) {
        setShaft(inverted);
    }

    void Tmc2209::begin() {
        // Configure GPIO pins owned by the driver
        ungula::gpio::configOutput(enablePin_);
        ungula::gpio::configOutput(dirPin_);
        ungula::gpio::setHigh(enablePin_);  // Start disabled (active LOW)
        ungula::gpio::setLow(dirPin_);

        // Clear global status flags (write 1 to clear)
        writeRegister(reg::GSTAT, GSTAT_CLEAR_ALL);

        // Sync register cache — if UART doesn't respond, all return 0,
        // that's fine because we set all fields explicitly below.
        gconf_ = readRegister(reg::GCONF);
        chopconf_ = readRegister(reg::CHOPCONF);
        pwmconf_ = readRegister(reg::PWMCONF);
        iholdrun_ = readRegister(reg::IHOLD_IRUN);

        // Apply init parameters from config_ — host overrides defaults via setConfig()
        setToff(config_.toff);
        setBlankTime(config_.blankTime);
        setHstrt(config_.hstrt);
        setHend(config_.hend);
        setPwmAutoscale(config_.pwmAutoscale);
        setPwmAutograd(config_.pwmAutograd);
        setSpreadCycle(config_.spreadCycle);
        setPdnDisable(config_.pdnDisable);
        setIScaleAnalog(config_.iScaleAnalog);
        setInternalRsense(config_.internalRsense);
        setInterpol(config_.interpol);
        setMicrosteps(config_.microsteps);
        setIholddelay(config_.iholddelay);
        setTpowerdown(config_.tpowerdown);
        setTpwmthrs(config_.tpwmthrs);
        holdFraction_ = config_.holdFraction;
        setRunCurrent(config_.runCurrentMa, config_.holdFraction);

        // When DIAG pin is configured, set up StallGuard4 registers
        if (diagPin_ != motor::GPIO_NONE) {
            ungula::gpio::configInput(diagPin_);
            enableStallDetection();
        }

        // Build info string after init so version() reads the real IC value
        snprintf(infoBuf_, INFO_BUF_SIZE, "TMC2209 v0x%02X @ UART%u addr=%u", version(),
                 uart_.port(), address_);
    }

    void Tmc2209::setRunCurrent(uint16_t milliAmps) {
        setRunCurrent(milliAmps, holdFraction_);
    }

    // ============================================================
    // Bit/field helpers — update cache, write full register
    // ============================================================
    void Tmc2209::setBit(uint32_t& cache, uint8_t regAddr, uint32_t mask, bool value) {
        if (value) {
            cache |= mask;
        } else {
            cache &= ~mask;
        }
        writeRegister(regAddr, cache);
    }

    void Tmc2209::setField(uint32_t& cache, uint8_t regAddr, uint32_t mask, uint32_t value) {
        cache = (cache & ~mask) | (value & mask);
        writeRegister(regAddr, cache);
    }

    // ============================================================
    // GCONF configuration
    // ============================================================
    void Tmc2209::setInternalRsense(bool enable) {
        setBit(gconf_, reg::GCONF, gconf::INTERNAL_RSENSE, enable);
    }
    void Tmc2209::setSpreadCycle(bool enable) {
        setBit(gconf_, reg::GCONF, gconf::EN_SPREADCYCLE, enable);
    }
    void Tmc2209::setPdnDisable(bool disable) {
        setBit(gconf_, reg::GCONF, gconf::PDN_DISABLE, disable);
    }
    void Tmc2209::setIScaleAnalog(bool enable) {
        setBit(gconf_, reg::GCONF, gconf::I_SCALE_ANALOG, enable);
    }
    void Tmc2209::setShaft(bool reverse) {
        setBit(gconf_, reg::GCONF, gconf::SHAFT, reverse);
    }

    // ============================================================
    // CHOPCONF configuration
    // ============================================================
    void Tmc2209::setToff(uint8_t offTime) {
        setField(chopconf_, reg::CHOPCONF, chop::TOFF_MASK, offTime & 0x0FU);
    }

    void Tmc2209::setBlankTime(uint8_t blankTime) {
        setField(chopconf_, reg::CHOPCONF, chop::TBL_MASK,
                 static_cast<uint32_t>(blankTime & 0x03U) << chop::TBL_SHIFT);
    }

    void Tmc2209::setHstrt(uint8_t hstrt) {
        setField(chopconf_, reg::CHOPCONF, chop::HSTRT_MASK,
                 static_cast<uint32_t>(hstrt & 0x07U) << chop::HSTRT_SHIFT);
    }

    void Tmc2209::setHend(uint8_t hend) {
        setField(chopconf_, reg::CHOPCONF, chop::HEND_MASK,
                 static_cast<uint32_t>(hend & 0x0FU) << chop::HEND_SHIFT);
    }

    void Tmc2209::setMicrosteps(uint16_t microsteps) {
        microsteps_ = microsteps;
        uint8_t mres = mresFromMicrosteps(microsteps);
        setField(chopconf_, reg::CHOPCONF, chop::MRES_MASK,
                 static_cast<uint32_t>(mres) << chop::MRES_SHIFT);

        // Enable mstep_reg_select so MRES in CHOPCONF is used (not MS1/MS2 pins)
        setBit(gconf_, reg::GCONF, gconf::MSTEP_REG_SEL, true);
    }

    void Tmc2209::setInterpol(bool enable) {
        setBit(chopconf_, reg::CHOPCONF, chop::INTPOL, enable);
    }

    // ============================================================
    // PWMCONF configuration
    // ============================================================
    void Tmc2209::setPwmAutoscale(bool enable) {
        setBit(pwmconf_, reg::PWMCONF, pwm::AUTOSCALE, enable);
    }
    void Tmc2209::setPwmAutograd(bool enable) {
        setBit(pwmconf_, reg::PWMCONF, pwm::AUTOGRAD, enable);
    }

    // ============================================================
    // IHOLD_IRUN + current
    // ============================================================
    void Tmc2209::setIholddelay(uint8_t holdDelay) {
        setField(iholdrun_, reg::IHOLD_IRUN, ihr::IHOLDDELAY_MASK,
                 static_cast<uint32_t>(holdDelay & 0x0FU) << ihr::IHOLDDELAY_SHIFT);
    }

    void Tmc2209::setRunCurrent(uint16_t milliAmps, float holdFrac) {
        float rTotal = rSense_ + RSENSE_INTERNAL;
        float currentAmps = static_cast<float>(milliAmps) / MA_PER_AMP;

        // Calculate current scale (CS) with standard sensitivity first
        int32_t currentScale = static_cast<int32_t>(
                (CS_STEPS * SQRT2 * currentAmps * rTotal / VFS_STANDARD) - 1.0F);

        bool vsense = false;
        if (currentScale < CS_VSENSE_THRESHOLD) {
            vsense = true;
            currentScale = static_cast<int32_t>(
                    (CS_STEPS * SQRT2 * currentAmps * rTotal / VFS_HIGH_SENS) - 1.0F);
        }
        if (currentScale > CS_MAX) {
            currentScale = CS_MAX;
        }
        if (currentScale < CS_MIN) {
            currentScale = CS_MIN;
        }

        uint8_t irun = static_cast<uint8_t>(currentScale);
        uint8_t ihold = static_cast<uint8_t>(static_cast<float>(currentScale) * holdFrac);
        if (ihold > CS_MAX) {
            ihold = CS_MAX;
        }

        // Set vsense bit in CHOPCONF
        setBit(chopconf_, reg::CHOPCONF, chop::VSENSE, vsense);

        // Set IHOLD + IRUN, preserving IHOLDDELAY from cache
        iholdrun_ = (iholdrun_ & ihr::IHOLDDELAY_MASK) |
                    (static_cast<uint32_t>(ihold) & ihr::IHOLD_MASK) |
                    ((static_cast<uint32_t>(irun) << ihr::IRUN_SHIFT) & ihr::IRUN_MASK);
        writeRegister(reg::IHOLD_IRUN, iholdrun_);
    }

    // ============================================================
    // Standalone register writes
    // ============================================================
    void Tmc2209::setTpowerdown(uint8_t powerDelay) {
        writeRegister(reg::TPOWERDOWN, powerDelay);
    }
    void Tmc2209::setTpwmthrs(uint32_t threshold) {
        writeRegister(reg::TPWMTHRS, threshold);
    }
    void Tmc2209::setTcoolthrs(uint32_t threshold) {
        writeRegister(reg::TCOOLTHRS, threshold);
    }
    void Tmc2209::setStallGuardThreshold(uint8_t threshold) {
        writeRegister(reg::SGTHRS, threshold);
    }

    void Tmc2209::enableStallDetection() {
        // TMC2209 uses StallGuard4 (SG4) which works in StealthChop mode.
        // SpreadCycle uses SG2, but TMC2209's SG4 is designed for StealthChop.
        setSpreadCycle(false);

        // StealthChop at all speeds — no automatic switchover to SpreadCycle.
        setTpwmthrs(0);

        // Disable CoolStep — it adjusts current dynamically and interferes
        // with StallGuard measurements.
        writeRegister(reg::COOLCONF, 0);

        // TCOOLTHRS = max 20-bit value: stall detection always active.
        // Refined per motion start via setTcoolthrs() with speed-based value.
        constexpr uint32_t TCOOLTHRS_ALWAYS_ON = 0xFFFFF;
        setTcoolthrs(TCOOLTHRS_ALWAYS_ON);
    }

    // ============================================================
    // Status reads
    // ============================================================
    uint8_t Tmc2209::version() {
        uint32_t ioinReg = readRegister(reg::IOIN);
        return static_cast<uint8_t>((ioinReg >> ioin::VERSION_SHIFT) & 0xFF);
    }

    uint32_t Tmc2209::drvStatus() {
        return readRegister(reg::DRV_STATUS);
    }
    uint32_t Tmc2209::clearGstat() {
        return readRegister(reg::GSTAT);
    }

    uint16_t Tmc2209::readStallGuardResult() {
        constexpr uint16_t SG_RESULT_MASK = 0x3FFU;
        uint32_t raw = readRegister(reg::SG_RESULT);
        return static_cast<uint16_t>(raw & SG_RESULT_MASK);
    }

    // ============================================================
    // Stall detection configuration
    // ============================================================

    void Tmc2209::configureStall(const StallConfig& config) {
        diagPin_ = config.diagPin;
        stallSensitivity_ = config.sensitivity;
        stallDetector_.setDiagScoreLimit(config.diagConfirmCount);
        stallDetector_.setSgPerSps(config.sgSlope);
        stallDetector_.setSgBaselineCap(config.sgMaxBaseline);
        stallDetector_.setStallFraction(config.sgDropFraction);
        stallDetector_.setScoreLimit(config.sgConfirmCount);
    }

    void Tmc2209::setStallSensitivity(uint8_t sensitivity) {
        stallSensitivity_ = sensitivity;
    }

    // ============================================================
    // Stall detection lifecycle (IMotorDriver)
    // ============================================================

    void Tmc2209::prepareStallDetection(int32_t speedSps, uint32_t accelMs) {
        stallDetector_.clear();
        stallDetector_.configureForSpeed(speedSps);
        targetSpeedSps_ = speedSps;

        // Suppress stall detection during the acceleration ramp.
        // TMC2209 StallGuard is unreliable at low speed — DIAG reads HIGH
        // at standstill or during ramp-up, causing false stall triggers.
        stallBlankUntilMs_ = ungula::TimeControl::millis() + accelMs + STALL_SETTLE_MARGIN_MS;

        reconfigureStallForSpeed(speedSps);
    }

    void Tmc2209::updateStallDetectionSpeed(int32_t speedSps) {
        targetSpeedSps_ = speedSps;
        stallDetector_.configureForSpeed(speedSps);
        reconfigureStallForSpeed(speedSps);
    }

    void Tmc2209::serviceStallDetection() {
        if (!isStallDetectionEnabled()) {
            return;
        }

        uint32_t nowMs = ungula::TimeControl::syncNow();

        // Always read SG_RESULT while motor is running (throttled to avoid
        // UART blocking). Independent of blanking — short moves with long
        // accel ramps would never get a reading otherwise.
        if ((nowMs - lastSgPollMs_) >= motor::svc::SG_POLL_INTERVAL_MS) {
            lastSgPollMs_ = nowMs;
            lastSgResult_ = readStallGuardResult();
        }

        // Stall scoring — only after blanking period expires
        bool stallBlankActive = (nowMs < stallBlankUntilMs_);
        if (stallBlankActive) {
            return;
        }

        bool lowSpeedMode = (targetSpeedSps_ < motor::stall::LOW_SPEED_SPS);

        // Path 1: DIAG pin — suppressed at low speeds where it's unreliable
        if (diagPin_ != motor::GPIO_NONE && !lowSpeedMode) {
            bool diagHigh = ungula::gpio::isHigh(diagPin_);
            stallDetector_.pollPin(diagHigh);
        }

        // Path 2: SG_RESULT register — speed-scaled threshold, all speeds
        stallDetector_.pollRegister(lastSgResult_);
    }

    bool Tmc2209::isStalling() const {
        return stallDetector_.isStalling();
    }

    void Tmc2209::clearStall() {
        stallDetector_.clear();
    }

    // ============================================================
    // Stall detection helpers
    // ============================================================

    // Reconfigure StallGuard registers for the target speed.
    // Sensitivity and TCOOLTHRS are speed-dependent.
    void Tmc2209::reconfigureStallForSpeed(int32_t speedSps) {
        if (stallSensitivity_ == 0) {
            return;
        }
        setStallGuardThreshold(stallSensitivity_);
        setTcoolthrs(computeTcoolthrs(speedSps, microsteps_));
    }

    // TCOOLTHRS calculation — see local_motor.cpp for the full explanation.
    // TSTEP = fCLK * microsteps / (stepFrequency * 256)
    // We set TCOOLTHRS slightly above (1.5x) the expected TSTEP at target speed.
    uint32_t Tmc2209::computeTcoolthrs(int32_t speedSps, uint16_t microsteps) {
        if (speedSps <= 0) {
            return 0;
        }
        float tstep = (TMC_FCLK * static_cast<float>(microsteps)) /
                      (static_cast<float>(speedSps) * TMC_INTERNAL_USTEPS);
        return static_cast<uint32_t>(tstep * TCOOLTHRS_MARGIN);
    }

    // ============================================================
    // Helpers
    // ============================================================
    uint8_t Tmc2209::mresFromMicrosteps(uint16_t microsteps) {
        switch (microsteps) {
            case 256:
                return 0;
            case 128:
                return 1;
            case 64:
                return 2;
            case 32:
                return 3;
            case 16:
                return 4;
            case 8:
                return 5;
            case 4:
                return 6;
            case 2:
                return 7;
            case 1:
                return 8;
            default:
                return 4;  // 16 microsteps
        }
    }

}  // namespace tmc
