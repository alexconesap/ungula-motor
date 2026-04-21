// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#ifdef MOTOR_DRIVER_TMC2209

#include "tmc_stepper.h"

#include <emblogx/logger.h>
#include <hal/gpio/gpio_access.h>
#include <time/time_control.h>

namespace ungula {
    namespace motor {

        // Debug cadence
        static constexpr uint32_t DEBUG_POLL_MS = 200;
        static constexpr uint16_t MSCNT_MIN_DELTA = 8;

        TmcStepper::TmcStepper(int stepPin, int enablePin, int dirPin, HardwareSerial& serialPort,
                               float rSense, uint8_t driverAddress, int rxPin, int txPin)
            : stepPin_(stepPin),
              enablePin_(enablePin),
              dirPin_(dirPin),
              serialPort_(serialPort),
              rSense_(rSense),
              driverAddress_(driverAddress),
              rxPin_(rxPin),
              txPin_(txPin),
              driver_(&serialPort, rSense, driverAddress) {}

        void TmcStepper::begin() {
            ungula::gpio::configOutput(stepPin_);
            ungula::gpio::configOutput(enablePin_);
            ungula::gpio::configOutput(dirPin_);

            // Default: disabled, DIR low
            ungula::gpio::setHigh(enablePin_);
            ungula::gpio::setLow(dirPin_);

            // UART mapping
            serialPort_.begin(115200, SERIAL_8N1, rxPin_, txPin_);

            driver_.begin();

            // spreadCycle-only profile (stable torque)
            driver_.en_spreadCycle(true);
            driver_.toff(5);
            driver_.blank_time(24);
            driver_.hstrt(5);
            driver_.hend(2);
            driver_.tbl(2);
            driver_.intpol(true);
            driver_.microsteps(16);

            // No stealthChop while running
            driver_.pwm_autoscale(false);
            driver_.pwm_autograd(false);
            driver_.TPWMTHRS(0);

            // Current control via UART
            driver_.I_scale_analog(false);

            // Safe default current
            setRunCurrent(1100, HOLD_FRAC);

            driver_.iholddelay(10);
            driver_.TPOWERDOWN(20);
            driver_.pdn_disable(true);

            // Clear sticky flags
            (void)driver_.GSTAT();
        }

        void TmcStepper::setRunCurrent(uint16_t run_mA, float hold_frac) {
            if (hold_frac < 0.10f)
                hold_frac = 0.10f;
            if (hold_frac > 0.50f)
                hold_frac = 0.50f;
            driver_.I_scale_analog(false);
            driver_.rms_current(run_mA, hold_frac);
        }

        void TmcStepper::setCurrentBySps(int sps) {
            // Normalize speed into [0,1] band
            float t;
            if (sps <= SPS_LO)
                t = 0.0f;
            else if (sps >= SPS_HI)
                t = 1.0f;
            else
                t = (float)(sps - SPS_LO) / (float)(SPS_HI - SPS_LO);

            // Smoothstep (cubic) for a gentle S-curve
            float u = t * t * (3.0f - 2.0f * t);

            // Interpolate current and apply
            uint16_t mA = (uint16_t)(I_LO_MA + (I_HI_MA - I_LO_MA) * u + 0.5f);
            setRunCurrent(mA, HOLD_FRAC);
        }

        void TmcStepper::setDirection(bool clockwise) {
            ungula::gpio::write(dirPin_, clockwise);
        }

        void TmcStepper::setEnable(bool enable) {
            ungula::gpio::write(enablePin_, !enable);
        }

        void TmcStepper::debugPoll() {
            static uint32_t last_ms = 0;

            const uint32_t now = TimeControl::millis();
            if (now - last_ms < DEBUG_POLL_MS)
                return;
            last_ms = now;

            uint8_t gstat = driver_.GSTAT();
            uint32_t drv_raw = driver_.DRV_STATUS();
            uint32_t ioin = driver_.IOIN();
            int en_gpio = ungula::gpio::read(enablePin_) ? 1 : 0;
            uint16_t mscnt = driver_.MSCNT() & 0x3FF;

            bool otpw = driver_.otpw(), ot = driver_.ot();
            bool s2ga = driver_.s2ga(), s2gb = driver_.s2gb();
            bool s2vsa = driver_.s2vsa(), s2vsb = driver_.s2vsb();
            bool ola = driver_.ola(), olb = driver_.olb();
            bool t120 = driver_.t120(), t150 = driver_.t150();
            uint8_t cs = driver_.cs_actual();

            uint16_t d = 0;
            static uint16_t last = 0xFFFF;
            if (last != 0xFFFF)
                d = (mscnt >= last) ? (mscnt - last) : (uint16_t)(1024 - last + mscnt);
            last = mscnt;

            char flags[64] = "";
            if (otpw || ot || s2ga || s2gb || s2vsa || s2vsb || ola || olb || t120 || t150) {
                char* p = flags;
                p += sprintf(p, " FLAGS:");
                if (otpw)
                    p += sprintf(p, " OTPW");
                if (ot)
                    p += sprintf(p, " OT");
                if (t120)
                    p += sprintf(p, " T120");
                if (t150)
                    p += sprintf(p, " T150");
                if (s2ga)
                    p += sprintf(p, " S2GA");
                if (s2gb)
                    p += sprintf(p, " S2GB");
                if (s2vsa)
                    p += sprintf(p, " S2VSA");
                if (s2vsb)
                    p += sprintf(p, " S2VSB");
                if (ola)
                    p += sprintf(p, " OLA");
                if (olb)
                    p += sprintf(p, " OLB");
            }

            log_debug("[TMC] EN=%d GSTAT=0x%02X DRV=0x%08lX IOIN=0x%08lX MSCNT=%u d=%u CS=%d%s",
                      en_gpio, gstat, drv_raw, ioin, mscnt, d, (int)cs, flags);

            if (otpw || ot || s2ga || s2gb || s2vsa || s2vsb || ola || olb || t120 || t150 ||
                (gstat & 0x03)) {
                dumpBanner("TMC FAULT");
            }
        }

        void TmcStepper::dumpBanner(const char* reason) {
            uint8_t gstat = driver_.GSTAT();
            uint32_t drv = driver_.DRV_STATUS();
            uint32_t ioin = driver_.IOIN();
            int en_gpio = ungula::gpio::read(enablePin_) ? 1 : 0;

            bool otpw = driver_.otpw(), ot = driver_.ot();
            bool s2ga = driver_.s2ga(), s2gb = driver_.s2gb();
            bool s2vsa = driver_.s2vsa(), s2vsb = driver_.s2vsb();
            bool ola = driver_.ola(), olb = driver_.olb();
            bool t120 = driver_.t120(), t150 = driver_.t150();

            char flags[64] = "";
            char* p = flags;
            if (otpw)
                p += sprintf(p, " OTPW");
            if (ot)
                p += sprintf(p, " OT");
            if (t120)
                p += sprintf(p, " T120");
            if (t150)
                p += sprintf(p, " T150");
            if (s2ga)
                p += sprintf(p, " S2GA");
            if (s2gb)
                p += sprintf(p, " S2GB");
            if (s2vsa)
                p += sprintf(p, " S2VSA");
            if (s2vsb)
                p += sprintf(p, " S2VSB");
            if (ola)
                p += sprintf(p, " OLA");
            if (olb)
                p += sprintf(p, " OLB");

            log_error("================================================================");
            log_error("=====                   TMC EVENT BANNER                  =====");
            log_error("================================================================");
            log_error("Reason: %s", reason ? reason : "EVENT");
            log_error("EN_GPIO=%d  GSTAT=0x%02X  DRV=0x%08lX  IOIN=0x%08lX", en_gpio, gstat, drv,
                      ioin);
            log_error("Flags:%s", flags[0] ? flags : " (none)");
            log_error("================================================================");
        }

    }  // namespace motor
}  // namespace ungula

#endif  // MOTOR_DRIVER_TMC2209
