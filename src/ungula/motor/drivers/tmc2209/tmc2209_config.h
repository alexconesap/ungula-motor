// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/motor_step_timing.h"
#include "ungula/motor/motor_units.h" // GPIO_NONE

namespace ungula::motor::tmc2209
{

/// Microstep depth selector. Values are the chip's `MRES` field
/// in CHOPCONF — the host-facing names (16×, 32×, …) make the
/// multiplier obvious. The driver caches the active depth and exposes
/// it via diagnostics.
enum class MicrostepDepth : uint8_t {
        x1 = 8, // = full step (rough, discouraged)
        x2 = 7,
        x4 = 6,
        x8 = 5,
        x16 = 4,
        x32 = 3,
        x64 = 2,
        x128 = 1,
        x256 = 0,
};

/// Returns the physical microstep multiplier (1, 2, 4, …, 256) for a
/// depth value. Useful when the host wants to compute steps-per-rev
/// from the chip configuration without keeping a separate constant in
/// sync.
constexpr uint32_t microstepMultiplier(MicrostepDepth d)
{
        // MicrostepDepth values are the chip's MRES (inverse log).
        // Convert by computing 2^(8 - MRES).
        return 1u << (8u - static_cast<uint32_t>(d));
}

/// Default StallGuard sensitivity used by `Tmc2209Config` when the
/// host doesn't set one explicitly. 50 % puts SGTHRS at 127 and the
/// trip threshold at SG_RESULT = 254 - a defensible midpoint for a
/// first-light setup. Hosts that have tuned to their motor should
/// override the field with `StallSensitivity::pct(...)` or
/// `StallSensitivity::rawSgthrs(...)` directly.
constexpr uint8_t kDefaultStallSensitivityPct = 50;

/// StallGuard4 sensitivity expressed in the unit the host wants.
/// Mirrors the Speed / Distance / Acceleration tagged-value pattern
/// the rest of the lib uses: the host picks the unit at the call
/// site, the lib converts to the chip's native register byte
/// internally. Two units:
///
///   - `Pct` (0..100). The lib computes SGTHRS = pct * 255 / 100.
///     Good for "I want roughly this much sensitivity" without
///     reading the datasheet. Higher pct = higher SGTHRS = chip
///     trips at higher SG_RESULT (more sensitive).
///   - `RawSgthrs` (0..255). The literal byte written to SGTHRS,
///     so the host can pin a value they read off the chip's
///     SG_RESULT dump (`Tmc2209Driver::readStallSnapshot`) and
///     not have to translate back to a percentage. The chip's
///     trip is then `SG_RESULT < 2 * RawSgthrs`.
///
/// The chip's silicon is the same either way; only the host-side
/// convenience changes.
struct StallSensitivity {
        enum class Unit : uint8_t {
                Pct = 0,
                RawSgthrs = 1,
        };

        uint16_t value = kDefaultStallSensitivityPct;
        Unit unit = Unit::Pct;

        static constexpr StallSensitivity pct(uint8_t v)
        {
                return { v, Unit::Pct };
        }
        static constexpr StallSensitivity rawSgthrs(uint8_t sgthrs)
        {
                return { sgthrs, Unit::RawSgthrs };
        }

        /// Resolve to the byte the lib writes into SGTHRS. Clamps
        /// out-of-range Pct values; passes RawSgthrs through.
        constexpr uint8_t toSgthrsByte() const
        {
                if (unit == Unit::Pct) {
                        const uint16_t clamped = (value > 100u) ? 100u : value;
                        return static_cast<uint8_t>((clamped * 255u) / 100u);
                }
                return static_cast<uint8_t>(value & 0xFFu);
        }
};

/// Host-facing TMC2209 configuration. The host fills this once when
/// it constructs the driver. No chip-internal jargon appears as a
/// field name — `runCurrentMa` is mA, `stallSensitivity` carries
/// either a percentage or a raw SGTHRS byte (see `StallSensitivity`).
/// Driver-private mapping (mA → IRUN, % → SGTHRS, etc.) lives inside
/// the driver implementation, NOT here.
struct Tmc2209Config {
        // ---- Bus ---------------------------------------------------------
        /// 0..3 — matches the NAI[1:0] pin configuration of the chip on
        /// the shared UART. Out-of-range is rejected at `begin()`.
        uint8_t slaveAddress = 0;

        // ---- Pins --------------------------------------------------------
        /// Driver-enable pin (EN). Set to `GPIO_NONE` for boards that
        /// hard-wire EN to ground. The chip's coils are de-energised at
        /// boot; the driver only writes EN during `enable()` /
        /// `disable()`.
        uint8_t enablePin = GPIO_NONE;
        /// EN polarity. Every TMC2209 carrier I've seen (BTT, Watterott,
        /// FYSETC) is active-LOW. Flip only for a custom board.
        bool enableActiveLow = true;

        // ---- Coil currents ----------------------------------------------
        /// Run current in mA RMS. Driver converts to the chip's 5-bit
        /// `IRUN` field using `senseResistorOhms` + the vsense bit. Out
        /// of range or zero is rejected with `InvalidConfig` at
        /// `begin()`.
        uint16_t runCurrentMa = 800;
        /// Hold current in mA RMS. Common practice: 30–50 % of run.
        uint16_t holdCurrentMa = 400;
        /// Sense resistor on the carrier board, in ohms. **Wrong value
        /// here = wrong actual current** — check the silk screen on
        /// your board (0.11 Ω BTT, 0.075 Ω Watterott, 0.05 Ω some
        /// industrial designs).
        float senseResistorOhms = 0.11f;
        /// Vsense bit. `false` (default) = low-sensitivity range (best
        /// for currents above ~300 mA RMS). `true` = high-sensitivity
        /// range (use for low-current setups so you're not stuck at
        /// IRUN=1).
        bool useHighSensitivity = false;

        // ---- Microstepping ----------------------------------------------
        MicrostepDepth microsteps = MicrostepDepth::x16;
        /// Microstep interpolation to 256. The chip interpolates
        /// internally; the host just turns it on. Smooths coarse step
        /// rates without changing the host's commanded step count.
        bool interpolate = true;

        // ---- Chopper (CHOPCONF) -----------------------------------------
        /// TOFF: chopper-off time (datasheet 8.1). Range 1-15; 0 disables
        /// the chopper entirely. Default 5 is a practical baseline: long
        /// enough for stable current regulation on typical NEMA17 / 23,
        /// short enough not to add audible high-frequency noise.
        uint8_t toff = 5;
        /// TBL: chopper blank time, expressed as the datasheet's clock-
        /// cycle value (16 / 24 / 36 / 54). Default 24 clk is a balanced
        /// starting point. Higher values quiet the current sense at very low
        /// load; lower values respond faster but can mis-sense at the
        /// motor's commutation edge.
        uint8_t blankTimeClk = 24;

        // ---- PWMCONF (StealthChop chopper baseline) ---------------------
        /// pwm_autoscale: chip auto-tunes PWM_OFS during operation.
        /// Default true (matches TMCStepper's `pwm_autoscale(true)`).
        /// Disable only when you're driving PWM_OFS / PWM_GRAD by hand.
        bool pwmAutoscale = true;
        /// pwm_autograd: chip auto-tunes PWM_GRAD during operation.
        /// Default true.
        bool pwmAutograd = true;
        /// pwm_freq selector (datasheet 5.4.5): 0 = 2/1024 fCLK,
        /// 1 = 2/683 fCLK, 2 = 2/512 fCLK, 3 = 2/410 fCLK. At the
        /// chip's 12 MHz internal clock those land at 23.4 / 35.2 /
        /// 46.9 / 58.5 kHz. Default 1 (35 kHz) is the TMCStepper
        /// default and works for almost every NEMA-class motor.
        uint8_t pwmFreq = 1;

        // ---- STEP / DIR pins (self-owns ctor only) -----------------------
        // Used ONLY by the self-owns constructor
        // (`Tmc2209Driver(cfg, transport)`). The pluggable constructor
        // ignores these because the host's step signal generator was
        // already configured externally.
        uint8_t stepPin = GPIO_NONE;
        uint8_t dirPin = GPIO_NONE;
        bool dirActiveHigh = true;
        uint32_t dirSetupUs     = timing::kDefaultDirSetupUs;
        uint32_t minPulseHighUs = timing::kDefaultMinPulseHighUs;
        uint32_t minPulseLowUs  = timing::kDefaultMinPulseLowUs;

        // ---- Stall detection (optional) ----------------------------------
        /// DIAG output pin from the chip, wired to a host GPIO as
        /// `LimitKind::StallSensor` in the host's `MotorAxisConfig`.
        /// Set to `GPIO_NONE` to disable stall detection — the driver
        /// won't write SGTHRS / TCOOLTHRS when no DIAG pin is wired.
        uint8_t diagPin = GPIO_NONE;
        /// Stall threshold. Use the tagged-value factories:
        ///
        ///   chip.stallSensitivity = tmc::StallSensitivity::pct(35);
        ///   // or, for raw datasheet-level control:
        ///   chip.stallSensitivity = tmc::StallSensitivity::rawSgthrs(90);
        ///
        /// Default = 50 % (SGTHRS = 127). Tune against your mechanics
        /// by reading `Tmc2209Driver::readStallSnapshot()` while the
        /// motor runs free and while you block it: the trip
        /// (`SGTHRS * 2`) needs to sit between those two SG_RESULT
        /// readings.
        StallSensitivity stallSensitivity = StallSensitivity::pct(kDefaultStallSensitivityPct);
};

} // namespace ungula::motor::tmc2209
