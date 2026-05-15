// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include <memory>

#include "ungula/hal/uart/uart.h"

#include "ungula/motor/axis.h"
#include "ungula/motor/axis_config.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_configurator.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_coolstep.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h"
#include "ungula/motor/drivers/tmc2209/tmc2209_stallguard.h"
#include "ungula/motor/result.h"

namespace ungula::motor::tmc2209
{

/// Single-call bundle for "set up one TMC2209-driven axis from
/// scratch". Wraps the pieces a TMC2209 host would otherwise have
/// to wire by hand: HAL UART (optional — can be shared), `ITmcUart`
/// transport, `Tmc2209Configurator`, optional `Tmc2209StallGuard`,
/// optional `Tmc2209CoolStep`, and the `Axis` itself (with its HAL
/// timer, pulse engine, and STEP/DIR actuator).
///
/// ## When to use the kit vs. composing by hand
///
/// The kit is the recommended entry point for new code. It covers
/// the common case: one TMC2209 with optional stall + CoolStep,
/// wired to a single STEP/DIR/EN trio, optionally sharing a UART
/// with other chips.
///
/// Compose by hand only when you need a non-standard arrangement:
/// custom pulse engine, custom actuator, or a HAL UART with
/// non-default RX/TX buffer sizes.
///
/// ## N-motor projects
///
/// The kit owns instance state in member `unique_ptr`s — no
/// file-scope state anywhere. Construct as many kits as your
/// project needs. Two motors sharing one UART go through
/// `makeStepperKitOnUart()`; see below.
///
/// ## Lifetime
///
/// A `StepperKit` owns everything it constructed. Destroying it
/// destroys the axis, configurator, stall/CoolStep helpers, the
/// transport, and (if owned) the UART. `makeStepperKitOnUart()`
/// does NOT own its UART; the caller must keep the UART alive at
/// least until every kit using it has been destroyed.
struct StepperKitConfig {
        AxisCommonConfig common{};

        // ---- Pin wiring -------------------------------------------------
        // Primary drive.
        StepPin stepPin{};
        DirectionPin dirPin{};
        EnablePin enablePin{}; // GPIO_NONE allowed; some hosts hardwire EN
        bool dirActiveHigh = true;
        bool enableActiveLow = true; // TMC2209 EN is active-LOW
        uint32_t dirSetupUs = 5;

        /// Optional secondary tandem wiring (two TMC2209s sharing this
        /// STEP pin). Leave at GPIO_NONE for the single-motor case.
        DirectionPin secondaryDirPin{};
        bool secondaryDirActiveHigh = true;
        bool secondaryDirInverted = false;
        EnablePin secondaryEnablePin{};
        bool secondaryEnableActiveLow = true;

        // ---- UART transport (only used by `makeStepperKit()`) ----------
        /// Hardware UART peripheral index (e.g. 1 or 2 on ESP32).
        /// Ignored by `makeStepperKitOnUart()`, which uses the
        /// caller-provided UART instead.
        uint8_t uartPort = 1;
        uint32_t uartBaud = 115200;
        uint8_t uartTxPin = 0;
        uint8_t uartRxPin = 0;
        /// TMC2209 NAI[1:0] address, 0..3. Used by the
        /// owned-UART factory; `makeStepperKitOnUart()` takes an
        /// explicit parameter instead.
        uint8_t slaveAddress = 0;

        // ---- TMC2209 chip configuration ---------------------------------
        Tmc2209Configurator::Config chip{};

        // ---- Optional StallGuard ----------------------------------------
        bool useStallGuard = false;
        Tmc2209StallGuard::Config stall{};

        // ---- Optional CoolStep ------------------------------------------
        bool useCoolStep = false;
        Tmc2209CoolStep::Config coolStep{};

        // ---- Sensors ----------------------------------------------------
        SensorInputConfig sensors[MAX_SENSOR_INPUTS]{};
        uint8_t sensorCount = 0;
};

/// Bundle returned by `makeStepperKit()` / `makeStepperKitOnUart()`.
/// All helper members are public so callers can reach them at
/// runtime (e.g. `kit->stallGuard->setSgThreshold(x)` to tune SGTHRS
/// while running). `axis` is the production handle for motion calls.
class StepperKit {
    public:
        /// One-shot lifecycle. In order:
        ///   1. `uart->begin()` — only when this kit owns the UART
        ///      (caller used `makeStepperKit`, not the shared-UART
        ///      factory).
        ///   2. `configurator->begin(chip)`.
        ///   3. `stallGuard->begin(stall)` — if `useStallGuard`.
        ///   4. `coolStep->begin(coolStep)` — if `useCoolStep`.
        ///   5. `axis->begin()`.
        ///
        /// On any failure, the kit is left partially initialised.
        /// Destroy and report. Idempotent at the axis layer only
        /// (`AlreadyInitialized` is treated as success there).
        Status begin();

        // ---- Constructed helpers ---------------------------------------
        /// UART owned by this kit. Null for shared-UART kits.
        std::unique_ptr<ungula::hal::uart::Uart> uart;
        std::unique_ptr<Tmc2209HalUart> transport;
        std::unique_ptr<Tmc2209Configurator> configurator;
        /// Null when `useStallGuard == false`.
        std::unique_ptr<Tmc2209StallGuard> stallGuard;
        /// Null when `useCoolStep == false`.
        std::unique_ptr<Tmc2209CoolStep> coolStep;
        std::unique_ptr<Axis> axis;

        // ---- Stored config (consumed by `begin()`) ---------------------
        // Public so callers can inspect / tweak before `begin()` runs.
        StepperKitConfig storedCfg{};
        /// True when this kit owns its UART (set by the factories).
        bool ownsUart = false;
};

/// Construct a TMC2209 kit that owns its UART. The kit's `begin()`
/// will call `uart->begin()` using the baud / pin fields in `cfg`.
/// Returns `InvalidConfig` if `stepPin` or `dirPin` is unset, if
/// the slave address is out of range, or if the trajectory limits
/// are not populated. The chip is NOT touched until `begin()`.
Result<std::unique_ptr<StepperKit>> makeStepperKit(const StepperKitConfig &cfg);

/// Construct a TMC2209 kit that shares a UART with other kits. The
/// caller must have called `uart.begin()` before invoking the kit's
/// `begin()`. Suitable for N-motor projects where 2..4 TMC2209
/// chips share one bus, each with its own slave address.
///
/// ```
/// ungula::hal::uart::Uart bus(2);
/// bus.begin(115200, txPin, rxPin);
/// auto k1 = makeStepperKitOnUart(bus, /*addr=*/0, cfgA);
/// auto k2 = makeStepperKitOnUart(bus, /*addr=*/1, cfgB);
/// k1.value()->begin();
/// k2.value()->begin();
/// ```
///
/// `cfg.slaveAddress` is ignored — the explicit `slaveAddress`
/// parameter wins.
Result<std::unique_ptr<StepperKit>>
makeStepperKitOnUart(ungula::hal::uart::Uart &uart, uint8_t slaveAddress,
                     const StepperKitConfig &cfg);

} // namespace ungula::motor::tmc2209
