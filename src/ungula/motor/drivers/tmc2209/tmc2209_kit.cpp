// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/drivers/tmc2209/tmc2209_kit.h"

#include "ungula/motor/limits/sensor_input.h" // GPIO_NONE

namespace ungula::motor::tmc2209
{

namespace
{

bool pinSet(uint8_t v)
{
        return v != GPIO_NONE;
}

/// Builds the underlying axis config from the kit's pin / motion
/// fields. Pulled out so both factories agree on what gets copied
/// where. Chip config is applied separately at `begin()` time, after
/// the transport is up.
StepDirStepperAxisConfig buildAxisConfig(const StepperKitConfig &cfg)
{
        StepDirStepperAxisConfig a{};
        a.common = cfg.common;
        a.stepPin = cfg.stepPin;
        a.dirPin = cfg.dirPin;
        a.enablePin = cfg.enablePin;
        a.dirActiveHigh = cfg.dirActiveHigh;
        a.enableActiveLow = cfg.enableActiveLow;
        a.dirSetupUs = cfg.dirSetupUs;
        a.secondaryDirPin = cfg.secondaryDirPin;
        a.secondaryDirActiveHigh = cfg.secondaryDirActiveHigh;
        a.secondaryDirInverted = cfg.secondaryDirInverted;
        a.secondaryEnablePin = cfg.secondaryEnablePin;
        a.secondaryEnableActiveLow = cfg.secondaryEnableActiveLow;
        for (uint8_t i = 0; i < cfg.sensorCount && i < MAX_SENSOR_INPUTS; ++i) {
                a.sensors[i] = cfg.sensors[i];
        }
        a.sensorCount = cfg.sensorCount;
        return a;
}

Status validateKitInputs(const StepperKitConfig &cfg, uint8_t slaveAddress)
{
        if (!pinSet(cfg.stepPin.value) || !pinSet(cfg.dirPin.value)) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        if (slaveAddress > 3) {
                return Status::Err(ErrorCode::InvalidConfig);
        }
        return Status::Ok();
}

} // namespace

Status StepperKit::begin()
{
        // 1. UART. Only when this kit owns it — the shared-UART factory
        // assumes the caller already called `uart.begin()`.
        if (ownsUart) {
                if (!uart) {
                        // Programmer error: ownership flag without an
                        // owned object.
                        return Status::Err(ErrorCode::InternalError);
                }
                if (!uart->begin(storedCfg.uartBaud, storedCfg.uartTxPin,
                                 storedCfg.uartRxPin)) {
                        return Status::Err(ErrorCode::TransportError);
                }
        }

        // 2. Chip configuration. This is the first traffic on the wire.
        if (!configurator) {
                return Status::Err(ErrorCode::InternalError);
        }
        {
                const auto s = configurator->begin(storedCfg.chip);
                if (!s.ok())
                        return s;
        }

        // 3. Optional StallGuard.
        if (stallGuard) {
                const auto s = stallGuard->begin(storedCfg.stall);
                if (!s.ok())
                        return s;
        }

        // 4. Optional CoolStep.
        if (coolStep) {
                const auto s = coolStep->begin(storedCfg.coolStep);
                if (!s.ok())
                        return s;
        }

        // 5. Axis. The kit doesn't `enable()` — the host decides when
        // to release the brake stage and start motion.
        if (!axis) {
                return Status::Err(ErrorCode::InternalError);
        }
        const auto s = axis->begin();
        if (!s.ok() && s.error() != ErrorCode::AlreadyInitialized) {
                return s;
        }
        return Status::Ok();
}

namespace
{

/// Shared kit-assembly path. `ownedUart` is null for the shared-UART
/// case, in which case `sharedUart` must be non-null and pre-begun.
/// One of the two is always non-null.
Result<std::unique_ptr<StepperKit>>
assembleKit(const StepperKitConfig &cfg, uint8_t slaveAddress,
            std::unique_ptr<ungula::hal::uart::Uart> ownedUart,
            ungula::hal::uart::Uart *sharedUart)
{
        const auto v = validateKitInputs(cfg, slaveAddress);
        if (!v.ok())
                return Result<std::unique_ptr<StepperKit>>::Err(v.error());

        ungula::hal::uart::Uart *uartRef = ownedUart ? ownedUart.get() : sharedUart;
        // Defensive: assembleKit's contract says exactly one is non-null.
        if (!uartRef) {
                return Result<std::unique_ptr<StepperKit>>::Err(ErrorCode::InternalError);
        }

        // Build transport + configurator BEFORE the axis. The
        // configurator IS the `IDriverIdentityProvider` for the
        // TMC2209 — identity lives on the driver class, no separate
        // provider object needed. The configurator outlives the
        // actuator: it's declared in the kit struct in an order that
        // destructs after.
        auto transport = std::make_unique<Tmc2209HalUart>(*uartRef, slaveAddress);
        auto configurator = std::make_unique<Tmc2209Configurator>(*transport);

        std::unique_ptr<Tmc2209StallGuard> stallGuard;
        if (cfg.useStallGuard) {
                stallGuard = std::make_unique<Tmc2209StallGuard>(*transport);
        }
        std::unique_ptr<Tmc2209CoolStep> coolStep;
        if (cfg.useCoolStep) {
                coolStep = std::make_unique<Tmc2209CoolStep>(*transport);
        }

        auto axisCfg = buildAxisConfig(cfg);
        axisCfg.identityProvider = configurator.get();
        auto axisRes = Axis::createStepDirStepper(axisCfg);
        if (!axisRes.ok()) {
                return Result<std::unique_ptr<StepperKit>>::Err(axisRes.error());
        }

        auto kit = std::make_unique<StepperKit>();
        kit->ownsUart = (ownedUart != nullptr);
        kit->uart = std::move(ownedUart);
        kit->transport = std::move(transport);
        kit->configurator = std::move(configurator);
        kit->stallGuard = std::move(stallGuard);
        kit->coolStep = std::move(coolStep);
        kit->axis = axisRes.takeValue();
        kit->storedCfg = cfg;
        // Make sure the explicit slave address wins over the field.
        kit->storedCfg.slaveAddress = slaveAddress;

        return Result<std::unique_ptr<StepperKit>>::Ok(std::move(kit));
}

} // namespace

Result<std::unique_ptr<StepperKit>> makeStepperKit(const StepperKitConfig &cfg)
{
        auto uart = std::make_unique<ungula::hal::uart::Uart>(cfg.uartPort);
        return assembleKit(cfg, cfg.slaveAddress, std::move(uart), nullptr);
}

Result<std::unique_ptr<StepperKit>>
makeStepperKitOnUart(ungula::hal::uart::Uart &uart, uint8_t slaveAddress,
                     const StepperKitConfig &cfg)
{
        return assembleKit(cfg, slaveAddress, nullptr, &uart);
}

} // namespace ungula::motor::tmc2209
