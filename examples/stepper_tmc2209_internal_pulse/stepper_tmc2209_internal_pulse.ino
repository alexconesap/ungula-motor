// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

// UngulaMotor — canonical TMC2209 stepper example.
//
// Wires a single open-loop stepper axis behind the `Axis` facade with
// autonomous internal pulse mode (a hardware timer + ISR generates
// every step pulse). The application loop only services the axis —
// it never touches the step pin and never blocks pulse timing.
//
// What this sketch demonstrates:
//   - Building an axis via `Axis::createStepDirStepper`.
//   - Configuring a TMC2209 driver via UART (`Tmc2209Configurator`).
//   - Issuing forward/backward moves with `moveBy`.
//   - Driving the supervisory `service()` tick from `loop()`.
//   - Subscribing a listener to receive motion events from task
//     context (never ISR).
//
// Hardware:
//   ESP32 DevKit + BTT TMC2209 stepper driver + NEMA17 motor.
//
// Pin wiring (adjust to your board):
//   STEP        = GPIO 18
//   DIR         = GPIO 19
//   EN          = GPIO 21  (active-LOW)
//   TMC UART TX = GPIO 17
//   TMC UART RX = GPIO 16
//
// The TMC2209's PDN_UART pin must be wired so single-wire UART works
// (typical: tied through a 1k resistor to the MCU TX line).

#include <Arduino.h>

#include <emblogx.h>

// Forwarders help Arduino CLI discover symlinked libraries.
#include <ungula_core.h>
#include <ungula_hal.h>
#include <ungula_motor.h>

#include <ungula/motor.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>
#include <ungula/hal/uart/uart.h>

using namespace ungula::motor;

namespace
{

constexpr uint8_t kStepPin = 18;
constexpr uint8_t kDirPin = 19;
constexpr uint8_t kEnablePin = 21;
constexpr uint8_t kTmcUartPort = 1;
constexpr uint8_t kTmcTxPin = 17;
constexpr uint8_t kTmcRxPin = 16;
constexpr uint32_t kTmcBaud = 115200;
constexpr uint8_t kTmcSlaveAddr = 0;

ungula::hal::uart::Uart tmcUart(kTmcUartPort);
tmc2209::Tmc2209HalUart tmcTransport(tmcUart, kTmcSlaveAddr);
tmc2209::Tmc2209Configurator tmcConfig(tmcTransport);

std::unique_ptr<Axis> axis;

// Simple listener — prints every event to Serial. In production this
// is where you'd hook the UI / state machine / safety supervisor.
class SerialListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                Serial.printf("[axis %u] %s  state=%s  pos=%ld  reason=%s\n", ev.axisId.value,
                              axisEventTypeToString(ev.type), axisStateToString(ev.state),
                              static_cast<long>(ev.commandedPosition),
                              stopReasonToString(ev.stopReason));
        }
};
SerialListener listener;

bool buildAxis()
{
        StepDirStepperAxisConfig cfg;
        cfg.common.axisId = AxisId(0);
        cfg.common.name = "x";
        cfg.common.units.stepsPerMm = 80.0f; // 1.8° motor, 16x microstepping, GT2 20T pulley
        cfg.common.limits.maxVelocitySps = 8000;
        cfg.common.limits.accelSpsPerSec = 20000;
        cfg.common.limits.decelSpsPerSec = 20000;
        cfg.common.limits.minPulseHighUs = 2;
        cfg.common.limits.minPulseLowUs = 2;
        cfg.common.limits.maxStepRateSps = 200000;

        cfg.stepPin = StepPin{ kStepPin };
        cfg.dirPin = DirectionPin{ kDirPin };
        cfg.enablePin = EnablePin{ kEnablePin };
        cfg.dirActiveHigh = true;
        cfg.enableActiveLow = true;
        cfg.dirSetupUs = 5;

        auto r = Axis::createStepDirStepper(cfg);
        if (!r.ok()) {
                Serial.printf("Axis factory failed: %s\n", errorToString(r.error()));
                return false;
        }
        axis = r.takeValue();
        return true;
}

bool configureDriver()
{
        if (!tmcUart.begin(kTmcBaud, kTmcTxPin, kTmcRxPin)) {
                Serial.println("TMC UART begin failed");
                return false;
        }
        tmc2209::Tmc2209Configurator::Config c;
        c.runCurrentMa = 700;
        c.holdCurrentMa = 300;
        c.microsteps = tmc2209::Microsteps::Sixteenth;
        c.mode = tmc2209::ChopperMode::StealthChop;

        const auto s = tmcConfig.begin(c);
        if (!s.ok()) {
                Serial.printf("TMC begin failed: %s\n", errorToString(s.error()));
                return false;
        }
        return true;
}

} // namespace

void setup()
{
        Serial.begin(115200);
        delay(200);
        Serial.println("\nUngulaMotor — TMC2209 internal pulse example");

        if (!buildAxis()) {
                while (true)
                        delay(1000);
        }
        if (!configureDriver()) {
                while (true)
                        delay(1000);
        }

        auto sub = axis->subscribe(&listener);
        if (!sub.ok())
                Serial.println("subscribe failed");

        if (!axis->begin().ok()) {
                Serial.println("axis begin failed");
                while (true)
                        delay(1000);
        }
        if (!axis->enable().ok()) {
                Serial.println("axis enable failed");
                while (true)
                        delay(1000);
        }

        Serial.println("Setup complete — entering motion loop.");
}

void loop()
{
        static enum { Forward, Wait1, Backward, Wait2 } step = Forward;
        static uint32_t waitUntilMs = 0;

        axis->service(millis());

        if (!axis->isHoming() && axis->state() == AxisState::Idle) {
                switch (step) {
                case Forward:
                        (void)axis->moveBy(3200); // 1 revolution at 16x microsteps
                        step = Wait1;
                        waitUntilMs = millis() + 500;
                        break;
                case Wait1:
                        if (millis() >= waitUntilMs)
                                step = Backward;
                        break;
                case Backward:
                        (void)axis->moveBy(-3200);
                        step = Wait2;
                        waitUntilMs = millis() + 500;
                        break;
                case Wait2:
                        if (millis() >= waitUntilMs)
                                step = Forward;
                        break;
                }
        }
}
