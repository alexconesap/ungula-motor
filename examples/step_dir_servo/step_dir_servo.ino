// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

// UngulaMotor — STEP/DIR servo example.
//
// Drives an industrial STEP/DIR servo (e.g. Yaskawa Sigma-7, Delta
// ASD-A2, Leadshine ELP) over the same pulse interface as a stepper.
// Differences from the open-loop stepper sketch:
//
//   - `createStepDirServo` instead of `createStepDirStepper`. The
//     actuator reports `StepDirActuatorKind::StepDirServo`; capability
//     flags reflect drive-resident position tracking.
//   - Enable polarity is active-HIGH (industrial drive SRV-ON / SVON
//     convention). Most stepper drivers are active-LOW; servos are
//     not.
//   - DIR setup time bumped to 5 µs — servos sample DIR at the step
//     edge and want generous margin. Some drives need 50–100 µs;
//     check the drive's manual.
//   - Optional alarm / in-position inputs surfaced as
//     `hasAlarmInput` / `hasInPositionInput` capabilities. Wiring
//     them into the SensorBank as `CrashLimit`-role inputs gives
//     interrupt-driven fault handling.
//
// Hardware:
//   ESP32 DevKit + industrial STEP/DIR servo drive.
//
// Pin wiring (example):
//   STEP    = GPIO 18  (PUL+ input on drive)
//   DIR     = GPIO 19  (DIR+ input on drive)
//   SVON    = GPIO 21  (SRV-ON input, active-HIGH)
//   ALM     = GPIO 34  (drive's ALM- output, active-LOW)
//   INP     = GPIO 35  (drive's INP / COIN output, active-HIGH)

#include <Arduino.h>

#include <emblogx.h>

// Forwarders help Arduino CLI discover symlinked libraries.
#include <ungula_core.h>
#include <ungula_hal.h>
#include <ungula_motor.h>

#include <ungula/motor.h>

using namespace ungula::motor;

namespace
{

constexpr uint8_t kStepPin = 18;
constexpr uint8_t kDirPin = 19;
constexpr uint8_t kEnablePin = 21; // SVON
constexpr uint8_t kAlmPin = 34;
constexpr uint8_t kInpPin = 35;

std::unique_ptr<Axis> axis;

class SerialListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                Serial.printf("[axis %u] %s  state=%s  pos=%ld  reason=%s  fault=%s\n",
                              ev.axisId.value, axisEventTypeToString(ev.type),
                              axisStateToString(ev.state), static_cast<long>(ev.commandedPosition),
                              stopReasonToString(ev.stopReason), faultToString(ev.faultCode));
        }
};
SerialListener listener;

bool buildAxis()
{
        StepDirServoAxisConfig cfg;
        cfg.common.axisId = AxisId(0);
        cfg.common.name = "servo";
        cfg.common.units.stepsPerMm = 1000.0f; // 10000 pulses/rev, 10 mm/rev lead
        cfg.common.limits.maxVelocitySps = 100000; // industrial servos eat fast pulse trains
        cfg.common.limits.accelSpsPerSec = 500000;
        cfg.common.limits.decelSpsPerSec = 500000;
        cfg.common.limits.minPulseHighUs = 3;
        cfg.common.limits.minPulseLowUs = 3;
        cfg.common.limits.maxStepRateSps = 500000;

        cfg.stepPin = StepPin{ kStepPin };
        cfg.dirPin = DirectionPin{ kDirPin };
        cfg.enablePin = EnablePin{ kEnablePin };
        cfg.alarmInputPin = InputPin{ kAlmPin };
        cfg.inPositionInputPin = InputPin{ kInpPin };
        cfg.dirActiveHigh = true;
        cfg.enableActiveLow = false; // SRV-ON is active-HIGH
        cfg.dirSetupUs = 10; // industrial drives like more margin

        // The alarm input goes into the SensorBank as a CrashLimit role
        // so it gets the interrupt-driven path. The drive's INP output
        // is informational and stays out of the SensorBank for now —
        // hosts that need it poll `feedback().inPosition` via the
        // capability flag.
        cfg.sensors[0].pin = kAlmPin;
        cfg.sensors[0].role = SensorRole::CrashLimit;
        cfg.sensors[0].polarity =
            SensorPolarity::NormallyClosed; // ALM- is open-collector, LOW = fault
        cfg.sensors[0].direction = Direction::Forward; // ALM doesn't care about direction
        cfg.sensorCount = 1;

        auto r = Axis::createStepDirServo(cfg);
        if (!r.ok()) {
                Serial.printf("Servo factory failed: %s\n", errorToString(r.error()));
                return false;
        }
        axis = r.takeValue();
        return true;
}

} // namespace

void setup()
{
        Serial.begin(115200);
        delay(200);
        Serial.println("\nUngulaMotor — STEP/DIR servo example");

        if (!buildAxis()) {
                while (true)
                        delay(1000);
        }

        (void)axis->subscribe(&listener);

        if (!axis->begin().ok()) {
                Serial.println("begin failed");
                while (true)
                        delay(1000);
        }

        // SRV-ON must be high for ≥ 50 ms before the drive accepts pulses.
        if (!axis->enable().ok()) {
                Serial.println("enable failed");
                while (true)
                        delay(1000);
        }
        delay(100);

        Serial.println("Setup complete — motion loop starting.");
}

void loop()
{
        axis->service(millis());

        static enum { Out, Pause, Back } step = Out;
        static uint32_t waitMs = 0;
        if (axis->state() != AxisState::Idle)
                return;

        switch (step) {
        case Out:
                (void)axis->moveBy(10000); // 10 mm at 1000 steps/mm
                step = Pause;
                waitMs = millis() + 500;
                break;
        case Pause:
                if (millis() >= waitMs)
                        step = Back;
                break;
        case Back:
                (void)axis->moveBy(-10000);
                step = Out;
                waitMs = 0;
                break;
        }
}
