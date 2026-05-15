// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

// UngulaMotor — limit-switch homing example.
//
// Builds on the canonical stepper sketch and adds a home-reference
// switch wired to GPIO 34. The axis runs through the
// `LimitSwitchHomingStrategy` at boot:
//
//   FastApproach (backward at 2000 sps)
//   → Backoff      (forward, 200 steps at 200 sps)
//   → SlowApproach (backward at 200 sps)
//   → SetHomePosition (commanded position = 0)
//   → Complete
//
// What this sketch demonstrates:
//   - Wiring a `SensorRole::Home` switch into the axis config.
//   - Registering a `LimitSwitchHomingStrategy` with a timeout.
//   - Running `axis.home()` and waiting for the controller to settle.
//   - Issuing a normal motion AFTER homing succeeds.
//
// Hardware:
//   ESP32 DevKit + TMC2209 + NEMA17.
//   Home switch: normally-closed micro-switch wired between GPIO 34
//   and GND. (NC means a cut wire reads as "pressed", which is the
//   fail-safe convention.)
//
// Pin wiring:
//   STEP    = GPIO 18
//   DIR     = GPIO 19
//   EN      = GPIO 21
//   HOME    = GPIO 34   (input only; external 10k pull-up if not NC)

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
constexpr uint8_t kEnablePin = 21;
constexpr uint8_t kHomePin = 34;

std::unique_ptr<Axis> axis;

LimitSwitchHomingStrategy::Config makeHomingCfg()
{
        LimitSwitchHomingStrategy::Config c;
        c.approachDirection = Direction::Backward;
        c.fastFeedSps = 2000;
        c.slowFeedSps = 200;
        c.backoffSteps = 200;
        c.homePositionSteps = 0;
        return c;
}
LimitSwitchHomingStrategy strategy(makeHomingCfg());

class SerialListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                Serial.printf("[axis %u] %s  state=%s  pos=%ld\n", ev.axisId.value,
                              axisEventTypeToString(ev.type), axisStateToString(ev.state),
                              static_cast<long>(ev.commandedPosition));
        }
};
SerialListener listener;

bool buildAxis()
{
        StepDirStepperAxisConfig cfg;
        cfg.common.axisId = AxisId(0);
        cfg.common.units.stepsPerMm = 80.0f;
        cfg.common.limits.maxVelocitySps = 8000;
        cfg.common.limits.accelSpsPerSec = 20000;
        cfg.common.limits.decelSpsPerSec = 20000;
        cfg.common.limits.minPulseHighUs = 2;
        cfg.common.limits.minPulseLowUs = 2;
        cfg.common.limits.maxStepRateSps = 200000;

        cfg.stepPin = StepPin{ kStepPin };
        cfg.dirPin = DirectionPin{ kDirPin };
        cfg.enablePin = EnablePin{ kEnablePin };

        // One home sensor; the rest of the slots stay unused.
        cfg.sensors[0].pin = kHomePin;
        cfg.sensors[0].role = SensorRole::Home;
        cfg.sensors[0].polarity = SensorPolarity::NormallyClosed;
        cfg.sensors[0].direction = Direction::Backward; // home is in -Z direction
        cfg.sensors[0].debounceMs = 20;
        cfg.sensorCount = 1;

        auto r = Axis::createStepDirStepper(cfg);
        if (!r.ok()) {
                Serial.printf("Axis factory failed: %s\n", errorToString(r.error()));
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
        Serial.println("\nUngulaMotor — limit-switch homing example");

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
        if (!axis->enable().ok()) {
                Serial.println("enable failed");
                while (true)
                        delay(1000);
        }

        // Wire the strategy. 30 s timeout — far more than any sensible
        // setup needs, short enough that a stuck mechanism doesn't run
        // forever.
        (void)axis->setHomingStrategy(&strategy, /*timeoutMs=*/30000);

        Serial.println("Starting homing cycle...");
        const auto s = axis->home();
        if (!s.ok()) {
                Serial.printf("home() rejected: %s\n", errorToString(s.error()));
                while (true)
                        delay(1000);
        }
}

void loop()
{
        axis->service(millis());

        if (axis->isHoming())
                return;

        static bool didFirstMove = false;
        if (!didFirstMove && axis->isHomed()) {
                didFirstMove = true;
                Serial.println("Homed. Moving to absolute position 1600 steps.");
                (void)axis->moveTo(1600);
        }
}
