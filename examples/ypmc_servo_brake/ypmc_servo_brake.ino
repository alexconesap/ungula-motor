// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

// UngulaMotor — RATTMOTOR YPMC-400W servo + S2SVD15 drive + brake.
//
// The S2SVD15 is a STEP/DIR industrial servo controller; the host
// supplies the pulse train and the drive runs its own position loop.
// This sketch shows:
//
//   - applyDriveDefaults() pre-populating the StepDirServoAxisConfig
//     with the S2SVD15's documented timing / polarity numbers.
//   - ALM− wired as a CrashLimit sensor for ISR-driven fault halts.
//   - BrakeController sequenced explicitly with the axis lifecycle:
//     release before motion, engage after motion stops or faults.
//
// Hardware:
//   ESP32 DevKit + S2SVD15 drive + YPMC-400W motor (brake option) +
//   24 V supply for the brake coil + opto-MOSFET / SSR between the
//   GPIO and the coil.
//
// Pin wiring (example):
//   STEP    = GPIO 18  -> PUL+
//   DIR     = GPIO 19  -> DIR+
//   SRV-ON  = GPIO 21  -> SRV-ON (active-HIGH)
//   ALM     = GPIO 34  <- ALM− (active-LOW, open-collector + pull-up)
//   COIN    = GPIO 35  <- COIN (active-HIGH; informational)
//   BRAKE   = GPIO 25  -> opto-MOSFET gate, drives 24 V brake coil

#include <Arduino.h>

#include <ungula_core.h>
#include <ungula_hal.h>
#include <ungula_motor.h>

#include <ungula/motor.h>
#include <ungula/motor/drivers/ypmc/ypmc_servo.h>

using namespace ungula::motor;
namespace ypmc = ungula::motor::ypmc;

namespace
{

constexpr uint8_t kStepPin   = 18;
constexpr uint8_t kDirPin    = 19;
constexpr uint8_t kSrvOnPin  = 21;
constexpr uint8_t kAlmPin    = 34;
constexpr uint8_t kCoinPin   = 35;
constexpr uint8_t kBrakePin  = 25;

std::unique_ptr<Axis> axis;

ypmc::BrakeController brake({
    /*.brakeReleasePin       =*/ kBrakePin,
    /*.brakeReleaseActiveHigh=*/ true,
    /*.releaseSettleMs       =*/ 120,
    /*.engageSettleMs        =*/ 30,
    /*.autoEngageOnMotionEnd =*/ true,
});

class SerialListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                Serial.printf("[axis %u] %s  state=%s  pos=%ld  reason=%s  fault=%s\n",
                              ev.axisId.value, axisEventTypeToString(ev.type),
                              axisStateToString(ev.state),
                              static_cast<long>(ev.commandedPosition),
                              stopReasonToString(ev.stopReason),
                              faultToString(ev.faultCode));
        }
};
SerialListener listener;

bool buildAxis()
{
        StepDirServoAxisConfig cfg;
        cfg.common.axisId = AxisId(0);
        cfg.common.name   = "ypmc";

        // YPMC-400W with the S2SVD15 default electronic gear: 10000
        // pulses per revolution. Assume a 10 mm/rev lead screw:
        //   10000 pulses / 10 mm = 1000 steps per mm.
        cfg.common.units.stepsPerMm     = 1000.0f;
        cfg.common.units.stepsPerDegree = 10000.0f / 360.0f;

        // 3000 rpm rated speed × 10000 pulses/rev = 500 kpps. That's
        // the drive's CMD+DIR ceiling — applyDriveDefaults() clamps
        // maxStepRateSps to it.
        cfg.common.limits.maxVelocitySps = 200'000;   // 1200 rpm; comfortable working speed
        cfg.common.limits.accelSpsPerSec = 800'000;
        cfg.common.limits.decelSpsPerSec = 800'000;

        cfg.stepPin            = StepPin{ kStepPin };
        cfg.dirPin             = DirectionPin{ kDirPin };
        cfg.enablePin          = EnablePin{ kSrvOnPin };
        cfg.alarmInputPin      = InputPin{ kAlmPin };
        cfg.inPositionInputPin = InputPin{ kCoinPin };

        // applyDriveDefaults sets dir setup, pulse widths, max rate,
        // SRV-ON polarity, and wires ALM as a CrashLimit sensor.
        const auto def = ypmc::applyDriveDefaults(cfg);
        if (!def.ok()) {
                Serial.printf("YPMC defaults failed: %s\n", errorToString(def.error()));
                return false;
        }

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
        Serial.println("\nUngulaMotor — YPMC servo + brake example");

        if (!buildAxis()) {
                while (true) delay(1000);
        }

        if (!brake.begin().ok()) {
                Serial.println("Brake controller begin failed");
                while (true) delay(1000);
        }

        (void)axis->subscribe(&listener);
        (void)axis->subscribe(&brake);  // auto-engages on motion-end / fault

        if (!axis->begin().ok()) {
                Serial.println("Axis begin failed");
                while (true) delay(1000);
        }

        if (!axis->enable().ok()) {
                Serial.println("Axis enable failed");
                while (true) delay(1000);
        }
        delay(60);  // SRV-ON debounce inside the drive

        // Release brake BEFORE the first motion. The call blocks for
        // releaseSettleMs (~120 ms) so the brake fully releases before
        // a STEP pulse arrives.
        if (!brake.release().ok()) {
                Serial.println("Brake release failed");
                while (true) delay(1000);
        }

        Serial.println("Setup complete — motion loop starting.");
}

void loop()
{
        axis->service(millis());

        if (axis->state() != AxisState::Idle)
                return;

        // Brake engages automatically on MotionStopped / MotionCompleted
        // via the subscribed listener. We re-release for the next move.
        if (!brake.isReleased()) {
                (void)brake.release();
        }

        static bool outward = true;
        const Distance delta = outward ? 10'000 : -10'000;
        (void)axis->moveBy(delta);
        outward = !outward;

        delay(500);  // pause between moves
}
