# UngulaMotor

> **Embedded C++20 motion control for ESP32 and STM32 (via lib_hal).** Open-loop steppers, STEP/DIR servos, and CAN servos behind a single `Axis` facade.

The library is built around one rule:

> **Slow application code, slow UART diagnostics, slow event listeners, and slow CAN traffic must never disturb step-pulse timing.**

Pulse generation lives inside a hardware-timer ISR. Everything else — the main loop, UART register reads to a TMC2209, event listeners, CAN traffic, logging — runs in task context and observes the engine, never commands its timing.

## Table of contents

- [Features](#features)
- [Supported drives](#supported-drives)
- [Dependencies](#dependencies)
- [Architecture](#architecture)
- [Quick start — open-loop stepper (TMC2209)](#quick-start--open-loop-stepper-tmc2209)
- [Quick start — STEP/DIR servo](#quick-start--stepdir-servo)
- [Quick start — RATTMOTOR YPMC servo + brake](#quick-start--rattmotor-ypmc-servo--brake)
- [Quick start — limit-switch homing](#quick-start--limit-switch-homing)
- [Speed and acceleration in user units](#speed-and-acceleration-in-user-units)
- [Live trajectory tuning](#live-trajectory-tuning)
- [Idle policy and the `AutoDisableOnIdle` listener](#idle-policy-and-the-autodisableonidle-listener)
- [Stall-based homing](#stall-based-homing)
- [The service loop](#the-service-loop)
- [Blocking move wait (20 cm)](#blocking-move-wait-20-cm)
- [Events](#events)
- [Sensors, limits, and safety inputs](#sensors-limits-and-safety-inputs)
- [TMC2209 driver split](#tmc2209-driver-split)
- [Driver authoring guide](#driver-authoring-guide)
- [Faults, stop modes, and recovery](#faults-stop-modes-and-recovery)
- [Composing axes by hand](#composing-axes-by-hand)
- [Testing](#testing)
- [License](#license)

## Features

- **One `Axis` facade for three drive families** — open-loop steppers, STEP/DIR servos, CAN servos. The wire protocol differs; the application code does not.
- **ISR-driven pulse generation.** Steps are emitted from a hardware-timer ISR off precomputed `MotionSegment` queues. The main loop can block for hundreds of milliseconds without dropping a step.
- **Integer-only motion planner.** Trapezoidal + triangular profiles, jog, soft-stop ramps. Step count is preserved exactly — no silent drift from rounding.
- **Typed user units.** `Speed` and `Acceleration` carry a value AND a unit (`mm/s`, `cm/min`, `rpm`, ...). Named factories (`Speed::mmPerSec(150.0f)`) keep call sites unambiguous; resolution to steps/sec happens against the axis's `UnitScaling`.
- **Live trajectory tuning.** `setMaxVelocity(Speed)` while an indefinite jog is running stops and re-arms at the new feed. Accel / decel changes apply on the next motion.
- **Typed results everywhere.** `Result<T>` / `Status` instead of `void`/silent failures. No exceptions, no RTTI.
- **Heap allocation only at boot.** Factories return `Result<std::unique_ptr<Axis>>`; steady-state runtime is allocation-free.
- **Sensor-aware limits.** `Home` (polled, debounced), `TravelLimit` (polled), `CrashLimit` and `EmergencyStop` (GPIO interrupt → sub-µs halt of the pulse engine), and `Stall` (ISR with hit-count + arm-delay debounce for TMC2209 DIAG).
- **Explicit homing FSM.** `Idle → FastApproach → Backoff → SlowApproach → SetHomePosition → Complete/Failed`. Two built-in strategies: `LimitSwitchHomingStrategy` (wired switch) and `StallHomingStrategy` (sensorless, against a mechanical hard-stop).
- **Selectable idle policy.** `Axis::setDefaultIdlePolicy(IdlePolicy::HoldCurrent | AutoDisable)` runs as a built-in listener — keep holding torque or freewheel automatically at every motion end. Custom `IAxisEventListener` instances coexist for richer behaviours.
- **TMC2209 split into focused components.** `Tmc2209Configurator` (mA-based current API, boot config, no reads), `Tmc2209Diagnostics` (opt-in register reads, off motion path), `Tmc2209StallGuard` (DIAG-pin first, refuses to configure in SpreadCycle), `Tmc2209CoolStep` (independent load-based current scaling). UART traffic is never on the motion timing path.
- **Bounded event queue.** ISR-safe enqueue, task-context drain. Listeners are dispatched from `service()` — never from ISR.

## Supported drives

| Drive family | Wire protocol | Driver class | Notes |
| --- | --- | --- | --- |
| Open-loop stepper (TMC2209, A4988, DRV8825) | STEP + DIR + EN (active-LOW) | `StepDirActuator` | Optional UART configuration via `Tmc2209Configurator` |
| STEP/DIR servo (Yaskawa, Delta, Leadshine) | STEP + DIR + SVON (typically active-HIGH) + optional ALM / INP | `StepDirActuator` (kind = `StepDirServo`) | Drive owns position truth; encoder feedback wiring deferred |
| RATTMOTOR YPMC + S2SVD15 servo (with optional brake) | STEP + DIR + SRV-ON (active-HIGH) + ALM / COIN + 24 V brake coil | `StepDirActuator` (kind = `StepDirServo`) + `ungula::motor::ypmc::applyDriveDefaults` + `ypmc::BrakeController` | Helper fills S2SVD15 timing / polarity; brake controller sequences a host-driven 24 V holding brake against the axis lifecycle |
| CAN servo (CiA-402, vendor-specific) | CAN bus | `CanServoActuator` + `ICanServoProtocol` | Concrete protocols not bundled — bring your own |

## Dependencies

- **UngulaCore** — `ungula::core::time::millis() / delayUs()`.
- **UngulaHal** — `gpio` (incl. ISR install), `timer::IHwTimer` + `HwTimer` driver, `uart::Uart` for TMC UART, `can::Can` for CAN servos, `sync::CriticalSection` for the event queue.

No logging dependency inside the motion path. Hosts wire events to their own logger or callback.

### Include reference

The umbrella header pulls in everything you typically need:

```cpp
#include <ungula/motor.h>
```

If you want to keep build dependencies tight, the focused headers are:

```cpp
#include <ungula/motor/axis.h>                          // the facade
#include <ungula/motor/axis_config.h>                   // config structs
#include <ungula/motor/events/axis_event.h>             // AxisEvent, listener interface
#include <ungula/motor/homing/limit_switch_homing_strategy.h>  // homing strategy

// TMC2209 (only when you want UART configuration):
#include <ungula/hal/uart/uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_stallguard.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_coolstep.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>

// Sensorless homing against a mechanical stop:
#include <ungula/motor/homing/stall_homing_strategy.h>
```

## Architecture

```text
ungula::motor
├── Axis                              -- user-facing facade (owns the three below)
│    ├── std::unique_ptr<IHwTimer>    -- from lib_hal
│    ├── std::unique_ptr<IPulseEngine>
│    │    ├── HalPulseEngine          -- production; timer-ISR-driven
│    │    └── FakePulseEngine         -- header-only test fake
│    └── std::unique_ptr<IAxisActuator>
│         ├── StepDirActuator         -- open-loop + STEP/DIR servo
│         └── CanServoActuator        -- CAN drives (skeleton; needs ICanServoProtocol)
│
├── MotionPlanner                     -- stateless; integer trapezoidal/triangular
├── SensorBank                        -- polled + ISR sensor service
├── HomingController + IHomingStrategy
│    ├── LimitSwitchHomingStrategy    -- wired home/limit switch
│    └── StallHomingStrategy          -- sensorless; uses StallGuard
├── AxisEventQueue                    -- bounded; ISR-safe enqueue, task drain
│
└── drivers/tmc2209/
     ├── ITmcUart                     -- transport abstraction
     │    ├── Tmc2209HalUart          -- lib_hal Uart adapter (CRC, framing)
     │    └── FakeTmcUart             -- test fake (register file in memory)
     ├── Tmc2209Configurator          -- boot config (mA-based currents); no reads
     ├── Tmc2209Diagnostics           -- opt-in reads, off motion path
     ├── Tmc2209StallGuard            -- DIAG-pin + optional SG_RESULT polling
     └── Tmc2209CoolStep              -- load-based dynamic current scaling
```

`Axis` is the only type most applications need. The factories validate the config, allocate the underlying components, and return a fully wired axis ready for `begin()`.

For a per-class deep dive — who owns what, ISR vs. task boundaries,
how a `moveBy()` call turns into STEP edges, how DIAG edges become
`FaultCode::Stall` — see **[ARCHITECTURE.md](ARCHITECTURE.md)**.
For driver authoring (adding a new chip / drive integration under
`drivers/<vendor_model>/`) see the
[driver authoring guide](src/ungula/motor/drivers/README.md).

## Quick start — TMC2209 kit (recommended)

The kit factory bundles every TMC2209 helper plus the underlying axis. One call, one returned `unique_ptr`. Use this for new code.

```cpp
#include <Arduino.h>
#include <ungula/motor.h>

using namespace ungula::motor;

std::unique_ptr<tmc2209::StepperKit> motor;

void setup() {
    tmc2209::StepperKitConfig cfg;
    cfg.common.axisId            = AxisId(0);
    cfg.common.units.stepsPerMm  = 80.0f;
    cfg.common.limits.maxVelocitySps = 8000;
    cfg.common.limits.accelSpsPerSec = 20000;
    cfg.common.limits.decelSpsPerSec = 20000;
    cfg.common.limits.maxStepRateSps = 200000;

    cfg.stepPin   = StepPin{18};
    cfg.dirPin    = DirectionPin{19};
    cfg.enablePin = EnablePin{21};

    cfg.uartPort     = 1;          // ESP32 UART1
    cfg.uartBaud     = 115200;
    cfg.uartTxPin    = 17;
    cfg.uartRxPin    = 16;
    cfg.slaveAddress = 0;

    cfg.chip.runCurrentMa      = 800;
    cfg.chip.holdCurrentMa     = 300;
    cfg.chip.senseResistorOhms = 0.11f;

    // Optional: enable StallGuard if you wired the DIAG pin to a GPIO
    // and added a `SensorRole::Stall` entry to `cfg.sensors`.
    cfg.useStallGuard       = true;
    cfg.stall.sgThreshold   = 20;
    cfg.stall.tCoolThrs     = 0xFFFFF;

    auto r = tmc2209::makeStepperKit(cfg);
    if (!r.ok()) { Serial.println("kit failed"); return; }
    motor = r.takeValue();

    motor->begin();             // brings up UART + chip + axis
    motor->axis->enable();
}

void loop() {
    motor->axis->service(millis());
    if (motor->axis->state() == AxisState::Idle) {
        motor->axis->moveBy(3200);
    }
}
```

### Two TMC2209s on one UART

For N-motor projects, share a single UART and give each kit its own NAI[1:0] slave address:

```cpp
ungula::hal::uart::Uart bus(2);                  // host owns the UART
std::unique_ptr<tmc2209::StepperKit> motorA;
std::unique_ptr<tmc2209::StepperKit> motorB;

void setup() {
    bus.begin(115200, /*tx=*/17, /*rx=*/16);

    motorA = tmc2209::makeStepperKitOnUart(bus, /*addr=*/0, cfgA).takeValue();
    motorB = tmc2209::makeStepperKitOnUart(bus, /*addr=*/1, cfgB).takeValue();

    motorA->begin();
    motorB->begin();
}
```

The kits do not own the UART — the host keeps it alive.

## Quick start — TMC2209 by hand (advanced)

The original compose-by-hand path stays available for non-standard wiring (custom pulse engine, custom UART buffer sizes, etc.).

```cpp
#include <Arduino.h>

#include <ungula/motor.h>
#include <ungula/hal/uart/uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>

using namespace ungula::motor;

ungula::hal::uart::Uart       tmcUart(1);
tmc2209::Tmc2209HalUart       tmcTransport(tmcUart, /*slaveAddress=*/0);
tmc2209::Tmc2209Configurator  tmcConfig(tmcTransport);
std::unique_ptr<Axis>         axis;

void setup() {
    Serial.begin(115200);

    // 1) UART (host owns this — the TMC driver only borrows the bus).
    tmcUart.begin(/*baud=*/115200, /*tx=*/17, /*rx=*/16);

    // 2) TMC2209 boot config — currents, microsteps, chopper, GSTAT clear.
    //    Currents are in mA RMS. The configurator converts to the chip's
    //    5-bit IRUN/IHOLD fields using the sense resistor on your board
    //    (check the silk screen — usually 0.11 Ω for BTT modules,
    //    0.075 Ω for Watterott SilentStepSticks).
    tmc2209::Tmc2209Configurator::Config tc;
    tc.runCurrentMa        = 800;     // mA RMS at the coil
    tc.holdCurrentMa       = 300;     // ~30..50% of runCurrentMa is typical
    tc.iHoldDelay          = 1;       // ~21 ms run → hold transition
    tc.senseResistorOhms   = 0.11f;
    tc.useHighSensitivity  = false;   // vsense=1 only for low-current setups (<300 mA)
    tc.microsteps          = tmc2209::Microsteps::Sixteenth;
    tc.mode                = tmc2209::ChopperMode::StealthChop;
    tmcConfig.begin(tc);          // returns Status; check it in production code

    // 3) The axis.
    StepDirStepperAxisConfig cfg;
    cfg.common.axisId            = AxisId(0);
    cfg.common.units.stepsPerMm  = 80.0f;
    cfg.common.limits.maxVelocitySps = 8000;
    cfg.common.limits.accelSpsPerSec = 20000;
    cfg.common.limits.decelSpsPerSec = 20000;
    cfg.common.limits.maxStepRateSps = 200000;
    cfg.stepPin   = StepPin{18};
    cfg.dirPin    = DirectionPin{19};
    cfg.enablePin = EnablePin{21};       // active-LOW for TMC2209

    auto r = Axis::createStepDirStepper(cfg);
    if (!r.ok()) { Serial.println("axis factory failed"); return; }
    axis = r.takeValue();

    axis->begin();
    axis->enable();
}

void loop() {
    axis->service(millis());

    if (axis->state() == AxisState::Idle) {
        axis->moveBy(3200);              // 1 revolution at 16x microsteps
    }
}
```

The pulse engine begins emitting steps the moment `moveBy` arms it. The main loop only calls `service(millis())` — pulse timing is entirely inside the timer ISR.

The full version of this sketch is in [`examples/stepper_tmc2209_internal_pulse/`](examples/stepper_tmc2209_internal_pulse/).

## Quick start — STEP/DIR servo

Same wire protocol, different defaults: active-HIGH SVON, optional ALM and INP digital inputs.

```cpp
#include <Arduino.h>

#include <ungula/motor.h>

using namespace ungula::motor;

std::unique_ptr<Axis> axis;

void setup() {
    StepDirServoAxisConfig cfg;
    cfg.common.axisId                = AxisId(0);
    cfg.common.units.stepsPerMm      = 1000.0f;     // 10000 pulses/rev, 10 mm lead
    cfg.common.limits.maxVelocitySps = 100000;
    cfg.common.limits.accelSpsPerSec = 500000;
    cfg.common.limits.decelSpsPerSec = 500000;
    cfg.common.limits.maxStepRateSps = 500000;

    cfg.stepPin            = StepPin{18};
    cfg.dirPin             = DirectionPin{19};
    cfg.enablePin          = EnablePin{21};         // SVON
    cfg.alarmInputPin      = InputPin{34};          // optional
    cfg.inPositionInputPin = InputPin{35};          // optional
    cfg.dirActiveHigh   = true;
    cfg.enableActiveLow = false;                    // servos use active-HIGH
    cfg.dirSetupUs      = 10;                       // industrial drives want margin

    // Wire ALM as a CrashLimit sensor so a drive fault halts the
    // pulse engine from the GPIO ISR — sub-µs latency.
    cfg.sensors[0].pin       = 34;
    cfg.sensors[0].role      = SensorRole::CrashLimit;
    cfg.sensors[0].polarity  = SensorPolarity::NormallyClosed;
    cfg.sensors[0].direction = Direction::Forward;
    cfg.sensorCount          = 1;

    auto r = Axis::createStepDirServo(cfg);
    axis  = r.takeValue();

    axis->begin();
    axis->enable();
    delay(100);                                     // SRV-ON settle time
}

void loop() {
    axis->service(millis());
    if (axis->state() == AxisState::Idle) {
        axis->moveBy(10000);                        // 10 mm at 1000 steps/mm
    }
}
```

Full sketch in [`examples/step_dir_servo/`](examples/step_dir_servo/).

## Quick start — RATTMOTOR YPMC servo + brake

The YPMC + S2SVD15 kit is a STEP/DIR industrial servo with an
optional 24 V holding brake on the motor. The wire shape is the same
as the generic STEP/DIR servo above — `ungula::motor::ypmc` adds two
focused helpers on top:

- `applyDriveDefaults(StepDirServoAxisConfig&)` — pre-populates the
  S2SVD15's timing (DIR setup, min pulse widths, 500 kpps max),
  polarity (SRV-ON active-HIGH, ALM− active-LOW), and wires ALM as a
  `CrashLimit` sensor when its pin is set.
- `BrakeController` — sequences a host-driven brake relay against the
  axis lifecycle. Auto-engages on `MotionCompleted` / `MotionStopped`
  / faults; release is explicit (blocks for the brake's mechanical
  release time before the first STEP can fire).

```cpp
#include <ungula/motor.h>
#include <ungula/motor/drivers/ypmc/ypmc_servo.h>

using namespace ungula::motor;
namespace ypmc = ungula::motor::ypmc;

StepDirServoAxisConfig cfg;
cfg.common.axisId = AxisId(0);
cfg.common.units.stepsPerMm = 1000.0f;            // 10000 pulses/rev, 10 mm lead
cfg.common.limits.maxVelocitySps = 200'000;
cfg.common.limits.accelSpsPerSec = 800'000;
cfg.common.limits.decelSpsPerSec = 800'000;
cfg.stepPin            = StepPin{ 18 };
cfg.dirPin             = DirectionPin{ 19 };
cfg.enablePin          = EnablePin{ 21 };          // SRV-ON
cfg.alarmInputPin      = InputPin{ 34 };           // ALM−
cfg.inPositionInputPin = InputPin{ 35 };           // COIN
ypmc::applyDriveDefaults(cfg);                     // fills timing + polarity + alarm sensor

auto axis = Axis::createStepDirServo(cfg).takeValue();

ypmc::BrakeController brake({
    /*.brakeReleasePin       =*/ 25,
    /*.brakeReleaseActiveHigh=*/ true,
    /*.releaseSettleMs       =*/ 120,
    /*.engageSettleMs        =*/ 30,
    /*.autoEngageOnMotionEnd =*/ true,
});

void setup() {
    brake.begin();
    axis->subscribe(&brake);     // auto-engages on motion-end / fault
    axis->begin();
    axis->enable();
    delay(60);                   // SRV-ON debounce inside the drive
    brake.release();             // blocks ~120 ms for mechanical release
    axis->moveBy(10000);
}
```

The brake controller does NOT auto-release on `MotionStarted` — by
the time that event fires the engine has already armed. Hosts call
`release()` explicitly between `enable()` and the first motion
command. See [`examples/ypmc_servo_brake/`](examples/ypmc_servo_brake/)
for the full sketch and
[`src/ungula/motor/drivers/ypmc/README.md`](src/ungula/motor/drivers/ypmc/README.md)
for S2SVD15 wiring and tuning notes.

### YPMC kit shortcut

`makeServoKit(cfg)` does the same work as the by-hand sketch above:
builds the axis via `Axis::createStepDirServo`, applies the S2SVD15
defaults, optionally constructs a `BrakeController` and subscribes it
to the event bus.

```cpp
ypmc::ServoKitConfig cfg;
cfg.common.axisId = AxisId(0);
cfg.common.units.stepsPerMm = 1000.0f;
cfg.common.limits.maxVelocitySps = 200'000;
cfg.common.limits.accelSpsPerSec = 800'000;
cfg.common.limits.decelSpsPerSec = 800'000;
cfg.stepPin            = StepPin{ 18 };
cfg.dirPin             = DirectionPin{ 19 };
cfg.enablePin          = EnablePin{ 21 };           // SRV-ON
cfg.alarmInputPin      = InputPin{ 34 };
cfg.inPositionInputPin = InputPin{ 35 };
cfg.useBrake           = true;
cfg.brake.brakeReleasePin = 25;

auto kit = ypmc::makeServoKit(cfg).takeValue();
kit->begin();              // brake.begin() + axis->begin() + subscribe
kit->axis->enable();
delay(60);
kit->brake->release();     // before the first move
kit->axis->moveBy(10000);
```

### Tandem YPMCs on one STEP pin

For two YPMCs sharing one STEP signal (e.g. mounted face-to-face on
the same shaft), set the `secondary*` pin fields:

```cpp
cfg.secondaryDirPin       = DirectionPin{ 22 };
cfg.secondaryEnablePin    = EnablePin{ 23 };
cfg.secondaryDirInverted  = true;   // face-to-face → opposite electrical dir
```

The pulse engine writes both DIRs atomically inside the same
`dirSetupUs` window. `enable()` / `disable()` flip both SRV-ON pins.
`secondaryDirInverted = true` produces the same physical rotation
from both motors despite opposite mounting.

## Quick start — limit-switch homing

Add a home reference switch on GPIO 34 and a homing strategy. `Axis::home()` runs the 7-state FSM until the strategy reports Complete or Failed.

```cpp
#include <Arduino.h>

#include <ungula/motor.h>
#include <ungula/motor/homing/limit_switch_homing_strategy.h>

using namespace ungula::motor;

std::unique_ptr<Axis> axis;

LimitSwitchHomingStrategy::Config makeHomingCfg() {
    LimitSwitchHomingStrategy::Config c;
    c.approachDirection = Direction::Backward;     // home is in -direction
    c.fastFeedSps       = 2000;
    c.slowFeedSps       = 200;                     // ×10 slower for repeatability
    c.backoffSteps      = 200;
    c.homePositionSteps = 0;
    return c;
}
LimitSwitchHomingStrategy strategy(makeHomingCfg());

void setup() {
    StepDirStepperAxisConfig cfg;
    cfg.common.axisId            = AxisId(0);
    cfg.common.units.stepsPerMm  = 80.0f;
    cfg.common.limits.maxVelocitySps = 8000;
    cfg.common.limits.accelSpsPerSec = 20000;
    cfg.common.limits.decelSpsPerSec = 20000;
    cfg.common.limits.maxStepRateSps = 200000;
    cfg.stepPin   = StepPin{18};
    cfg.dirPin    = DirectionPin{19};
    cfg.enablePin = EnablePin{21};

    // Wire the home switch — normally-closed, fail-safe.
    cfg.sensors[0].pin        = 34;
    cfg.sensors[0].role       = SensorRole::Home;
    cfg.sensors[0].polarity   = SensorPolarity::NormallyClosed;
    cfg.sensors[0].direction  = Direction::Backward;
    cfg.sensors[0].debounceMs = 20;
    cfg.sensorCount           = 1;

    axis = Axis::createStepDirStepper(cfg).takeValue();
    axis->begin();
    axis->enable();
    axis->setHomingStrategy(&strategy, /*timeoutMs=*/30000);
    axis->home();                                  // returns immediately; runs async
}

void loop() {
    axis->service(millis());                       // drives the homing FSM

    static bool didFirstMove = false;
    if (!didFirstMove && axis->isHomed()) {
        didFirstMove = true;
        axis->moveTo(1600);                        // absolute, from home
    }
}
```

Full sketch in [`examples/limit_switch_homing/`](examples/limit_switch_homing/).

### Tiered limit switches (home + travel + E-stop on both ends)

A typical vertical axis benefits from layered safety: a home reference at
one end, a redundant E-stop at the same end (in case the home switch fails
to stop motion), a travel limit at the other end, and a redundant E-stop
there too. All four are normally-closed so a cut wire reads as "pressed".

```cpp
// Top end of travel: home reference + redundant E-stop.
cfg.sensors[0].pin       = 21;   // LIMIT_HOME_1 — home reference
cfg.sensors[0].role      = SensorRole::Home;
cfg.sensors[0].polarity  = SensorPolarity::NormallyClosed;
cfg.sensors[0].direction = Direction::Backward;

cfg.sensors[1].pin       = 22;   // LIMIT_HOME_2 — redundant E-stop at top
cfg.sensors[1].role      = SensorRole::EmergencyStop;
cfg.sensors[1].polarity  = SensorPolarity::NormallyClosed;

// Bottom end of travel: travel limit + redundant E-stop.
cfg.sensors[2].pin       = 18;   // LIMIT_BOTTOM_1 — travel limit (soft halt)
cfg.sensors[2].role      = SensorRole::TravelLimit;
cfg.sensors[2].polarity  = SensorPolarity::NormallyClosed;
cfg.sensors[2].direction = Direction::Forward;

cfg.sensors[3].pin       = 19;   // LIMIT_BOTTOM_2 — redundant E-stop at bottom
cfg.sensors[3].role      = SensorRole::EmergencyStop;
cfg.sensors[3].polarity  = SensorPolarity::NormallyClosed;

cfg.sensorCount = 4;
```

The two E-stops feed into the GPIO ISR path; the home and travel-limit
sensors are polled and debounced at the service tick.

## Speed and acceleration in user units

`Speed` and `Acceleration` are tiny value-types that pair a magnitude with
a unit. The Axis resolves them to internal `Velocity` (steps/sec) or
steps/sec² using the `UnitScaling` from `axis_config` — `stepsPerMm` for
linear units, `stepsPerDegree` for rotary.

```cpp
#include <ungula/motor/motion_units.h>

using namespace ungula::motor;

// Build a config with friendly user units. setup-time helpers mutate
// the TrajectoryLimits inside the config:
StepDirStepperAxisConfig cfg;
cfg.common.units.stepsPerMm     = 80.0f;
cfg.common.units.stepsPerDegree = 8.889f;

applyMaxVelocity (cfg.common.limits, Speed::mmPerSec(150.0f),               cfg.common.units);
applyAcceleration(cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);
applyDeceleration(cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);
// ... or, symmetric:
applyRampProfile (cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);

// Once the axis is built, jog with a per-call feed:
axis->jog(Direction::Forward, Speed::cmPerSec(2.5f));
axis->jog(Direction::Backward, Speed::rpm(60.0f));
```

Magnitude is always non-negative — `Direction` is supplied separately.
An unconfigured scaling field (e.g. asking for `mmPerSec` with
`stepsPerMm == 0`) returns `ErrorCode::InvalidConfig` instead of silently
producing a zero.

| Family | Linear units | Rotary units |
| --- | --- | --- |
| Speed | `mmPerSec`, `mmPerMin`, `cmPerSec`, `cmPerMin`, `inchesPerSec`, `inchesPerMin` | `degreesPerSec`, `rpm`, `rps` |
| Acceleration | `mmPerSecSquared`, `cmPerSecSquared`, `inchesPerSecSquared` | `degreesPerSecSquared`, `rpmPerSec`, `rpsPerSec` |
| Native | `stepsPerSec` | `stepsPerSecSquared` |

## Live trajectory tuning

The Axis exposes runtime setters that mutate the live `TrajectoryLimits`:

```cpp
axis->setMaxVelocity (Speed::mmPerSec(200.0f));
axis->setAcceleration(Acceleration::mmPerSecSquared(800.0f));
axis->setDeceleration(Acceleration::mmPerSecSquared(800.0f));
axis->setRampProfile (Acceleration::mmPerSecSquared(800.0f));  // accel == decel
```

Semantics — these are deliberately not symmetric:

- **`setMaxVelocity` while JOGGING**: the engine is stopped and re-armed
  at the new feed. Brief glitch, no fault. Use this to retarget feed
  during an indefinite jog (e.g. teach-pendant jog at variable speed).
- **`setMaxVelocity` while MOVING** (`moveTo`/`moveBy` in flight): the
  in-flight profile is already segmented in the engine queue and is
  locked. The new value applies on the next motion.
- **`setAcceleration` / `setDeceleration` / `setRampProfile`**: always
  next-motion only. Changing ramp coefficients mid-flight would require
  splicing a fresh ramp into the segment queue, which the engine doesn't
  support today.

## Idle policy and the `AutoDisableOnIdle` listener

When a motion ends the axis can either keep the driver enabled (TMC2209
transitions IRUN → IHOLD after `iHoldDelay`, so the motor still holds
position) or drop the EN pin entirely (cool, silent, but the carrier
loses position reference). Use the policy for the common case; reach
for a listener for richer behaviours.

```cpp
// Built-in policy — runs as an internal listener:
axis->setDefaultIdlePolicy(IdlePolicy::AutoDisable);
// or, the default:
axis->setDefaultIdlePolicy(IdlePolicy::HoldCurrent);
```

Equivalent behaviour written as a host-side listener — the user-extensible
shape behind `setDefaultIdlePolicy(AutoDisable)`. Wire your own version
when you want to do more than just disable (light a pillar, log the
cycle, kick a brake solenoid):

```cpp
#include <ungula/motor/axis.h>
#include <ungula/motor/events/axis_event.h>

using namespace ungula::motor;

class AutoDisableOnIdle final : public IAxisEventListener {
public:
    explicit AutoDisableOnIdle(Axis& a) : axis_(a) {}

    void onAxisEvent(const AxisEvent& ev) override {
        switch (ev.type) {
        case AxisEventType::MotionCompleted:
        case AxisEventType::MotionStopped:
            // Drop EN at the end of every move. Motor coils freewheel;
            // the carrier is held only by mechanical friction / a brake.
            (void) axis_.disable();
            break;
        default:
            break;
        }
    }

private:
    Axis& axis_;
};

AutoDisableOnIdle idleListener(*axis);
axis->subscribe(&idleListener);
```

The two coexist. If you only need plain auto-disable, set the policy. If
you need to disable AND do something else (signalling, logging, brake
control) keep the built-in policy at `HoldCurrent` and run a custom
listener that owns the full behaviour — otherwise you get two `disable()`
calls per motion end, which is harmless but noisy.

The host must call `enable()` again before the next motion command when
`AutoDisable` is active (or when a custom listener disables the axis).
There is no automatic re-enable — eliminates the "the motor jumped after
clearFault" failure mode the same way as `clearFault()` returning to
`Disabled` rather than `Idle`.

## Stall-based homing

For axes that home against a mechanical hard-stop instead of a wired
switch, `StallHomingStrategy` runs the same FSM shape as the limit-switch
strategy but watches the TMC2209's DIAG (StallGuard) output. Requires:

- A `SensorRole::Stall` sensor wired to the DIAG pin (NormallyOpen,
  ISR-driven with hit-count debounce).
- The TMC2209 in **StealthChop** mode (StallGuard4 is StealthChop-only).
- `Tmc2209StallGuard::begin()` called with valid SGTHRS / TCOOLTHRS for
  the motor + load. `verifyChopperMode = true` (the default) refuses
  to configure StallGuard if GCONF reports SpreadCycle — silent
  "stall never fires" is far harder to diagnose later than a clean
  `InvalidConfig` at boot.

```cpp
#include <ungula/motor/homing/stall_homing_strategy.h>

using namespace ungula::motor;

// 1) Sensor for the DIAG pin — Stall role with debounce knobs.
cfg.sensors[0].pin                = 33;     // wired to TMC2209 DIAG
cfg.sensors[0].role               = SensorRole::Stall;
cfg.sensors[0].polarity           = SensorPolarity::NormallyOpen;
cfg.sensors[0].direction          = Direction::Backward;
cfg.sensors[0].stallHitsToTrigger = 4;      // filter spurious pulses
cfg.sensors[0].stallArmDelayMs    = 200;    // ignore StealthChop auto-tune transient
cfg.sensorCount                   = 1;

// 2) StallGuard config (after Tmc2209Configurator::begin in StealthChop).
tmc2209::Tmc2209StallGuard::Config sg;
sg.sgThreshold      = 10;          // SG_RESULT < 2*SGTHRS triggers stall
sg.tCoolThrs        = 0xFFFFF;     // active at any non-zero velocity
sg.verifyChopperMode = true;       // refuses to configure if SpreadCycle
tmcStallGuard.begin(sg);

// 3) Homing strategy — same shape as LimitSwitchHomingStrategy.
StallHomingStrategy::Config hc;
hc.approachDirection = Direction::Backward;
hc.fastFeedSps       = 1500;
hc.slowFeedSps       = 200;
hc.backoffSteps      = 200;
hc.homePositionSteps = 0;
StallHomingStrategy strategy(hc);

axis->setHomingStrategy(&strategy, /*timeoutMs=*/30000);
axis->home();
```

During homing, a latched stall is treated as a **success signal** — the
axis soft-stops the move and the strategy advances to the next phase.
Outside a homing cycle the same stall raises `FaultCode::Stall` with
`StopReason::StallDetected`. The Axis tracks the difference internally;
strategies and applications don't need to.

## The service loop

The host calls `Axis::service(nowMs)` from a regular task tick. **1 ms is typical; 10 ms is acceptable for low-speed work.** A single `service()` call:

1. Polls debounced sensors (Home, TravelLimit) and consumes ISR-latched ones (CrashLimit, EmergencyStop).
2. Halts motion when a TravelLimit in the current move's direction activates, or when a Home sensor fires during a homing cycle.
3. Detects engine motion-complete transitions and emits `MotionCompleted` / `MotionStopped`.
4. Ticks the homing controller — drives the strategy's FSM through its phases.
5. Drains the event queue and dispatches to listeners from task context.

```cpp
void loop() {
    axis->service(millis());
    // ... your own work ...
}
```

The pulse engine is **not** dependent on `service()` running on time. If the loop blocks for 500 ms, the engine keeps stepping; the missed time only delays event delivery and limit-switch reaction (CrashLimit/EmergencyStop still halt the engine instantly through their GPIO ISR).

### Raw ESP-IDF `app_main` — yield the task

If you call `service()` from a raw `app_main` loop (not Arduino's `loop()`, which the framework yields implicitly), you must yield the task yourself. Otherwise the main task pins one core at 100%, `IDLE0` never runs, and the task watchdog trips after ~5 s:

```cpp
#include <ungula/core/time/time.h>

extern "C" void app_main(void) {
    setup();
    for (;;) {
        axis->service(ungula::core::time::millis());
        // One scheduler tick — guaranteed to feed the watchdog.
        ungula::core::time::yield();
    }
}
```

**Use `yield()`, not `delay(N)`.** The right minimum-blocking duration depends on the target's scheduler tick:

- Stock ESP-IDF runs FreeRTOS at 100 Hz → 1 tick = **10 ms**. `pdMS_TO_TICKS(1..9)` rounds *down* to zero ticks; `vTaskDelay(0)` only does a `taskYIELD()` that can't reach `IDLE0` (priority 0 vs main's priority 1), so the watchdog still starves.
- If you raise `CONFIG_FREERTOS_HZ` to 1000, 1 tick = **1 ms** — and `yield()` blocks for that instead.

`yield()` knows the tick from the platform header and always blocks for exactly one tick — no magic millisecond constant in your code. On the host backend it maps to `std::this_thread::yield()`, which is correct for host tests (no watchdog to feed).

If you need a *specific* cadence (e.g. 5 ms periodic sampling decoupled from the FreeRTOS tick), use `delayUntilMs()` with a value that rounds to ≥ 1 tick. The pulse engine doesn't care about service cadence — it lives in the timer ISR and runs autonomously. The cadence only affects sensor debounce latency and homing FSM responsiveness.

## Blocking move wait (20 cm)

If you want to command a move and block until it either reaches target or stops for another reason (stall, limit, fault, user stop), run a bounded `while` loop that keeps calling `service()`.

The important part is not the loop itself, but the exit criteria:

- keep servicing the axis every iteration
- treat `Idle` and `Disabled` as terminal (AutoDisable can move `Idle -> Disabled` immediately after completion)
- inspect `lastStopReason()` and `faultStatus()` to classify why motion ended
- add a timeout so a miswired system cannot block forever

```cpp
#include <cstdint>

#include <ungula/core/time/time.h>
#include <ungula/motor.h>

using namespace ungula::motor;

enum class MoveWaitOutcome : uint8_t {
    ReachedTarget,
    StoppedByLimit,
    StoppedByStall,
    StoppedByUser,
    Faulted,
    Timeout,
    CommandRejected,
};

struct MoveWaitResult {
    MoveWaitOutcome outcome = MoveWaitOutcome::Faulted;
    StopReason stopReason = StopReason::None;
    FaultStatus fault{};
};

MoveWaitResult moveBy20CmAndWait(Axis& axis, uint32_t timeoutMs)
{
    MoveWaitResult result;

    const Status cmd = axis.moveBy(/*delta=*/20, DistanceUnit::Cm);
    if (!cmd.ok()) {
        result.outcome = MoveWaitOutcome::CommandRejected;
        return result;
    }

    const int64_t startedAtMs = ungula::core::time::millis();

    while (true) {
        const int64_t nowMs = ungula::core::time::millis();
        axis.service(nowMs);

        const AxisState st = axis.state();
        const StopReason reason = axis.lastStopReason();
        const FaultStatus fault = axis.faultStatus();

        if (st == AxisState::Faulted || st == AxisState::EmergencyStopped) {
            result.outcome = MoveWaitOutcome::Faulted;
            result.stopReason = reason;
            result.fault = fault;
            return result;
        }

        if (st == AxisState::Idle || st == AxisState::Disabled) {
            result.stopReason = reason;
            result.fault = fault;
            switch (reason) {
            case StopReason::TargetReached:
                result.outcome = MoveWaitOutcome::ReachedTarget;
                break;
            case StopReason::TravelLimit:
            case StopReason::LimitSwitch:
                result.outcome = MoveWaitOutcome::StoppedByLimit;
                break;
            case StopReason::StallDetected:
                result.outcome = MoveWaitOutcome::StoppedByStall;
                break;
            case StopReason::UserStop:
                result.outcome = MoveWaitOutcome::StoppedByUser;
                break;
            default:
                result.outcome = fault.active() ? MoveWaitOutcome::Faulted :
                                                 MoveWaitOutcome::StoppedByUser;
                break;
            }
            return result;
        }

        if (timeoutMs > 0 && (nowMs - startedAtMs) >= static_cast<int64_t>(timeoutMs)) {
            (void) axis.stop(StopMode::Immediate);
            result.outcome = MoveWaitOutcome::Timeout;
            result.stopReason = axis.lastStopReason();
            result.fault = axis.faultStatus();
            return result;
        }

        // In raw app_main loops this prevents starving IDLE/WDT.
        ungula::core::time::yield();
    }
}
```

Typical use:

```cpp
MoveWaitResult r = moveBy20CmAndWait(*axis, /*timeoutMs=*/15000);
if (r.outcome == MoveWaitOutcome::ReachedTarget) {
    // success
} else {
    // inspect r.outcome, r.stopReason, r.fault
}
```

This pattern is safe for one-shot "go there and wait" flows in setup, calibration, recipe steps, and test routines.

## Events

Subscribe a listener; events arrive at `service()`-drain time, never from ISR.

```cpp
// For Arduino's style use Arduino.h and replace the `log_info` call by a `Serial.printf()`
#include <emblogx/logger.h>
#include <ungula/motor.h>

using namespace ungula::motor;

class SerialListener final : public IAxisEventListener {
    public:
        void onAxisEvent(const AxisEvent &ev) override
        {
                log_info("[axis %u] %s  state=%s  pos=%ld  reason=%s  fault=%s", ev.axisId.value,
                         axisEventTypeToString(ev.type), axisStateToString(ev.state),
                         static_cast<long>(ev.commandedPosition), stopReasonToString(ev.stopReason),
                         faultToString(ev.faultCode));
        }
};
SerialListener listener;

void setup() {
        // ... axis initialization
        axis->subscribe(&listener);
}
```

Event types [AxisEventType](./src/ungula/motor/events/axis_event.h) emitted by the Axis:

| Event | Trigger |
| --- | --- |
| `StateChanged` | Any FSM transition |
| `MotionStarted` | `moveTo` / `moveBy` / `jog` armed the engine |
| `MotionCompleted` | Engine reported `TargetReached` |
| `MotionStopped` | Engine stopped before reaching target (user stop, limit, fault) |
| `LimitActivated` | Crash/E-stop ISR fired, or travel-limit in direction activated |
| `HomingStarted` | `home()` accepted; controller is `FastApproach` |
| `HomingCompleted` | Controller reached `Complete` |
| `HomingFailed` | Controller reached `Failed` (timeout, stuck switch, etc.) |
| `FaultRaised` | Driver fault, pulse-engine fault, or limit/estop fault |
| `FaultCleared` | `clearFault()` succeeded |
| `EmergencyStopped` | `emergencyStop()` or E-stop input fired |

The listener slot count is fixed at compile time (`MAX_AXIS_EVENT_LISTENERS`, default 4). The event queue capacity is the template parameter on `AxisEventQueue<N>` (default 32).

## Sensors, limits, and safety inputs

`SensorRole` distinguishes the five kinds of switches. They have **different** timing and safety semantics — keep them separate.

| Role | Path | Use for |
| --- | --- | --- |
| `Home` | Polled with `debounceMs` filter | Reference / homing only |
| `TravelLimit` | Polled with debounce | Soft limits that should halt motion at the service tick |
| `CrashLimit` | **GPIO ISR** → `engine.haltFromIsr()` | Hard limit that must stop the motor immediately |
| `EmergencyStop` | **GPIO ISR** → `engine.haltFromIsr()` + latch | E-stop circuit (palm button, door switch, gate) |
| `Stall` | **GPIO ISR** with hit-count + arm-delay debounce | TMC2209 DIAG output. Reports `StallDetected` / `Stall` so the host can tell a chip stall apart from a driver fault. During a homing cycle a stall is treated as a success signal (soft-stop, no fault). |

The `CrashLimit` and `EmergencyStop` ISR path is sub-microsecond: the GPIO ISR calls `IPulseEngine::haltFromIsr()` directly, which disarms the hardware timer and latches the fault. Bookkeeping (event emission, state transition) happens at the next `service()` tick from task context.

```cpp
SensorInputConfig sensors[3];
// Home reference — polled, debounced.
sensors[0].pin       = 34;
sensors[0].role      = SensorRole::Home;
sensors[0].polarity  = SensorPolarity::NormallyClosed;
sensors[0].direction = Direction::Backward;
sensors[0].debounceMs = 20;

// Travel limit on the positive side — polled.
sensors[1].pin       = 35;
sensors[1].role      = SensorRole::TravelLimit;
sensors[1].polarity  = SensorPolarity::NormallyClosed;
sensors[1].direction = Direction::Forward;

// Hardware E-stop — ISR-driven.
sensors[2].pin       = 32;
sensors[2].role      = SensorRole::EmergencyStop;
sensors[2].polarity  = SensorPolarity::NormallyClosed;

cfg.sensors[0]  = sensors[0];
cfg.sensors[1]  = sensors[1];
cfg.sensors[2]  = sensors[2];
cfg.sensorCount = 3;
```

`NormallyClosed` is the default convention for safety inputs: a cut or disconnected wire reads as "pressed" (fail-safe).

## TMC2209 driver split

The TMC2209 driver lives in `motor/drivers/tmc2209/` and is split into four single-responsibility components. Each is independent — pick the ones you need.

| Component | Responsibility | UART traffic |
| --- | --- | --- |
| `Tmc2209Configurator` | Boot config (mA-based currents, microsteps, chopper, mode, GSTAT clear) | Writes only, during boot |
| `Tmc2209Diagnostics` | Opt-in `DRV_STATUS` / `IOIN` / `GSTAT` snapshot + on-demand reads | Off motion path |
| `Tmc2209StallGuard` | SGTHRS / TCOOLTHRS config; refuses to configure in SpreadCycle | DIAG-pin is the default detection path |
| `Tmc2209CoolStep` | Load-based dynamic current scaling (SEMIN/SEMAX/SEUP/SEDN/SEIMIN) | Writes only, during boot or runtime |

### Currents in milliamps

The configurator takes `runCurrentMa` / `holdCurrentMa` in **mA RMS** and converts them to the chip's 5-bit IRUN/IHOLD fields using the sense resistor on your board:

```cpp
tmc2209::Tmc2209Configurator::Config c;
c.runCurrentMa        = 800;     // mA RMS at the coil
c.holdCurrentMa       = 300;
c.iHoldDelay          = 1;
c.senseResistorOhms   = 0.11f;   // check the silk screen on your carrier
c.useHighSensitivity  = false;   // true for <300 mA setups (vsense=1 → Vfs=0.180V)
c.microsteps          = tmc2209::Microsteps::Sixteenth;
c.mode                = tmc2209::ChopperMode::StealthChop;
config.begin(c);

// At runtime — same units. Cached sense-resistor settings are reused.
config.setCurrents(/*runMa=*/600, /*holdMa=*/200, /*iHoldDelay=*/1);

// Two static helpers expose the conversion for diagnostics / UIs:
const uint8_t  cs    = tmc2209::Tmc2209Configurator::milliampsToCs(600, 0.11f);
const uint16_t back  = tmc2209::Tmc2209Configurator::csToMilliamps(cs,  0.11f);
```

`runCurrentMa > 0` and `holdCurrentMa <= runCurrentMa` are validated at `begin()`. The conversion formula is `CS = round(I_rms · 32 · √2 · (R_sense + 0.02) / V_fs) - 1`, clamped to 0..31. `V_fs` is 0.325 V at vsense=0 and 0.180 V at vsense=1; the 0.02 Ω term is the chip's internal MOSFET resistance.

### Boot, diagnostics, and DIAG-based stall detection

```cpp
#include <ungula/hal/uart/uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_stallguard.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>

ungula::hal::uart::Uart            uart(1);
ungula::motor::tmc2209::Tmc2209HalUart      transport(uart, /*slaveAddress=*/0);
ungula::motor::tmc2209::Tmc2209Configurator config(transport);
ungula::motor::tmc2209::Tmc2209Diagnostics  diag(transport);
ungula::motor::tmc2209::Tmc2209StallGuard   sg(transport);

void setup() {
    uart.begin(115200, /*tx=*/17, /*rx=*/16);

    ungula::motor::tmc2209::Tmc2209Configurator::Config c;
    c.runCurrentMa      = 800;
    c.holdCurrentMa     = 300;
    c.senseResistorOhms = 0.11f;
    c.microsteps        = ungula::motor::tmc2209::Microsteps::Sixteenth;
    c.mode              = ungula::motor::tmc2209::ChopperMode::StealthChop;
    config.begin(c);

    // Optional StallGuard — fully off the motion path. The chip
    // pulses its DIAG output when a stall is detected; wire DIAG
    // to a Stall sensor on the Axis side.
    //
    // `verifyChopperMode = true` (default) reads GCONF first and
    // returns InvalidConfig if the chip is in SpreadCycle — SG4 is
    // StealthChop-only on the TMC2209 and a silent "stall never
    // fires" misconfiguration is hard to diagnose later.
    ungula::motor::tmc2209::Tmc2209StallGuard::Config sCfg;
    sCfg.sgThreshold      = 10;
    sCfg.tCoolThrs        = 0xFFFFF;
    sCfg.verifyChopperMode = true;
    sg.begin(sCfg);
}

void diagnoseFromLowPriorityTask() {
    // Diagnostics are never called from the motion path. Run this
    // from a 100 ms-ish task or on-demand after a fault event.
    if (diag.refresh().ok()) {
        if (diag.snapshot().driveFault()) {
            // Drive reports overtemperature, short, etc.
        }
    }
}
```

### CoolStep (load-based current scaling)

CoolStep watches `SG_RESULT` (the same load measurement StallGuard uses) and ramps coil current up or down based on actual mechanical load. Lighter load → lower current → cooler chip. It's strictly better than host-side velocity-curves because it keys off load, not commanded speed.

CoolStep is **independent** of StallGuard. You can run either alone, or both. Both write to `TCOOLTHRS`; when using both, set it on one and leave the other at `0` to skip writing.

```cpp
#include <ungula/motor/drivers/tmc2209/tmc2209_coolstep.h>

ungula::motor::tmc2209::Tmc2209CoolStep cool(transport);

ungula::motor::tmc2209::Tmc2209CoolStep::Config cs;
cs.loadThresholdToIncrease       = 4;       // SEMIN — 1..15 enables, 0 disables
cs.loadThresholdToDecreaseMargin = 2;       // SEMAX — hysteresis width
cs.currentRampUp                 = ungula::motor::tmc2209::Tmc2209CoolStep::StepWidth::Step1;
cs.currentRampDown               = ungula::motor::tmc2209::Tmc2209CoolStep::StepWidth::Step2;
cs.currentFloor                  = ungula::motor::tmc2209::Tmc2209CoolStep::CurrentFloor::Half;
cs.tCoolThrs                     = 0;       // leave StallGuard's value alone
cool.begin(cs);

// Disable at runtime — clears SEMIN only, other fields keep their values:
cool.disable();
```

`Tmc2209Configurator::clearGstat()` writes the W1C mask. The original driver had a read-instead-of-write bug here that left flags latched after boot; the regression test (`test_tmc2209_configurator.cpp`) locks the correct behaviour.

## Driver authoring guide

If you need to add support for a new driver brand/model, use:

- [`src/ungula/motor/drivers/README.md`](src/ungula/motor/drivers/README.md)

That guide defines the required architecture, transport/configurator split, safety/fault mapping, testing checklist, and documentation updates expected for new driver integrations.

## Faults, stop modes, and recovery

```cpp
enum class StopMode : uint8_t {
    Decelerate,  // not implemented in 3.0.0 — returns Unsupported
    Immediate,   // halt on next ISR boundary; no decel ramp
    Emergency,   // same hardware path; latches a fault
};
```

The library **never** silently substitutes one stop mode for another. `stop(Decelerate)` returns `ErrorCode::Unsupported` until the engine learns to splice a fresh deceleration profile into an in-flight move. Motion-control safety depends on accurate stop semantics — lying here is a bug.

After an emergency stop or a driver/limit fault, `clearFault()` returns the axis to `Disabled` (not `Idle`). The host must explicitly call `enable()` again before the next motion — eliminates the "the motor jumped because I called clearFault" surprise.

```cpp
const auto fs = axis->faultStatus();
if (fs.active()) {
    Serial.printf("fault: %s (driver detail=0x%lx)\n",
                  faultToString(fs.code),
                  static_cast<unsigned long>(fs.driverDetail));
    axis->clearFault();
    axis->enable();
}
```

## Composing axes by hand

The three `create*()` factories handle 99% of cases. For the remaining 1% — host integration tests, custom timer backends, alternative actuators — there's `createComposed`:

```cpp
#include <ungula/motor.h>
#include <ungula/motor/pulse/fake_pulse_engine.h>
#include <ungula/motor/actuator/step_dir_actuator.h>

using namespace ungula::motor;

auto engine   = std::make_unique<FakePulseEngine>();    // or your own IPulseEngine
StepDirActuator::Config aCfg;
auto actuator = std::make_unique<StepDirActuator>(*engine, aCfg);

Axis::ComposedComponents comp;
comp.timer    = nullptr;                                // fake engine doesn't need one
comp.actuator = std::move(actuator);
comp.common.limits.maxVelocitySps = 4000;
comp.common.limits.accelSpsPerSec = 8000;
comp.common.limits.decelSpsPerSec = 8000;
comp.common.limits.maxStepRateSps = 200000;
comp.engine   = std::move(engine);

auto r = Axis::createComposed(std::move(comp));
auto axis = r.takeValue();
```

This is the same factory `test_axis_e2e_smoke` uses to drive a real `Axis` against `FakePulseEngine` for deterministic host testing.

## Testing

Host-side tests live under `tests/` and use GoogleTest. Build them with the workspace's regular CMake flow:

```bash
cd lib_motor/tests
mkdir -p build && cd build
cmake ..
cmake --build .
ctest --output-on-failure

# Or
cd lib_motor/tests
chmod +x *sh
./1_build.sh
./2_run.sh
```

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT. Copyright (c) 2026 Alex Conesa. See [`LICENSE`](LICENSE).
