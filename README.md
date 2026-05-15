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
- [Quick start — limit-switch homing](#quick-start--limit-switch-homing)
- [The service loop](#the-service-loop)
- [Events](#events)
- [Sensors, limits, and safety inputs](#sensors-limits-and-safety-inputs)
- [TMC2209 driver split](#tmc2209-driver-split)
- [Faults, stop modes, and recovery](#faults-stop-modes-and-recovery)
- [Composing axes by hand](#composing-axes-by-hand)
- [Testing](#testing)
- [License](#license)

## Features

- **One `Axis` facade for three drive families** — open-loop steppers, STEP/DIR servos, CAN servos. The wire protocol differs; the application code does not.
- **ISR-driven pulse generation.** Steps are emitted from a hardware-timer ISR off precomputed `MotionSegment` queues. The main loop can block for hundreds of milliseconds without dropping a step.
- **Integer-only motion planner.** Trapezoidal + triangular profiles, jog, soft-stop ramps. Step count is preserved exactly — no silent drift from rounding.
- **Typed results everywhere.** `Result<T>` / `Status` instead of `void`/silent failures. No exceptions, no RTTI.
- **Heap allocation only at boot.** Factories return `Result<std::unique_ptr<Axis>>`; steady-state runtime is allocation-free.
- **Sensor-aware limits.** `HomeSensor` (polled, debounced), `TravelLimitSensor` (polled), `CrashLimitSensor` and `EmergencyStop` (GPIO interrupt → sub-µs halt of the pulse engine).
- **Explicit homing FSM.** `Idle → FastApproach → Backoff → SlowApproach → SetHomePosition → Complete/Failed`. No more racing against auto-clearing terminal states.
- **TMC2209 split into focused components.** `Tmc2209Configurator` (boot config, no reads), `Tmc2209Diagnostics` (opt-in register reads, off motion path), `Tmc2209StallGuard` (DIAG-pin first, optional UART polling). UART traffic is never on the motion timing path.
- **Bounded event queue.** ISR-safe enqueue, task-context drain. Listeners are dispatched from `service()` — never from ISR.

## Supported drives

| Drive family | Wire protocol | Driver class | Notes |
| --- | --- | --- | --- |
| Open-loop stepper (TMC2209, A4988, DRV8825) | STEP + DIR + EN (active-LOW) | `StepDirActuator` | Optional UART configuration via `Tmc2209Configurator` |
| STEP/DIR servo (Yaskawa, Delta, Leadshine) | STEP + DIR + SVON (typically active-HIGH) + optional ALM / INP | `StepDirActuator` (kind = `StepDirServo`) | Drive owns position truth; encoder feedback wiring deferred |
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
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>
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
│    └── LimitSwitchHomingStrategy
├── AxisEventQueue                    -- bounded; ISR-safe enqueue, task drain
│
└── drivers/tmc2209/
     ├── ITmcUart                     -- transport abstraction
     │    ├── Tmc2209HalUart          -- lib_hal Uart adapter (CRC, framing)
     │    └── FakeTmcUart             -- test fake (register file in memory)
     ├── Tmc2209Configurator          -- boot config; no reads
     ├── Tmc2209Diagnostics           -- opt-in reads, off motion path
     └── Tmc2209StallGuard            -- DIAG-pin + optional SG_RESULT polling
```

`Axis` is the only type most applications need. The factories validate the config, allocate the underlying components, and return a fully wired axis ready for `begin()`.

## Quick start — open-loop stepper (TMC2209)

The canonical setup: ESP32 + TMC2209 + NEMA17 stepper.

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
    tmc2209::Tmc2209Configurator::Config tc;
    tc.runCurrent  = 16;          // 0..31 (scaled by Vref + sense resistor)
    tc.holdCurrent = 8;
    tc.microsteps  = tmc2209::Microsteps::Sixteenth;
    tc.mode        = tmc2209::ChopperMode::StealthChop;
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

## Events

Subscribe a listener; events arrive at `service()`-drain time, never from ISR.

```cpp
#include <Arduino.h>

#include <ungula/motor.h>

using namespace ungula::motor;

class PrintingListener final : public IAxisEventListener {
public:
    void onAxisEvent(const AxisEvent& ev) override {
        Serial.printf("[%lu] axis %u  %s  state=%s  pos=%ld  reason=%s  fault=%s\n",
                      static_cast<unsigned long>(ev.timestampMs),
                      ev.axisId.value,
                      axisEventTypeToString(ev.type),
                      axisStateToString(ev.state),
                      static_cast<long>(ev.commandedPosition),
                      stopReasonToString(ev.stopReason),
                      faultToString(ev.faultCode));
    }
};
PrintingListener listener;

// inside setup():
//   axis->subscribe(&listener);
```

Event types emitted by the Axis:

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

`SensorRole` distinguishes the four kinds of switches. They have **different** timing and safety semantics — keep them separate.

| Role | Path | Use for |
| --- | --- | --- |
| `Home` | Polled with `debounceMs` filter | Reference / homing only |
| `TravelLimit` | Polled with debounce | Soft limits that should halt motion at the service tick |
| `CrashLimit` | **GPIO ISR** → `engine.haltFromIsr()` | Hard limit that must stop the motor immediately |
| `EmergencyStop` | **GPIO ISR** → `engine.haltFromIsr()` + latch | E-stop circuit (palm button, door switch, gate) |

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

The TMC2209 driver lives in `motor/drivers/tmc2209/` and is split into three single-responsibility components:

| Component | Responsibility | UART traffic |
| --- | --- | --- |
| `Tmc2209Configurator` | Boot config (currents, microsteps, chopper, mode, GSTAT clear) | Writes only, during boot |
| `Tmc2209Diagnostics` | Opt-in `DRV_STATUS` / `IOIN` / `GSTAT` snapshot + on-demand reads | Off motion path |
| `Tmc2209StallGuard` | SGTHRS / TCOOLTHRS config, opt-in SG_RESULT read | DIAG-pin is the default detection path |

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
    c.runCurrent  = 16;
    c.holdCurrent = 8;
    c.microsteps  = ungula::motor::tmc2209::Microsteps::Sixteenth;
    c.mode        = ungula::motor::tmc2209::ChopperMode::StealthChop;
    config.begin(c);

    // Optional StallGuard — fully off the motion path. The chip
    // pulses its DIAG output when a stall is detected; wire DIAG
    // to a CrashLimit sensor on the Axis side.
    ungula::motor::tmc2209::Tmc2209StallGuard::Config sCfg;
    sCfg.sgThreshold = 10;
    sCfg.tCoolThrs   = 0x1000;
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

`Tmc2209Configurator::clearGstat()` writes the W1C mask. The original driver had a read-instead-of-write bug here that left flags latched after boot; the regression test (`test_tmc2209_configurator.cpp`) locks the correct behaviour.

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
