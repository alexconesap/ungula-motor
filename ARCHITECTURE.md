# UngulaMotor — Architecture

This document describes who owns what inside `lib_motor`. The
[`README.md`](README.md) covers user-facing setup and the high-level
overview; this file goes one layer deeper: per-class responsibilities,
ownership graph, ISR / task boundaries, threading rules, and how data
flows from a `moveBy()` call to STEP edges on a GPIO pin.

If you're building an application on top of the library, you can skip
this document. If you're modifying the library, fixing a stall path,
or adding a new driver, read this first.

## Table of contents

- [The one rule](#the-one-rule)
- [Layered design](#layered-design)
- [Class catalogue](#class-catalogue)
  - [Public facade](#public-facade)
  - [Motion planning](#motion-planning)
  - [Pulse generation](#pulse-generation)
  - [Actuators](#actuators)
  - [Sensors / limits / safety](#sensors--limits--safety)
  - [Homing](#homing)
  - [Events](#events)
  - [Result types](#result-types)
  - [TMC2209 driver split](#tmc2209-driver-split)
  - [YPMC servo helper](#ypmc-servo-helper)
- [Data flows](#data-flows)
  - [`moveBy()` → STEP edges](#moveby--step-edges)
  - [DIAG edge → `Faulted`](#diag-edge--faulted)
  - [`service()` tick](#service-tick)
- [ISR vs. task context](#isr-vs-task-context)
- [Allocation discipline](#allocation-discipline)
- [Test seams](#test-seams)
- [Extending the library](#extending-the-library)

## The one rule

> **Slow application code, slow UART diagnostics, slow event listeners,
> and slow CAN traffic must never disturb step-pulse timing.**

The pulse engine lives in a hardware-timer ISR. Everything else (the
main loop, UART reads to a TMC2209, listeners, CAN traffic, logging)
runs in task context and **observes** the engine. Nothing in task
context commands step timing on a per-step basis; the engine consumes
a precomputed `PlannedMove` and walks it autonomously.

Every architectural choice in this file falls out of that rule.

## Layered design

```text
   ┌─────────────────────────────────────────────────────────────┐
   │            Application code (your sketch / project)          │
   │   Axis::createStepDirStepper / moveBy / jog / service / ...  │
   └──────────────────────────┬──────────────────────────────────┘
                              │
   ┌──────────────────────────▼──────────────────────────────────┐
   │   Axis facade  (axis.h / axis.cpp)                           │
   │   Owns the pieces below. Translates user-unit commands into  │
   │   step plans, arms the engine, drains events, supervises     │
   │   sensors + homing.                                          │
   └────┬──────────┬──────────┬─────────┬─────────┬─────────┬────┘
        │          │          │         │         │         │
        ▼          ▼          ▼         ▼         ▼         ▼
   MotionPlanner  IAxisActuator  IPulseEngine  SensorBank  HomingController  AxisEventQueue
   (stateless)    (StepDir...)   (HalPulse...)             (FSM driver)
                  (CanServo...)  (FakePulse...)

   ┌─────────────────────────────────────────────────────────────┐
   │   lib_hal  (gpio, timer::IHwTimer, uart, can, sync)          │
   │                       lib_core/time                          │
   └─────────────────────────────────────────────────────────────┘
```

Headers under `ungula/motor/drivers/` (TMC2209, YPMC) sit OFF this
stack — they configure the chip / drive but don't own any axis state
or runtime motion path. They're opt-in helpers, never mandatory.

## Class catalogue

### Public facade

**`Axis`** (`ungula/motor/axis.{h,cpp}`)

The single user-facing class. Created via one of four factories:

- `Axis::createStepDirStepper(StepDirStepperAxisConfig)` — TMC2209,
  A4988, DRV8825, etc.
- `Axis::createStepDirServo(StepDirServoAxisConfig)` — industrial
  servo over STEP/DIR (Yaskawa, Delta, Leadshine, RATTMOTOR S2SVD15).
- `Axis::createCanServo(CanServoAxisConfig)` — CAN-bus servos (returns
  `Unsupported` until a concrete `ICanServoProtocol` is plugged in).
- `Axis::createComposed(ComposedComponents)` — advanced / test path
  that hands over a hand-built timer + engine + actuator triple.

Owns (as `std::unique_ptr`, destruction order matters):

```
Axis
 ├── timer_     : ungula::hal::timer::IHwTimer    (may be null on CAN)
 ├── engine_    : IPulseEngine                    (may be null on CAN)
 └── actuator_  : IAxisActuator                   (mandatory)
```

Plus several value-typed members:

- `planner_`  : `MotionPlanner` — stateless; same instance reused.
- `sensors_`  : `SensorBank` — polled + ISR sensors.
- `homing_`   : `HomingController` — runs the homing FSM.
- `events_`   : `AxisEventQueue<32>` — bounded, ISR-safe.

Public command surface, grouped:

- **Lifecycle**: `begin`, `enable`, `disable`,
  `isSafetyInterlockActive`.
- **Motion**: `moveTo`, `moveBy`, `jog`, `stop`, `emergencyStop`.
- **Trajectory tuning**: `setMaxVelocity`, `setAcceleration`,
  `setDeceleration`, `setRampProfile`.
- **Homing**: `setHomingStrategy`, `home`, `isHoming`, `isHomed`,
  `homingPhase`.
- **Idle policy**: `setDefaultIdlePolicy`, `defaultIdlePolicy`.
- **Faults**: `clearFault`, `faultStatus`.
- **Queries**: `state`, `feedback`, `lastStopReason`, `id`,
  `totalStallHits`.
- **Service**: `service(nowMs)` — the single supervisory tick.
- **Events**: `subscribe(listener)`, `serviceEvents()`.

Inherits `IHomingAxis` so a `HomingStrategy` can call a narrowed
subset (`commandMove`, `commandJog`, `stopMove`, `isHomeActive`,
`isStallActive`, `isMotionIdle`, `resetPosition`) without seeing the
full public surface — prevents strategies from deadlocking against
`Axis::moveTo` etc.

### Motion planning

**`MotionPlanner`** (`planning/motion_planner.{h,cpp}`)

Stateless. Computes integer trapezoidal / triangular step profiles.
Inputs: requested delta (steps) or direction (jog), `TrajectoryLimits`,
timer resolution. Output: a `PlannedMove` of up to 32 `MotionSegment`s.

Key methods:

- `planBy(delta, limits, resolutionHz, minHalfPeriodTicks)` —
  relative move; direction inferred from the delta sign.
- `planTo(currentPos, targetPos, ...)` — thin wrapper.
- `planJog(direction, maxSteps, ...)` — bounded jog; the safety
  bound prevents a runaway if the host never stops it.
- `planStop(direction, currentSps, ...)` — decel-only ramp from a
  given velocity to 0.

Hard guarantee: `sum(segments[i].stepCount) == |delta|` exactly.
Integer-rounding drift is rebalanced into the cruise segment for
trapezoidal moves, or the last decel segment for triangular / stop
ramps. The commanded distance is non-negotiable — no silent rounding.

**`PlannedMove`** (`planning/planned_move.h`) — POD describing a
single end-to-end move:

```cpp
struct PlannedMove {
    Direction      direction;
    uint32_t       totalSteps;          // == sum(segments[].stepCount)
    uint8_t        segmentCount;
    MotionSegment  segments[MAX_PLANNED_SEGMENTS];
};
```

**`MotionSegment`** (`pulse/motion_segment.h`) — a constant-velocity
stretch. `stepCount` steps at `halfPeriodTicks` per half-period.

The planner does no I/O. It's pure math + bookkeeping.

### Pulse generation

**`IPulseEngine`** (`pulse/i_pulse_engine.h`) — abstract interface
between the planner output and the wire. Two implementations:

**`HalPulseEngine`** (`pulse/hal_pulse_engine.{h,cpp}`) — production.

Drives a `lib_hal::timer::IHwTimer` and a STEP / DIR GPIO pair. The
timer's alarm callback runs in ISR context (`UNGULA_ISR_ATTR`,
IRAM-placed). Algorithm:

1. `begin(PulseMode::Internal)` — configures STEP / DIR as outputs,
   brings up the timer.
2. `loadMove(PlannedMove)` — copies the plan into local storage,
   validates every segment against the timer's `minTicks` floor and
   asserts `sum == totalSteps`. Task context.
3. `start()` — writes DIR, waits `dirSetupUs` (busy-wait — task
   context, no ISR yet), arms the timer with the first segment's
   `halfPeriodTicks`. Sets `running_=true` (atomic) before arming so
   the ISR sees consistent state if it fires immediately.
4. **ISR**: every alarm, toggle STEP, advance segment counters, on
   segment boundary re-arm the timer with the next segment's
   `halfPeriodTicks`. On the final edge of the final segment, latch
   `TargetReached`, clear `running_`, disarm.
5. `haltFromIsr(reason)` — entry point for `SensorBank` ISR
   trampolines. Disarms the timer atomically and sets `faulted_=true`.

`status()` / `isRunning()` / `commandedPositionSteps()` are atomic
reads, safe from any context. `loadMove` / `start` / `stop` /
`clearFault` are task-context only.

`StopMode::Decelerate` is rejected with `Unsupported` — splicing a
fresh decel ramp into an in-flight queue isn't implemented; lying
here would be a motion-control safety bug.

**`FakePulseEngine`** (`pulse/fake_pulse_engine.h`) — header-only,
host-only test fake. Same interface, deterministic. `tickSteps(N)`
advances the engine N steps from task context; `runMove()` finishes
the move instantly. Used by `test_axis_e2e_smoke` to exercise the
full pipeline without a real timer / GPIO.

### Actuators

**`IAxisActuator`** (`actuator/i_axis_actuator.h`) — narrow
interface the `Axis` uses to command motion. Methods:
`begin`, `enable`, `disable`, `armMotion(PlannedMove)`,
`startMotion`, `stop(StopMode)`, `emergencyStop`, `clearFault`,
`feedback`, `capabilities`, `faultStatus`.

**`StepDirActuator`** (`actuator/step_dir_actuator.{h,cpp}`) —
covers BOTH open-loop steppers AND STEP/DIR servos. A `kind` flag
(`OpenLoopStepper` / `StepDirServo`) selects the capability set
(open-loop reports `hasActualPosition=false`; servos with encoder
feedback wired report capabilities differently). The pulse stream
itself is identical.

**Direction-timing contract**: the actuator MUST NOT touch the DIR
pin. The pulse engine owns DIR (writes it in `start()`, waits
`dirSetupUs`, then arms the timer). Touching DIR from the actuator
violates drive setup-time guarantees.

The actuator owns the ENABLE pin (active-LOW for steppers,
active-HIGH for industrial servos via `enableActiveLow=false`) and
forwards `armMotion` / `startMotion` / `stop` / etc. to the engine.

**`CanServoActuator`** (`actuator/can_servo_actuator.{h,cpp}`) —
skeleton for CAN-bus servos. Holds a `ICanServoProtocol*` pointer; if
null, motion methods return `Unsupported`. Concrete protocols
(CiA-402, vendor-specific) are not bundled — host plugs one in.

### Sensors / limits / safety

**`SensorInputConfig`** (`limits/sensor_input.h`) — value type
describing one switch / sensor:

```cpp
struct SensorInputConfig {
    uint8_t        pin;
    SensorRole     role;       // Home, TravelLimit, CrashLimit, EmergencyStop, Stall
    SensorPolarity polarity;   // NormallyClosed (default), NormallyOpen
    Direction      direction;
    uint16_t       debounceMs;
    uint8_t        stallHitsToTrigger;   // Stall role only
    uint16_t       stallArmDelayMs;      // Stall role only
};
```

**`SensorBank`** (`limits/sensor_bank.{h,cpp}`) — owns up to
`MAX_SENSOR_INPUTS` sensors. Role-specific behaviour:

| Role           | Mechanism                                    | Side effect                                            |
| -------------- | -------------------------------------------- | ------------------------------------------------------ |
| `Home`         | Polled + per-sensor `debounceMs` filter      | `isActive` query for the homing controller             |
| `TravelLimit`  | Polled + debounce                            | Halts in-flight motion when active in current direction|
| `CrashLimit`   | GPIO ISR → `engine.haltFromIsr(LimitSwitch)` | Latches an atomic flag; sub-µs latency                 |
| `EmergencyStop`| GPIO ISR → `engine.haltFromIsr(EmergencyStop)` + latch | Same plus latches a fault                       |
| `Stall`        | GPIO ISR with hit-count + arm-delay debounce | ISR halts engine when threshold met; cumulative counter tickable from diagnostics |

Polled sensors get the polarity-derived pull-up / pull-down (NC → pull-up,
NO → pull-down) — without this, a floating-but-configured NC pin
reads "active" at rest and halts motion every service tick.

ISR-role sensors register a per-pin trampoline via
`lib_hal::gpio::addIsrHandler`. The trampoline lives in IRAM
(`UNGULA_ISR_ATTR`), reads atomic state, and calls
`engine.haltFromIsr` directly when appropriate. **No allocation, no
UART, no logging from inside the trampoline.**

Stall ISR specifics (3.1.6 — see [LIBRARY_VERSIONS.md](../../docs/LIBRARY_VERSIONS.md)
for the audit-fix changelog):

- ISR increments two counters: `stallHitCounter_` (debounce
  threshold) and `stallHitsTotal_` (cumulative diagnostic).
- ISR halts the engine **only when `stallHitCounter_ >= stallHitsToTrigger`** AND past the arm window. Earlier in 3.1.x the
  ISR halted on the first post-window edge, which faulted the engine
  before the bank latched a stall — a tracking bug fixed in 3.1.6.
- Once a stall is latched, subsequent ISR edges still bump the
  cumulative counter but no longer call `haltFromIsr` — the engine is
  already faulted, re-halting is noise.

**`onIsrTrampoline`** is the single static function the GPIO ISR
service calls. Branches by role: E-stop / crash halt immediately,
Stall goes through the arm-window + threshold gate, Home /
TravelLimit aren't ISR-driven (the trampoline doesn't see them).

### Homing

**`HomingPhase`** (`homing/homing_controller.h`) — explicit 7-state
FSM: `Idle → FastApproach → Backoff → SlowApproach → SetHomePosition
→ Complete / Failed`. Terminal states do NOT auto-clear; the host
sees the terminal phase until the next `home()` call.

**`IHomingStrategy`** (`homing/i_homing_strategy.h`) — interface:
`begin(IHomingAxis&)`, `step(IHomingAxis&) → HomingProgress`,
`finish(IHomingAxis&, bool)`, `currentPhase()`.

**`IHomingAxis`** (`homing/i_homing_axis.h`) — narrowed view of
`Axis` (`commandMove`, `commandJog`, `stopMove`, `isHomeActive`,
`isStallActive`, `isMotionIdle`, `resetPosition`). Strategies see
ONLY this — they can't call `moveTo`, `home`, or `subscribe` and so
can't deadlock against the public command API.

**`HomingController`** (`homing/homing_controller.{h,cpp}`) — drives
a `IHomingStrategy` through its FSM from `Axis::service`. Owns the
strategy pointer (does NOT take ownership; the host's strategy must
outlive the homing cycle), a configurable timeout, and the latched
failure reason.

Strategies:

- **`LimitSwitchHomingStrategy`** (`homing/limit_switch_homing_strategy.{h,cpp}`)
  — wired home switch (`SensorRole::Home`).
- **`StallHomingStrategy`** (`homing/stall_homing_strategy.{h,cpp}`)
  — sensorless homing against a mechanical hard-stop. Uses a stall
  sensor (`SensorRole::Stall`); during the homing cycle the Axis
  treats a stall as success rather than a fault.

Custom strategies plug into the same interface. See `API.md` for
the recipe.

### Events

**`AxisEvent`** (`events/axis_event.h`) — POD carrying sequence,
timestamp, axisId, state, position snapshot, type, stopReason,
faultCode. Emitted on every notable transition: `StateChanged`,
`MotionStarted/Completed/Stopped`, `LimitActivated`,
`HomingStarted/Completed/Failed`, `FaultRaised`, `FaultCleared`,
`EmergencyStopped`.

**`IAxisEventListener`** — single virtual `onAxisEvent(const AxisEvent&)`.
Up to `MAX_AXIS_EVENT_LISTENERS` (default 4) per axis. Listeners run
from `Axis::service()` (task context), never from ISR.

**`AxisEventQueue<N>`** (`events/axis_event_queue.h`) — ring buffer
of size N. `enqueue()` is ISR-safe (uses `lib_hal::sync::CriticalSection`); `drain()` runs from task. Capacity 32 by default
in `Axis`; events dropped on overflow are counted (`droppedEvents()`)
and reported but do not block.

### Result types

**`Result<T>`** (`result.h`) — `T` or an `ErrorCode`. Move-only-safe
for `T = std::unique_ptr<X>`. Factory: `Result<T>::Ok(value)` /
`Result<T>::Err(code)` (capital letters — collides with `bool ok()`
predicate otherwise on strict compilers).

**`Status`** — `Result<void>` equivalent. Same factories.

Every command in the library returns one of these. No silent failures,
no exceptions, no RTTI.

### TMC2209 driver split

Under `drivers/tmc2209/`. Four orthogonal components sharing one
`ITmcUart` transport. All UART traffic happens off the motion timing
path.

| Component                | What it owns                                                | UART traffic              |
| ------------------------ | ----------------------------------------------------------- | ------------------------- |
| `Tmc2209Configurator`    | Boot config: chopper, microsteps, mA-based currents, GSTAT clear | Writes only, at boot |
| `Tmc2209Diagnostics`     | Opt-in `DRV_STATUS` / `IOIN` / `GSTAT` snapshot              | Off motion path            |
| `Tmc2209StallGuard`      | SGTHRS / TCOOLTHRS; refuses to configure in SpreadCycle      | Writes at boot; opt-in SG_RESULT read |
| `Tmc2209CoolStep`        | SEMIN / SEMAX / SEUP / SEDN / SEIMIN — load-based current scaling | Writes only, at boot |

Stall detection on the wire: the chip pulses DIAG high when
`SG_RESULT < 2 * SGTHRS` during a motion with `TSTEP < TCOOLTHRS`.
The host wires DIAG to a GPIO and registers a `SensorRole::Stall`
sensor on it. The bank's ISR trampoline does the rest — UART is never
consulted during motion.

**Transport split**: `ITmcUart` decouples the four components from
`lib_hal::uart`. `Tmc2209HalUart` is the production adapter (CRC8 +
framing per Trinamic AN001); `FakeTmcUart` is a host-test fake that
models the register file including `IFCNT` auto-increment and
`GSTAT` W1C semantics.

### YPMC servo helper

Under `drivers/ypmc/`. Reuses `Axis::createStepDirServo` for the
pulse path; adds:

- `applyDriveDefaults(StepDirServoAxisConfig&)` — populates the
  S2SVD15's timing (DIR setup, min pulse widths, 500 kpps max) and
  polarity (SRV-ON active-HIGH, ALM− active-LOW).
- `BrakeController` — `IAxisEventListener` that sequences a 24 V
  holding-brake relay against the axis lifecycle. Explicit `release()`
  blocks for the brake's mechanical release time; auto-engage on
  motion-end / fault.

No new actuator / engine — purely a configuration + lifecycle helper.

## Data flows

### `moveBy()` → STEP edges

```
Application
    │  axis->moveBy(1000, DistanceUnit::Mm)
    ▼
Axis::moveBy(delta, unit)
    │  resolveToSteps(delta, unit) ─── uses UnitScaling.stepsPerMm
    │  planner_.planBy(deltaSteps, common_.limits,
    │                  timerResolutionHz_, timerMinTicks_)
    │      → PlannedMove { direction, totalSteps, segments[] }
    │  armAndStart(move, AxisState::Moving)
    │      ├── actuator_->armMotion(move)
    │      │     └── engine_->loadMove(move)   (validates segments)
    │      └── actuator_->startMotion()
    │            └── engine_->start()
    │                  ├── gpio::write(dirPin, ...)
    │                  ├── delayUs(dirSetupUs)   (busy-wait, task ctx)
    │                  ├── running_.store(true)
    │                  └── timer_->startOneShotTicks(firstHp)
    ▼
Hardware timer fires alarms in ISR
    ▼
HalPulseEngine::handleAlarmIsr()                  ← UNGULA_ISR_ATTR
    │  Toggle STEP pin (atomic flip)
    │  Advance currentSegment_, stepsInSegment_
    │  On segment boundary → re-arm timer with next half-period
    │  On final edge → finishedReason_ = TargetReached; running_ = false
    ▼
Hardware emits STEP edges on the wire to the driver chip.
```

`Axis::service(nowMs)` (called from the main loop, 1–10 ms cadence)
runs `pumpMotionState` which polls `engine_->status()`. When it sees
`running == false`, the Axis transitions to `Idle`, emits
`MotionCompleted`, and dispatches listeners.

### DIAG edge → `Faulted`

```
TMC2209 detects stall → drives DIAG HIGH
    ▼
ESP-IDF GPIO ISR fires for the configured pin
    ▼
SensorBank::onIsrTrampoline(IsrBinding*)          ← UNGULA_ISR_ATTR
    │  stallHitCounter_++
    │  stallHitsTotal_++
    │  Check armedAt (atomic load)
    │  Inside arm window? → return (no halt)
    │  Already latched?   → return (no re-halt)
    │  count < threshold? → return (accumulate)
    │  engine.haltFromIsr(StopReason::StallDetected)  ← also IRAM
    │  stallLatched_ = true
    │  stallHitCounter_ = 0
    ▼
HalPulseEngine::haltFromIsr(reason)               ← UNGULA_ISR_ATTR
    │  timer_->disarmFromIsr()
    │  running_ = false
    │  faulted_ = true
    │  finishedReason_ = StallDetected
    ▼
Next Axis::service(nowMs)
    │  pumpSensors():
    │     consumeStallActivation() == true
    │     → state_ = Faulted
    │     → lastStopReason_ = StallDetected
    │     → emitEvent(LimitActivated, FaultRaised, StateChanged)
    │
    │  pumpMotionState() — early-exit because motionInFlight_=false
    │
    │  events_.drain() → user's listeners see FaultRaised.
    ▼
Application loop sees state == Faulted, runs its recovery
(`clearFault → enable → flip direction → jog`).
```

### `service()` tick

`Axis::service(nowMs)` is the single periodic supervisory call. It
runs in task context at 1–10 ms cadence. One call does, in order:

1. **`pumpSensors(nowMs)`**
   - `sensors_.service(nowMs)` — polls debounced sensors, applies
     debounce filter, runs the stall debounce-counter cleanup.
   - Consumes ISR latches in precedence order: E-stop > crash > stall.
     Only the highest-priority latch in the same tick writes
     `state_` / `lastStopReason_`.
   - Halts in-flight motion on a TravelLimit-in-current-direction
     activation. Soft halt; state goes to `Idle`, not `Faulted`.
   - Stops in-flight homing motion on a Home-during-homing activation.

2. **`pumpMotionState()`**
   - Reads `engine_->status()`. If `running` is still true, returns.
   - Otherwise: clears `motionInFlight_`, calls
     `sensors_.notifyMotionEnd()`, then either raises a fault
     (`PulseEngineFault` for engine-internal halts) or emits the
     normal `MotionCompleted` / `MotionStopped` events.
   - Applies the built-in `IdlePolicy::AutoDisable` if configured.

3. **Homing tick**
   - If `homing_.isActive()`, calls `homing_.tick(*this, nowMs)`.
   - Drives the strategy through its FSM. On Complete / Failed,
     emits `HomingCompleted` / `HomingFailed` and returns to `Idle`.

4. **`events_.drain()`** — dispatch queued events to subscribed
   listeners. **All listener callbacks happen here.** Never from ISR.

If `service()` is delayed (host loop blocks for hundreds of ms), the
engine keeps stepping autonomously — only event delivery and
limit-switch reactions are delayed. CrashLimit / EmergencyStop still
halt the engine instantly via their GPIO ISR.

## ISR vs. task context

| Layer / class                       | Context     | Notes                                                    |
| ----------------------------------- | ----------- | -------------------------------------------------------- |
| `HalPulseEngine::handleAlarmIsr`    | ISR         | Toggles STEP, advances segments, re-arms timer           |
| `HalPulseEngine::haltFromIsr`       | ISR         | Disarms timer; atomic state write                        |
| `SensorBank::onIsrTrampoline`       | ISR         | Counts edges, gates stall, calls `haltFromIsr`           |
| `AxisEventQueue::enqueue`           | ISR-safe    | Uses `sync::CriticalSection`; emitters call from any ctx |
| `MotionPlanner::*`                  | Task        | Pure compute, no I/O                                     |
| `Tmc2209*::*`                       | Task        | UART reads/writes; ~1.5 ms at 115200 baud                |
| `Axis::service`                     | Task        | Drives sensors, motion, homing, listener drain           |
| `Axis::moveBy` / `jog` / `stop`     | Task        | Plans + arms + starts                                    |
| `IAxisEventListener::onAxisEvent`   | Task        | Called from `Axis::service` listener drain               |
| `IHomingStrategy::begin/step/finish`| Task        | Called from `HomingController::tick` in service tick     |

**Hard rules — violations crash:**

- No `Axis::service` from ISR — calls into the planner, sensors,
  homing controller, event drain (all task APIs).
- No `IPulseEngine::loadMove` / `start` / `stop` / `clearFault` from
  ISR. Use `haltFromIsr` only.
- No UART reads from the service tick (~1.5 ms blocking — runs them
  from a dedicated lower-priority task or on-demand).
- No event listeners doing slow work — they run from
  `Axis::service()`, and a slow listener delays the next motion-end
  detection.

## Allocation discipline

Heap allocation happens ONLY in the factories:
`Axis::createStepDirStepper` / `createStepDirServo` / `createCanServo`
/ `createComposed`. They `new` the timer, engine, and actuator and
hand back a `std::unique_ptr<Axis>` that owns the lot.

After `begin()`, no allocator is touched on the motion path. Every
runtime data structure is fixed-size: sensor configs in a fixed array,
event queue is a ring buffer, planned-move segments are a fixed array.

This makes the library safe for FreeRTOS / bare-metal builds with
small heaps, and ensures motion timing isn't perturbed by an
allocator pause.

## Test seams

Two host-only seams exist for tests, both clearly named:

- **`SensorBank::simulateIsrEdgeForTesting(SensorRole)`** /
  **`SensorBank::isStallArmed()`** — synthesise an ISR edge from
  task context, query the arm flag. Used by `test_sensor_bank` to
  exercise the stall pipeline without a real GPIO interrupt. Marked
  test-only in their docstrings.
- **`Axis::simulateSensorIsrForTesting(SensorRole)`** — Axis-level
  passthrough to the bank seam.

On the host, `lib_hal::gpio::detail::lastInputMode(pin)` records the
last `configInput*` call applied per pin — lets host tests verify
the lib applied the correct pull-mode for the configured polarity.

These exist for **tests only**. Production code calls the real ISR
path via `lib_hal::gpio::addIsrHandler`.

## Easy-setup kits

A "kit" is a single-call factory that bundles every component a real
project needs for one motor:

- `tmc2209::StepperKit` / `makeStepperKit(cfg)` /
  `makeStepperKitOnUart(uart&, slaveAddress, cfg)` —
  HAL UART (or shared), `Tmc2209HalUart`, `Tmc2209Configurator`,
  optional `Tmc2209StallGuard` + `Tmc2209CoolStep`, and the underlying
  `Axis`.
- `ypmc::YpmcServoKit` / `makeServoKit(cfg)` —
  `Axis::createStepDirServo` + S2SVD15 defaults + optional
  `BrakeController` subscribed to the axis event bus.

Kits own everything they construct in `unique_ptr` members — no
file-scope or singleton state. Multi-motor projects construct N kits;
TMC kits can share one UART by way of `makeStepperKitOnUart` (the
caller pre-`begin()`s the UART and keeps it alive at least as long as
every kit using it).

Kits are syntactic sugar over the compose-by-hand path; the underlying
classes remain reachable through the kit's members for runtime
inspection (e.g. `kit->stallGuard->setSgThreshold(x)`).

### Tandem STEP/DIR wiring

Two drives sharing one STEP pin (each with its own DIR + EN) are
configured by setting the `secondary*` fields on
`StepDirStepperAxisConfig` / `StepDirServoAxisConfig` (or the kit
configs that wrap them):

- `secondaryDirPin` / `secondaryDirActiveHigh` /
  `secondaryDirInverted` — the pulse engine writes both DIRs
  atomically inside the same `dirSetupUs` window so the second
  drive gets the same setup-time guarantee as the primary.
- `secondaryEnablePin` / `secondaryEnableActiveLow` — `enable()` /
  `disable()` flip both pins.
- `secondaryDirInverted = true` handles the face-to-face mounting
  case (one motor must rotate the OPPOSITE electrical direction to
  produce the SAME physical rotation as the primary).

This is one axis from the planner's perspective: one trajectory, one
event stream, one feedback record. The "second motor" is purely a
GPIO multiplexing concern at the engine + actuator layers.

## Extending the library

Adding a new driver (TMC family, industrial servo, etc.): follow the
authoring guide at
[`src/ungula/motor/drivers/README.md`](src/ungula/motor/drivers/README.md).
TL;DR — STEP/DIR drives never need a new actuator or pulse engine;
they need a vendor-specific configuration / diagnostics module under
`drivers/<vendor_model>/` and at most a host-side helper that fills a
`StepDirServoAxisConfig`. Bus-native drives (CAN, RS-485) implement
`ICanServoProtocol` (or analog) and feed it to `CanServoActuator`.

For drivers that warrant a kit (most do — TMC families, industrial
servos with brakes), add a `<vendor>_kit.{h,cpp}` alongside the other
files under `drivers/<vendor>/`. The kit factory should accept a
single `Config` struct, optionally accept a pre-`begin()`-ed UART for
shared-bus projects, and return a `unique_ptr<Kit>` that owns the
axis + every helper it constructs.

Adding a new homing pattern: implement `IHomingStrategy` and hand the
instance to `Axis::setHomingStrategy`. The controller drives the
familiar FSM; your strategy just decides what to command at each
phase.

Adding a new event type: add the enum value to `AxisEventType`,
the `axisEventTypeToString` arm, and emit it from `Axis` where the
transition fires. Subscribers see it through the same `onAxisEvent`.

## Cross-references

- [`README.md`](README.md) — user-facing guide and quick starts.
- [`API.md`](API.md) — public API reference, per-class.
- [`src/ungula/motor/drivers/README.md`](src/ungula/motor/drivers/README.md)
  — driver authoring guide.
- [`../docs/LIBRARY_VERSIONS.md`](../docs/LIBRARY_VERSIONS.md) —
  workspace-wide version + changelog for every library.
