# UngulaMotor (`lib_motor`)

LLM-oriented public-API reference for the motor library. For the
human-facing overview see [`README.md`](README.md). For project-wide
rules see `code/CLAUDE.md`.

The library is built around `ungula::motor::Axis` — one facade for
open-loop steppers, STEP/DIR servos, and CAN servos. The wire
protocol differs; the application code does not. Pulse generation
lives inside a hardware-timer ISR; every other component (planner,
actuator, sensors, homing, events) runs in task context and never
touches step timing.

All public commands return `Result<T>` or `Status`. There are no
silent failures. Stop modes are not silently substituted — `stop(Decelerate)`
returns `Unsupported` rather than degrading to `Immediate`.

---

## Headers

### Umbrella

Pulls in the entire public API. Use this if you don't care about
build-dependency minimisation.

```cpp
#include <ungula/motor.h>
```

### Focused headers

```cpp
#include <ungula/motor/result.h>              // Result<T>, Status, ErrorCode
#include <ungula/motor/axis_types.h>          // Direction, StopMode, Distance, Position, TrajectoryLimits, UnitScaling
#include <ungula/motor/axis_state.h>          // AxisState, StopReason, FaultCode, FaultStatus, IdlePolicy
#include <ungula/motor/axis_config.h>         // StepDirStepperAxisConfig, StepDirServoAxisConfig, CanServoAxisConfig
#include <ungula/motor/motion_units.h>        // Speed, Acceleration, SpeedUnit, AccelerationUnit, apply* helpers
#include <ungula/motor/axis.h>                // Axis facade + factories
#include <ungula/motor/events/axis_event.h>   // AxisEvent, AxisEventType, IAxisEventListener
#include <ungula/motor/planning/motion_planner.h>
#include <ungula/motor/planning/planned_move.h>
#include <ungula/motor/limits/sensor_input.h>
#include <ungula/motor/limits/sensor_bank.h>
#include <ungula/motor/homing/i_homing_strategy.h>
#include <ungula/motor/homing/i_homing_axis.h>
#include <ungula/motor/homing/homing_controller.h>
#include <ungula/motor/homing/limit_switch_homing_strategy.h>
#include <ungula/motor/homing/stall_homing_strategy.h>

// Actuators and pulse engines — needed when composing manually.
#include <ungula/motor/actuator/i_axis_actuator.h>
#include <ungula/motor/actuator/step_dir_actuator.h>
#include <ungula/motor/actuator/can_servo_actuator.h>
#include <ungula/motor/actuator/i_can_servo_protocol.h>
#include <ungula/motor/pulse/i_pulse_engine.h>
#include <ungula/motor/pulse/hal_pulse_engine.h>
#include <ungula/motor/pulse/fake_pulse_engine.h>      // header-only test fake

// TMC2209 driver components (only when configuring a TMC2209 chip).
#include <ungula/motor/drivers/tmc2209/i_tmc_uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>
#include <ungula/motor/drivers/tmc2209/fake_tmc_uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_registers.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_stallguard.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_coolstep.h>
```

---

## Result types

```cpp
#include <ungula/motor/result.h>
```

`Result<T>` and `Status` (`= Result<void>` equivalent) are the canonical
return types. Construction:

- `Status::Ok()` / `Status::Err(code)` (capital — see note below)
- `Result<T>::Ok(value)` / `Result<T>::Err(code)`

The static factories use capital `Ok` / `Err` so they don't clash
with the predicate `bool ok() const`. C++ forbids a static and a
non-static member with the same name in the same class; GCC enforces
it strictly.

Inspection:

- `r.ok()` / `bool(r)` — true iff success
- `r.error()` — the `ErrorCode` on failure (or `Ok`)
- `r.value()` — reference to the value (only valid when `ok()`)
- `r.takeValue()` — moves the value out; designed for `unique_ptr` payloads
- `r.valueOr(fallback)` — copyable-T convenience; compile error for move-only T

`Result<T>` is move-only when `T` is move-only (`std::unique_ptr<Axis>`).

```cpp
#include <ungula/motor/result.h>
#include <ungula/motor/axis.h>

using namespace ungula::motor;

auto r = Axis::createStepDirStepper(cfg);
if (!r.ok()) {
    // r.error() is one of: InvalidConfig, Unsupported, InternalError, ...
    return;
}
std::unique_ptr<Axis> axis = r.takeValue();
```

`ErrorCode` values are listed in `result.h`; convert with
`errorToString(code)`.

---

## Axis facade

```cpp
#include <ungula/motor/axis.h>
```

`ungula::motor::Axis` is the single user-facing type. It inherits
`IHomingAxis` — the strategy-facing narrowed view — but applications
should treat it as a single facade.

### Use case: open-loop stepper

```cpp
#include <ungula/motor/axis.h>
#include <ungula/motor/axis_config.h>

using namespace ungula::motor;

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

auto r = Axis::createStepDirStepper(cfg);
std::unique_ptr<Axis> axis = r.takeValue();
axis->begin();
axis->enable();
axis->moveBy(3200);
```

When to use this: any open-loop stepper driver — TMC2209, A4988, DRV8825.

### Use case: STEP/DIR servo

```cpp
#include <ungula/motor/axis.h>
#include <ungula/motor/axis_config.h>

using namespace ungula::motor;

StepDirServoAxisConfig cfg;
cfg.common.axisId                = AxisId(0);
cfg.common.units.stepsPerMm      = 1000.0f;
cfg.common.limits.maxVelocitySps = 100000;
cfg.common.limits.accelSpsPerSec = 500000;
cfg.common.limits.decelSpsPerSec = 500000;
cfg.common.limits.maxStepRateSps = 500000;
cfg.stepPin            = StepPin{18};
cfg.dirPin             = DirectionPin{19};
cfg.enablePin          = EnablePin{21};         // SVON
cfg.alarmInputPin      = InputPin{34};
cfg.inPositionInputPin = InputPin{35};
cfg.enableActiveLow    = false;                 // SRV-ON is active-HIGH
cfg.dirSetupUs         = 10;

auto axis = Axis::createStepDirServo(cfg).takeValue();
```

When to use this: industrial servo drives that accept STEP/DIR (Yaskawa, Delta, Leadshine).

### Use case: CAN servo (skeleton in 3.0.0)

```cpp
#include <ungula/motor/axis.h>
#include <ungula/motor/axis_config.h>

using namespace ungula::motor;

CanServoAxisConfig cfg;
cfg.common.axisId   = AxisId(0);
cfg.canNodeId       = 1;
cfg.canControllerNumber = 0;

auto r = Axis::createCanServo(cfg);
// r.error() == ErrorCode::Unsupported in 3.0.0 — a concrete
// ICanServoProtocol implementation is required before this path
// becomes functional.
```

When to use this: when a concrete protocol (CiA-402, vendor-specific) is wired into the `CanServoActuator` directly. The 3.0.0 release ships the actuator skeleton and the `ICanServoProtocol` interface.

### Use case: composing manually (tests, custom backends)

```cpp
#include <memory>

#include <ungula/motor/axis.h>
#include <ungula/motor/actuator/step_dir_actuator.h>
#include <ungula/motor/pulse/fake_pulse_engine.h>

using namespace ungula::motor;

auto engine = std::make_unique<FakePulseEngine>();
StepDirActuator::Config aCfg;
aCfg.kind = StepDirActuatorKind::OpenLoopStepper;
auto actuator = std::make_unique<StepDirActuator>(*engine, aCfg);

Axis::ComposedComponents comp;
comp.timer    = nullptr;       // fake engine doesn't need a timer
comp.engine   = std::move(engine);
comp.actuator = std::move(actuator);
comp.common.limits.maxVelocitySps = 4000;
comp.common.limits.accelSpsPerSec = 8000;
comp.common.limits.decelSpsPerSec = 8000;
comp.common.limits.maxStepRateSps = 200000;

auto r = Axis::createComposed(std::move(comp));
auto axis = r.takeValue();
```

When to use this: host integration tests, custom timer backends, or alternative actuator implementations. Production code should reach for the three named factories.

### Public methods

| Group | Method | Returns |
| --- | --- | --- |
| **Lifecycle** | `begin()` | `Status` |
| | `enable()` | `Status` |
| | `disable()` | `Status` |
| **Motion** | `moveTo(target, unit = Steps)` | `Status` |
| | `moveBy(delta, unit = Steps)` | `Status` |
| | `jog(direction)` | `Status` |
| | `jog(direction, Speed)` | `Status` (per-jog feed override) |
| | `stop(mode = Immediate)` | `Status` (`Decelerate` → `Unsupported`) |
| | `emergencyStop()` | `Status` |
| **Trajectory tuning** | `setMaxVelocity(Speed)` | `Status` (live during jog; next-motion during in-flight `moveTo`/`moveBy`) |
| | `setAcceleration(Acceleration)` | `Status` (next-motion only) |
| | `setDeceleration(Acceleration)` | `Status` (next-motion only) |
| | `setRampProfile(Acceleration)` | `Status` (accel == decel) |
| **Homing** | `setHomingStrategy(strategy, timeoutMs = 0)` | `Status` |
| | `home()` | `Status` |
| | `isHoming()` / `isHomed()` | `bool` |
| | `homingPhase()` | `HomingPhase` |
| **Idle** | `setDefaultIdlePolicy(IdlePolicy)` | `void` |
| | `defaultIdlePolicy()` | `IdlePolicy` |
| **Faults** | `clearFault()` | `Status` |
| | `faultStatus()` | `FaultStatus` |
| **Queries** | `state()` | `AxisState` |
| | `feedback()` | `AxisFeedback` |
| | `lastStopReason()` | `StopReason` |
| | `id()` | `AxisId` |
| **Service** | `service(nowMs)` | `void` |
| **Events** | `subscribe(listener)` | `Status` |
| | `serviceEvents()` | `uint32_t` events drained |

`service(nowMs)` must run from a task tick (1 ms typical). It drives sensors, observes the engine, ticks the homing controller, and drains events.

### State semantics

```cpp
#include <ungula/motor/axis_state.h>

enum class AxisState : uint8_t {
    Uninitialized, Disabled, Idle, Moving, Jogging,
    Homing, Stopping, Faulted, EmergencyStopped,
};
```

Transitions are explicit — no auto-clearing terminal states. `clearFault()` returns from `Faulted` / `EmergencyStopped` to **`Disabled`** (not `Idle`). The host must explicitly `enable()` again before commanding motion — eliminates the "motor jumped after clearFault" failure mode.

---

## User-unit Speed and Acceleration

```cpp
#include <ungula/motor/motion_units.h>
```

`Speed` and `Acceleration` are tiny value-types pairing a magnitude with
a unit enum. Resolved to internal `Velocity` (steps/sec) and
steps/sec² against `UnitScaling` (`stepsPerMm` for linear units,
`stepsPerDegree` for rotary). Magnitudes are non-negative — `Direction`
expresses sign.

```cpp
struct Speed         { float value; SpeedUnit         unit; };
struct Acceleration  { float value; AccelerationUnit  unit; };
```

Named factories:

```cpp
Speed::stepsPerSec(...) / mmPerSec / mmPerMin / cmPerSec / cmPerMin /
inchesPerSec / inchesPerMin / degreesPerSec / rpm / rps;

Acceleration::stepsPerSecSquared(...) / mmPerSecSquared / cmPerSecSquared /
inchesPerSecSquared / degreesPerSecSquared / rpmPerSec / rpsPerSec;
```

Resolvers — return `InvalidConfig` if the unit needs a scaling field that
is zero, or if the magnitude is negative / overflows the target integer:

```cpp
Result<Velocity> toStepsPerSec       (Speed,        const UnitScaling&);
Result<uint32_t> toStepsPerSecSquared(Acceleration, const UnitScaling&);
```

Setup-time helpers for the user's `TrajectoryLimits` (does not need the
axis instance):

```cpp
Status applyMaxVelocity (TrajectoryLimits&, Speed,        const UnitScaling&);
Status applyAcceleration(TrajectoryLimits&, Acceleration, const UnitScaling&);
Status applyDeceleration(TrajectoryLimits&, Acceleration, const UnitScaling&);
Status applyRampProfile (TrajectoryLimits&, Acceleration, const UnitScaling&);  // accel == decel
```

### Use case: configure in mm/s and rpm

```cpp
StepDirStepperAxisConfig cfg;
cfg.common.units.stepsPerMm     = 80.0f;
cfg.common.units.stepsPerDegree = 8.889f;

applyMaxVelocity (cfg.common.limits, Speed::mmPerSec(150.0f),               cfg.common.units);
applyAcceleration(cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);
applyDeceleration(cfg.common.limits, Acceleration::mmPerSecSquared(500.0f), cfg.common.units);

auto axis = Axis::createStepDirStepper(cfg).takeValue();
axis->begin();
axis->enable();
axis->jog(Direction::Forward, Speed::rpm(60.0f));     // per-jog feed override
```

### Live tuning semantics

The runtime setters intentionally have different semantics by mutation
kind:

| Setter | While `Jogging` | While `Moving` (moveTo/moveBy in flight) | While `Idle` |
| --- | --- | --- | --- |
| `setMaxVelocity` | Stops + re-arms the jog at the new feed (brief glitch, no fault) | Locked — in-flight segments already precomputed. Applies on the next motion. | Applies to the next motion |
| `setAcceleration` | Applies to the next motion | Applies to the next motion | Applies to the next motion |
| `setDeceleration` | Applies to the next motion | Applies to the next motion | Applies to the next motion |
| `setRampProfile` | Applies to the next motion | Applies to the next motion | Applies to the next motion |

Accel/decel are next-motion-only because mid-flight ramp changes would
require splicing fresh ramps into the segment queue — the engine doesn't
support that today and silently degrading would be a safety bug.

---

## Idle policy

```cpp
#include <ungula/motor/axis_state.h>

enum class IdlePolicy : uint8_t {
    HoldCurrent = 0,    // default — keep enabled; chip falls to IHOLD after iHoldDelay
    AutoDisable = 1,    // drop EN at every motion end; coils freewheel
};

void       Axis::setDefaultIdlePolicy(IdlePolicy);
IdlePolicy Axis::defaultIdlePolicy() const;
```

The policy is implemented inside the Axis as a built-in event listener
that observes `MotionCompleted` / `MotionStopped`. It coexists with any
custom `IAxisEventListener` you subscribe.

### Use case: built-in `AutoDisable`

```cpp
axis->setDefaultIdlePolicy(IdlePolicy::AutoDisable);
// next moveBy/moveTo/jog completes → axis automatically calls disable()
// The host must explicitly enable() again before the next motion.
```

### Use case: equivalent custom listener (`AutoDisableOnIdle`)

This is the user-extensible shape behind the built-in policy. Reach for
this when you want to do MORE than just disable at idle (signal a
pillar, kick a brake solenoid, log the cycle, ...).

```cpp
#include <ungula/motor/axis.h>
#include <ungula/motor/events/axis_event.h>

using namespace ungula::motor;

class AutoDisableOnIdle final : public IAxisEventListener {
public:
    explicit AutoDisableOnIdle(Axis& axis) : axis_(axis) {}

    void onAxisEvent(const AxisEvent& ev) override {
        switch (ev.type) {
        case AxisEventType::MotionCompleted:
        case AxisEventType::MotionStopped:
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

If you use a custom listener that already drops the EN pin, keep the
built-in policy at `HoldCurrent` — otherwise `disable()` is called twice
per motion end. Harmless on the chip side (idempotent) but the second
call's `Status` will surface a state-machine warning.

`AutoDisable` semantics:

- The host must call `enable()` again before each motion command. There
  is no automatic re-enable — same shape as `clearFault()` returning to
  `Disabled` rather than `Idle`.
- Coils freewheel at rest. The carrier is held only by friction or a
  mechanical brake. Don't use this on direct-drive vertical axes that
  rely on holding torque.
- Power dissipation drops to zero at rest. Right for thermally-bound
  small steppers idling for long periods.

---

## Motion planning

```cpp
#include <ungula/motor/planning/motion_planner.h>
#include <ungula/motor/planning/planned_move.h>
```

`MotionPlanner` is stateless. The same instance can plan many moves; nothing carries between calls. The Axis owns one internally; hosts rarely instantiate it directly.

### Use case: plan-by-hand (advanced)

```cpp
#include <ungula/motor/planning/motion_planner.h>
#include <ungula/motor/axis_types.h>

using namespace ungula::motor;

MotionPlanner planner;
TrajectoryLimits L;
L.maxVelocitySps = 4000;
L.accelSpsPerSec = 8000;
L.decelSpsPerSec = 8000;
L.maxStepRateSps = 200000;

const auto move = planner.planBy(/*deltaSteps=*/1000, L,
                                  /*resolutionHz=*/1'000'000,
                                  /*minHalfPeriodTicks=*/5);

// move.totalSteps == 1000 (exact — the planner rebalances rounding
// drift into the cruise segment for trapezoidal profiles or the
// last decel segment for triangular).
```

Exact step-count guarantee: `sum(move.segments[i].stepCount) == |deltaSteps|` for every successful plan. The user's commanded distance is non-negotiable.

### Planner methods

| Method | Use |
| --- | --- |
| `planBy(deltaSteps, limits, resolutionHz, minHalfPeriodTicks)` | Relative move; direction from delta sign |
| `planTo(currentPos, targetPos, limits, ...)` | Absolute move (thin wrapper) |
| `planJog(direction, maxSteps, limits, ...)` | Long-running trapezoidal jog with a safety bound |
| `planStop(direction, currentSps, limits, ...)` | Decel-only ramp from `currentSps` to 0 |

All return `PlannedMove`. Empty (`totalSteps == 0`) on zero-delta inputs or pathological limits.

---

## Pulse engine

```cpp
#include <ungula/motor/pulse/i_pulse_engine.h>
#include <ungula/motor/pulse/hal_pulse_engine.h>      // production
#include <ungula/motor/pulse/fake_pulse_engine.h>     // tests
```

`IPulseEngine` is the abstract pulse generator. The production
implementation `HalPulseEngine` drives an `ungula::hal::timer::IHwTimer`
via its alarm ISR. The test `FakePulseEngine` is header-only and
deterministic.

### Use case: building a production engine by hand

```cpp
#include <memory>

#include <ungula/hal/timer/drivers/hwtimer.h>
#include <ungula/motor/pulse/hal_pulse_engine.h>

using namespace ungula::motor;

auto timer = std::make_unique<ungula::hal::timer::drivers::HwTimer>();

HalPulseEngine::Config eCfg;
eCfg.stepPin           = 18;
eCfg.dirPin            = 19;
eCfg.dirActiveHigh     = true;
eCfg.dirSetupUs        = 5;
eCfg.timerResolutionHz = 1'000'000;
eCfg.timerMinTicks     = 5;

HalPulseEngine engine(*timer, eCfg);
```

The `Axis` factories assemble this combination internally — most callers don't write it out by hand.

### Use case: ISR-context immediate halt

`haltFromIsr(reason)` is the ISR-safe halt entry. It's wired into the
`SensorBank` ISR trampolines for CrashLimit and EmergencyStop sensors:
those GPIO ISRs call straight into the engine to halt the timer, with
no UART, no logging, no task wakeup.

```cpp
#include <ungula/motor/pulse/hal_pulse_engine.h>
#include <ungula/motor/axis_state.h>

// from inside a GPIO ISR (see SensorBank for the production wiring):
engine.haltFromIsr(ungula::motor::StopReason::EmergencyStop);
```

Application code typically doesn't call this directly — the SensorBank handles ISR wiring.

### Engine methods

| Method | Context | Notes |
| --- | --- | --- |
| `begin(PulseMode::Internal)` | Task | `External` mode is reserved for future host-driven ticking |
| `loadMove(PlannedMove)` | Task | Allowed only while not running |
| `start()` | Task | Writes DIR, waits `dirSetupUs`, arms the timer |
| `stop(StopMode)` | Task | `Decelerate` → `Unsupported`; `Immediate` / `Emergency` halt now |
| `emergencyStop()` | Task | `stop(Emergency)` + latches `faulted_` |
| `haltFromIsr(reason)` | **ISR** | Timer disarm + atomic state set; sub-µs |
| `isRunning()` / `status()` | Any | Atomic reads |
| `commandedPositionSteps()` | Any | Atomic read |
| `resetPosition(steps)` | Task | Allowed only while not running |
| `clearFault()` | Task | Returns `MotionInProgress` if running |

---

## Actuators

```cpp
#include <ungula/motor/actuator/i_axis_actuator.h>
#include <ungula/motor/actuator/step_dir_actuator.h>
#include <ungula/motor/actuator/can_servo_actuator.h>
```

### `StepDirActuator`

One class for both flavours of STEP/DIR drive. The wire protocol is identical; `StepDirActuatorKind` only changes feedback semantics and capability flags.

```cpp
#include <ungula/motor/actuator/step_dir_actuator.h>
#include <ungula/motor/pulse/hal_pulse_engine.h>

using namespace ungula::motor;

StepDirActuator::Config a;
a.kind            = StepDirActuatorKind::OpenLoopStepper;   // or StepDirServo
a.enablePin       = 21;                                     // GPIO_NONE if none
a.enableActiveLow = true;                                   // false for industrial servos
a.hasAlarmInput      = false;                               // surfaced in capabilities()
a.hasInPositionInput = false;
StepDirActuator actuator(engine, a);
```

**Direction-timing contract:** the actuator MUST NOT touch the DIR pin. The pulse engine owns DIR — it writes DIR in `start()` and waits `dirSetupUs` before arming the timer. Touching DIR from the actuator violates driver setup-time guarantees.

### `CanServoActuator` (skeleton)

```cpp
#include <ungula/motor/actuator/can_servo_actuator.h>
#include <ungula/motor/actuator/i_can_servo_protocol.h>
#include <ungula/hal/can/can.h>

using namespace ungula::motor;

ungula::hal::can::Can bus(0);
ICanServoProtocol* protocol = nullptr;   // your concrete impl goes here

CanServoActuator::Config c;
c.nodeId             = 1;
c.hasAlarmInput      = true;
c.hasInPositionInput = true;
c.hasActualPosition  = true;
c.hasNativeHoming    = true;

CanServoActuator actuator(bus, protocol, c);
```

In 3.0.0 the actuator returns `ErrorCode::Unsupported` from motion methods when `protocol == nullptr`. Lifecycle methods (`begin`/`enable`/`disable`/`clearFault`) and feedback queries forward to the protocol when set.

---

## Sensor inputs

```cpp
#include <ungula/motor/limits/sensor_input.h>
#include <ungula/motor/limits/sensor_bank.h>
```

### `SensorRole`

| Role | Path | Use for |
| --- | --- | --- |
| `Home` | Polled, debounced | Reference / homing only |
| `TravelLimit` | Polled, debounced | Soft limits — service-tick reaction |
| `CrashLimit` | GPIO ISR → `engine.haltFromIsr()` | Hard limits — sub-µs halt |
| `EmergencyStop` | GPIO ISR → `engine.haltFromIsr()` + latch | E-stop circuits |
| `Stall` | GPIO ISR with hit-count + arm-delay debounce | TMC2209 DIAG. Reports `StopReason::StallDetected` + `FaultCode::Stall`. During an active homing cycle the Axis treats the stall as a success signal (soft-stop, no fault). |

### Stall-sensor debounce knobs (role == `Stall` only)

```cpp
SensorInputConfig s;
s.role               = SensorRole::Stall;
s.stallHitsToTrigger = 4;     // ISR edges required before latching a stall
s.stallArmDelayMs    = 200;   // ms after motion start during which edges are ignored
```

The StealthChop auto-tune phase tends to glitch DIAG once or twice in the
first ~100 ms of motion. `stallHitsToTrigger` rejects a single spurious
pulse; `stallArmDelayMs` ignores the entire startup transient. Real stalls
hold DIAG high — the chip continues to pulse it as long as `SG_RESULT <
2*SGTHRS`, so several hits in a single service tick is the normal case.

### Use case: a home reference + a hardware E-stop

```cpp
#include <ungula/motor/axis_config.h>
#include <ungula/motor/limits/sensor_input.h>

using namespace ungula::motor;

StepDirStepperAxisConfig cfg;
// ... (pins, limits etc.) ...

cfg.sensors[0].pin        = 34;
cfg.sensors[0].role       = SensorRole::Home;
cfg.sensors[0].polarity   = SensorPolarity::NormallyClosed;
cfg.sensors[0].direction  = Direction::Backward;
cfg.sensors[0].debounceMs = 20;

cfg.sensors[1].pin       = 32;
cfg.sensors[1].role      = SensorRole::EmergencyStop;
cfg.sensors[1].polarity  = SensorPolarity::NormallyClosed;

cfg.sensorCount = 2;
```

The Axis copies the sensor configs out of the user's config and brings up the `SensorBank` in `begin()`. The `MAX_SENSOR_INPUTS` cap is 6 (home + 2 travel + crash + estop + spare).

### Use case: querying sensor state

```cpp
#include <ungula/motor/limits/sensor_bank.h>

// SensorBank methods (called by Axis::service internally; advanced
// hosts can compose their own service path):
bool homeActive = bank.isActive(SensorRole::Home);
bool fwdLimit   = bank.isActive(SensorRole::TravelLimit, Direction::Forward);

// ISR latches — return true exactly once per event.
if (bank.consumeCrashActivation()) {
    // First service tick after a crash-limit ISR
}
```

---

## Homing

```cpp
#include <ungula/motor/homing/homing_controller.h>
#include <ungula/motor/homing/limit_switch_homing_strategy.h>
```

### Phases

```cpp
enum class HomingPhase : uint8_t {
    Idle,
    FastApproach,
    Backoff,
    SlowApproach,
    SetHomePosition,
    Complete,
    Failed,
};
```

No auto-clearing terminal states — `Complete` and `Failed` stay until the next `home()` call.

### Use case: limit-switch homing

```cpp
#include <ungula/motor/axis.h>
#include <ungula/motor/homing/limit_switch_homing_strategy.h>

using namespace ungula::motor;

LimitSwitchHomingStrategy::Config c;
c.approachDirection = Direction::Backward;
c.fastFeedSps       = 2000;
c.slowFeedSps       = 200;
c.backoffSteps      = 200;
c.homePositionSteps = 0;
LimitSwitchHomingStrategy strategy(c);

axis->setHomingStrategy(&strategy, /*timeoutMs=*/30000);
axis->home();                  // returns immediately

while (axis->isHoming()) {
    axis->service(millis());
}

if (axis->isHomed()) {
    axis->moveTo(1600);        // absolute, from the home position
}
```

The strategy must outlive any active homing cycle. Passing `timeoutMs=0` disables the timeout (use carefully — a stuck mechanism will never report Failed).

### Use case: stall-based homing (sensorless)

For axes that home against a mechanical hard-stop instead of a wired
switch. Same FSM shape as `LimitSwitchHomingStrategy` but the strategy
reads `axis.isStallActive()` rather than `axis.isHomeActive()`.

Preconditions:

- A `SensorRole::Stall` sensor wired to the TMC2209 DIAG pin.
- The chip in **StealthChop** mode (StallGuard4 is StealthChop-only).
- `Tmc2209StallGuard::begin()` called with valid SGTHRS / TCOOLTHRS for
  the load. `verifyChopperMode = true` (the default) refuses the config
  if GCONF says SpreadCycle.

```cpp
#include <ungula/motor/homing/stall_homing_strategy.h>

using namespace ungula::motor;

StallHomingStrategy::Config c;
c.approachDirection = Direction::Backward;
c.fastFeedSps       = 1500;
c.slowFeedSps       = 200;
c.backoffSteps      = 200;
c.homePositionSteps = 0;
StallHomingStrategy strategy(c);

axis->setHomingStrategy(&strategy, /*timeoutMs=*/30000);
axis->home();
```

The Axis tracks stall-during-homing internally: a stall is a success
signal for the strategy (soft-stop, no fault), but the same stall
outside a homing cycle raises `FaultCode::Stall` with
`StopReason::StallDetected`.

### Use case: writing a custom strategy

```cpp
#include <ungula/motor/homing/i_homing_strategy.h>
#include <ungula/motor/homing/i_homing_axis.h>
#include <ungula/motor/homing/homing_controller.h>      // HomingPhase

class MyStrategy final : public ungula::motor::IHomingStrategy {
public:
    using HomingProgress = ungula::motor::HomingProgress;
    using HomingPhase    = ungula::motor::HomingPhase;

    ungula::motor::Status begin(ungula::motor::IHomingAxis& axis) override {
        phase_ = HomingPhase::FastApproach;
        return axis.commandJog(ungula::motor::Direction::Backward, /*sps=*/2000);
    }
    HomingProgress step(ungula::motor::IHomingAxis& axis) override {
        if (!axis.isHomeActive()) return HomingProgress::Failed;
        phase_ = HomingPhase::SetHomePosition;
        const auto s = axis.resetPosition(0);
        return s.ok() ? HomingProgress::Succeeded : HomingProgress::Failed;
    }
    void finish(ungula::motor::IHomingAxis&, bool) override {}
    HomingPhase currentPhase() const override { return phase_; }
private:
    HomingPhase phase_ = HomingPhase::Idle;
};
```

`IHomingAxis` is the narrowed view: `commandMove`, `commandJog`, `stopMove`, `isHomeActive`, `isStallActive`, `isMotionIdle`, `resetPosition`. Strategies cannot call `Axis::moveTo` / `home` / `subscribe` etc. — this prevents deadlocks against the public command API. `isStallActive()` is the per-cycle latch the Axis sets when a stall fires during homing; it's cleared on the next `commandMove` / `commandJog`.

---

## Events

```cpp
#include <ungula/motor/events/axis_event.h>
```

### Event payload

```cpp
struct AxisEvent {
    uint32_t      sequence;          // monotonic, per-axis
    int64_t       timestampMs;       // ungula::core::time::millis()
    AxisId        axisId;
    AxisState     state;             // snapshot at emission time
    Position      commandedPosition;
    Position      actualPosition;
    bool          hasActualPosition; // false for open-loop drives
    AxisEventType type;
    StopReason    stopReason;
    FaultCode     faultCode;
};
```

### Use case: subscribing a listener

```cpp
#include <Arduino.h>

#include <ungula/motor/axis.h>
#include <ungula/motor/events/axis_event.h>

using namespace ungula::motor;

class PrintingListener final : public IAxisEventListener {
public:
    void onAxisEvent(const AxisEvent& ev) override {
        Serial.printf("[%lu] %s state=%s pos=%ld\n",
                      static_cast<unsigned long>(ev.timestampMs),
                      axisEventTypeToString(ev.type),
                      axisStateToString(ev.state),
                      static_cast<long>(ev.commandedPosition));
    }
};
PrintingListener listener;

void setup() {
    // ... build and begin axis ...
    axis->subscribe(&listener);
}
```

Listeners are invoked from `Axis::service()` (which calls `serviceEvents()` internally). **Never** from ISR. The queue capacity is fixed at compile time; `enqueue` on a full queue returns `QueueFull` and increments `droppedEvents()` — hosts that can't drain fast enough should run `serviceEvents()` from a higher-priority task.

### Event type reference

| Type | Emitted when |
| --- | --- |
| `StateChanged` | Any `AxisState` transition |
| `MotionStarted` | `moveTo` / `moveBy` / `jog` armed the engine |
| `MotionCompleted` | Engine reported `StopReason::TargetReached` |
| `MotionStopped` | Engine stopped before reaching target |
| `LimitActivated` | Crash / E-stop ISR fired, or travel-limit-in-direction activated |
| `HomingStarted` | `home()` accepted |
| `HomingCompleted` | Controller reached `Complete` |
| `HomingFailed` | Controller reached `Failed` |
| `FaultRaised` | Driver, pulse-engine, or limit/estop fault |
| `FaultCleared` | `clearFault()` succeeded |
| `EmergencyStopped` | `emergencyStop()` or E-stop ISR |

---

## TMC2209 driver

```cpp
#include <ungula/hal/uart/uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_stallguard.h>
```

Four single-responsibility components share one `ITmcUart` transport.

### Use case: boot configuration (mA-based currents)

```cpp
#include <ungula/hal/uart/uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_hal_uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>

namespace tmc = ungula::motor::tmc2209;

ungula::hal::uart::Uart uart(1);
tmc::Tmc2209HalUart     transport(uart, /*slaveAddress=*/0);
tmc::Tmc2209Configurator config(transport);

void setup() {
    uart.begin(115200, /*tx=*/17, /*rx=*/16);

    tmc::Tmc2209Configurator::Config c;
    c.runCurrentMa        = 800;      // mA RMS at the coil
    c.holdCurrentMa       = 300;      // typically 30..50% of runCurrentMa
    c.iHoldDelay          = 1;        // ~21 ms IRUN → IHOLD on standstill
    c.senseResistorOhms   = 0.11f;    // check the silk screen on your carrier
    c.useHighSensitivity  = false;    // vsense bit — true for <300 mA setups
    c.microsteps          = tmc::Microsteps::Sixteenth;
    c.mode                = tmc::ChopperMode::StealthChop;
    c.toff                = 3;
    c.interpolate         = true;
    config.begin(c);             // writes GCONF, CHOPCONF, IHOLD_IRUN, GSTAT
}
```

Validation: `runCurrentMa > 0`, `holdCurrentMa <= runCurrentMa`,
`senseResistorOhms > 0`. Invalid values return `InvalidConfig` before
any UART traffic.

### mA ↔ CS conversion helpers

```cpp
// Both helpers are static — useful for UIs that show mA and CS side by
// side, or for compile-time sanity checks.
const uint8_t  cs   = tmc::Tmc2209Configurator::milliampsToCs(600, 0.11f);
const uint16_t back = tmc::Tmc2209Configurator::csToMilliamps(cs,  0.11f);

// Runtime change — reuses the cached senseResistorOhms / vsense flag
// from the last begin(). One UART write (~700 µs at 115200 baud).
config.setCurrents(/*runMa=*/600, /*holdMa=*/200, /*iHoldDelay=*/1);
```

Conversion formula: `CS = round(I_rms · 32 · √2 · (R_sense + 0.02) / V_fs) - 1`,
clamped to 0..31. `V_fs` is 0.325 V at vsense=0 and 0.180 V at
vsense=1; the 0.02 Ω term is the chip's internal MOSFET resistance.

`Tmc2209Configurator::clearGstat()` writes the W1C mask (real W1C — the original driver had a read bug here). `begin()` never reads from the chip; diagnostics live elsewhere.

### Use case: opt-in diagnostics from a low-priority task

```cpp
#include <ungula/motor/drivers/tmc2209/tmc2209_diagnostics.h>

namespace tmc = ungula::motor::tmc2209;
tmc::Tmc2209Diagnostics diag(transport);

void diagnoseLoop() {
    if (diag.refresh().ok()) {
        const auto& s = diag.snapshot();
        if (s.driveFault())       handleFault();
        if (s.overTemperaturePreWarn()) reduceCurrent();
    }
}
```

`refresh()` performs one UART round-trip per register (DRV_STATUS, IOIN, GSTAT) at ~1.5 ms each at 115200 baud. Run from a non-motion-critical task or on-demand after a fault event. **Never** from the Axis service tick.

### Use case: stall detection via DIAG pin

```cpp
#include <ungula/motor/drivers/tmc2209/tmc2209_stallguard.h>

namespace tmc = ungula::motor::tmc2209;
tmc::Tmc2209StallGuard sg(transport);

tmc::Tmc2209StallGuard::Config sCfg;
sCfg.sgThreshold       = 10;        // SG_RESULT < 2*SGTHRS → stall
sCfg.tCoolThrs         = 0xFFFFF;   // active at any non-zero velocity
sCfg.verifyChopperMode = true;      // refuses to configure in SpreadCycle (default)
sg.begin(sCfg);                     // returns InvalidConfig if chip is in SpreadCycle
```

The chip pulses DIAG high on a stall. Wire DIAG to a `SensorRole::Stall` sensor on the Axis side; the GPIO ISR with hit-count + arm-delay debounce halts the pulse engine and (during a homing cycle) advances the strategy.

`verifyChopperMode = true` (the default) reads GCONF first and refuses
to write SGTHRS / TCOOLTHRS if `EN_SPREADCYCLE` is set — StallGuard4 is
StealthChop-only on the TMC2209. Set this to `false` only if you are
deliberately preparing the registers before flipping back to
StealthChop.

For tuning, an opt-in `readSgResult()` is available — strictly off the motion-timing path.

### Use case: CoolStep (load-based dynamic current)

CoolStep is **independent** from StallGuard. They share `SG_RESULT`
(both read it) and `TCOOLTHRS` (either can write it). Use either alone,
or both together — when running both, set `tCoolThrs` on one and leave
the other at `0` (skip writing) so the second one inherits the first's
value.

```cpp
#include <ungula/motor/drivers/tmc2209/tmc2209_coolstep.h>

namespace tmc = ungula::motor::tmc2209;
tmc::Tmc2209CoolStep cool(transport);

tmc::Tmc2209CoolStep::Config cs;
cs.loadThresholdToIncrease       = 4;   // SEMIN — 1..15 enables, 0 disables
cs.loadThresholdToDecreaseMargin = 2;   // SEMAX — hysteresis width
cs.currentRampUp                 = tmc::Tmc2209CoolStep::StepWidth::Step1;
cs.currentRampDown               = tmc::Tmc2209CoolStep::StepWidth::Step2;
cs.currentFloor                  = tmc::Tmc2209CoolStep::CurrentFloor::Half;  // IRUN/2 floor
cs.tCoolThrs                     = 0;   // leave StallGuard's TCOOLTHRS untouched
cool.begin(cs);

// Disable at runtime — clears SEMIN only. Other fields remain so a
// subsequent begin() doesn't need to repeat them.
cool.disable();
```

`StepWidth` (SEUP / SEDN) is the per-tick current step: `Step1` is
gentle, `Step8` is aggressive. `CurrentFloor` (SEIMIN) caps how low
CoolStep can go at light load — `Half` = IRUN/2, `Quarter` = IRUN/4.

### Custom UART transport (e.g. RS-485 bridge)

```cpp
#include <ungula/motor/drivers/tmc2209/i_tmc_uart.h>

class Rs485TmcUart final : public ungula::motor::tmc2209::ITmcUart {
public:
    ungula::motor::Status writeRegister(uint8_t reg, uint32_t value) override { /* ... */ }
    ungula::motor::Result<uint32_t> readRegister(uint8_t reg) override { /* ... */ }
};
```

The Configurator / Diagnostics / StallGuard components don't care about the wire — they only see `writeRegister(reg, value)` and `readRegister(reg)`.

### Registers and bit constants

```cpp
#include <ungula/motor/drivers/tmc2209/tmc2209_registers.h>

namespace tmc = ungula::motor::tmc2209;
const uint32_t drv = transport.readRegister(tmc::reg::DRV_STATUS).valueOr(0);
if (drv & tmc::drv_status::FAULT_MASK) { /* any drive-fault condition */ }
```

---

## Testing

Header-only fakes are provided for all transport layers.

### `FakePulseEngine`

```cpp
#include <ungula/motor/pulse/fake_pulse_engine.h>

using namespace ungula::motor;

FakePulseEngine engine;
engine.begin(PulseMode::Internal);

PlannedMove move;
move.direction = Direction::Forward;
move.totalSteps = 100;
move.segmentCount = 1;
move.segments[0].stepCount = 100;
move.segments[0].halfPeriodTicks = 100;

engine.loadMove(move);
engine.start();
engine.tickSteps(50);
EXPECT_EQ(engine.commandedPositionSteps(), 50);
engine.runMove();                 // finish "instantly"
EXPECT_FALSE(engine.isRunning());

engine.injectFault();             // simulate an ISR-side fault
```

### `FakeTmcUart`

```cpp
#include <ungula/motor/drivers/tmc2209/fake_tmc_uart.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_configurator.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_registers.h>

namespace tmc = ungula::motor::tmc2209;

tmc::FakeTmcUart uart;
uart.seed(tmc::reg::DRV_STATUS, tmc::drv_status::OT);    // pre-seed registers
uart.failNextWrite = true;                               // fault-injection knob

tmc::Tmc2209Configurator cfg(uart);
auto s = cfg.begin({});
// s.error() == ErrorCode::TransportError because of failNextWrite
```

The fake honours GSTAT W1C and IFCNT auto-increment so configurator regressions are caught.

### Custom `IHomingAxis` fake

`test_homing_controller.cpp` ships a `FakeHomingAxis` that's intentionally local to the test — copy it as a starting point for your own strategy tests. Knobs:

- `homeActive` / `motionIdle` — simulate sensor + motion state.
- `failNextCommand` — make the next command return `InvalidState`.
- Per-method call counters and last-argument captures for assertions.

---

## Threading / ISR rules

Hard rules — violations cause crashes or silent data corruption:

- **No** `Axis::service` from ISR. It calls into the planner, sensors, homing controller, event drain — all task-context APIs.
- **No** `IPulseEngine::loadMove` / `start` / `stop` / `clearFault` from ISR. Use `haltFromIsr` instead.
- **No** UART reads (any TMC2209 method that reads a register) from the Axis service tick. They block for ~1.5 ms at 115200 baud — run them from a lower-priority task.
- **No** event listeners doing slow work. They run from `Axis::service()` and block the next motion-completion detection. Push slow work to your own task.
- ISR-context entries are explicitly annotated: `IPulseEngine::haltFromIsr`, `AxisEventQueue::enqueue` (uses `ungula::hal::sync::ScopedLock`). Everything else is task-context.

---

## Versioning

3.0.0 was the first production release of the refactored architecture.

3.1.0 adds:

- Typed `Speed` / `Acceleration` user-unit value-types and the `apply*`
  helpers; per-jog feed override and live `setMaxVelocity` semantics.
- `IdlePolicy` (`HoldCurrent` / `AutoDisable`) implemented as a built-in
  event listener; coexists with custom `IAxisEventListener` subscriptions.
- `SensorRole::Stall` with hit-count + arm-delay debounce, dedicated
  `StopReason::StallDetected` + `FaultCode::Stall`, and during-homing
  soft-stop routing (stall is a success signal, not a fault, while
  `home()` is active).
- `StallHomingStrategy` — sensorless homing against a mechanical stop.
- `Tmc2209Configurator` mA-based current API + `milliampsToCs` /
  `csToMilliamps` helpers + `useHighSensitivity` (vsense) selection.
- `Tmc2209StallGuard::Config::verifyChopperMode` — refuses to configure
  if the chip is in SpreadCycle.
- `Tmc2209CoolStep` — independent load-based dynamic current scaling.

Future minor versions will fill in the deferred items listed in
[`README.md`](README.md) (concrete `ICanServoProtocol`,
`IndexPulseHomingStrategy`, mid-flight `stop(Decelerate)` re-planning,
`PulseMode::External` host ticking).
