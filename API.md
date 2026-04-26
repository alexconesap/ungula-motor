# UngulaMotor (`lib_motor`)

Stepper motion stack for ESP32. The headline class `LocalMotor` runs a
hardware-timer-driven step generator with acceleration ramps, an 11-state
FSM, debounced limit switches, driver-owned stall detection, an event
publisher, and pluggable homing strategies — all autonomous after
`begin()`. A `RemoteMotor` proxy and a `MotorCoordinator` round out the
multi-axis story. Includes a built-in TMC2209 UART driver.

Depends on `UngulaCore` (`TimeControl`, `IPreferences` not used here),
`UngulaHal` (`gpio`, `uart`), `EmblogX` (logging — only legacy paths).
ESP32-only because step pulses ride a GPTimer ISR and the ramp ride
`esp_timer`. Host builds compile the platform-independent interfaces and
event/coordinator logic only.

Two namespaces:

- `motor::` — everything in this library.
- `tmc::` — the TMC2209 driver (in `motor/drivers/tmc2209.h`).

Umbrella header: `<ungula_motor.h>` brings in interfaces, types,
`MotorEventPublisher`, `RemoteMotor`, and `MotorCoordinator`. **Platform
pieces are not auto-included** — bring them in explicitly:

```cpp
#include <motor/local_motor.h>           // ESP-IDF gptimer + esp_timer
#include <motor/drivers/tmc2209.h>       // UART + GPIO
#include <motor/homing/limit_switch_homing_strategy.h>
#include <motor/homing/stall_homing_strategy.h>
#include <motor/homing/homing_runner.h>  // standalone runner (advanced)
```

---

## Usage

### Use case: Bare minimum — driver, pins, run

```cpp
#include <motor/local_motor.h>
#include <motor/drivers/tmc2209.h>
#include <hal/uart/uart.h>

ungula::uart::Uart uart(2);
tmc::Tmc2209       driver(uart, 0.11F, /*step=*/26, /*en=*/27, /*dir=*/14, /*addr=*/0);
motor::LocalMotor  mot;

void setup() {
    uart.begin(115200, /*tx=*/17, /*rx=*/16);
    mot.setDriver(driver);
    mot.begin();          // seeds default 500 SPS / 500 ms ramps
    mot.enable();
    mot.moveForward();
}

void loop() {}
```

When to use this: smoke-test on a fresh build. Motor turns at a slow,
visible speed. From here, layer on profiles, limits, stall, homing.

### Use case: Realistic axis with limits and stall-stop

```cpp
#include <motor/local_motor.h>
#include <motor/drivers/tmc2209.h>
#include <hal/uart/uart.h>

ungula::uart::Uart uart(2);
tmc::Tmc2209       driver(uart, 0.11F, 26, 27, 14, 0);
motor::LocalMotor  mot;

void setup() {
    uart.begin(115200, 17, 16);
    driver.configureStall({
        .diagPin           = 25,
        .sensitivity       = 40,
        .diagConfirmCount  = 8,
        .sgSlope           = 0.17F,
        .sgMaxBaseline     = 300,
        .sgDropFraction    = 0.65F,
        .sgConfirmCount    = 10,
    });

    mot.setDriver(driver);
    mot.setMicrosteps(16);
    mot.setRunCurrent(1100);
    mot.setStepsPerMm(200.0F);
    mot.addLimitBackward(/*pin=*/32);
    mot.addLimitForward(/*pin=*/33);
    mot.setAutoStopOnStall(true);

    mot.setProfileSpeed(motor::MotionProfile::JOG, 10000);
    mot.setProfileAccel(motor::MotionProfile::JOG, 500);
    mot.setActiveProfile(motor::MotionProfile::JOG);

    mot.begin();
    mot.enable();
}

void onStop() { mot.stop(); }
```

### Use case: Status queries (black-box, FSM-free)

```cpp
if (mot.isIdle()) {
    switch (mot.lastStopReason()) {
        case motor::StopReason::TargetReached:  /* completed cleanly */ break;
        case motor::StopReason::UserStop:       /* operator pressed stop */ break;
        case motor::StopReason::LimitHit:       /* recover */ break;
        case motor::StopReason::Stall:          mot.clearStall(); break;
        case motor::StopReason::Fault:          mot.clearFault(); break;
        default: break;
    }
}

if (mot.isLimitActive(motor::Direction::BACKWARD, /*index=*/0)) {
    // Sitting on the home switch right now.
}

bool busy = mot.isMoving() || mot.isHoming() || mot.isStalling();
```

When to use this: every state query a host needs to make. Avoid
comparing against `MotorFsmState` directly — those soft terminal states
auto-clear within one service tick.

### Use case: Positional moves

```cpp
mot.moveTo(1500.0F, motor::DistanceUnit::STEPS);
mot.moveTo(25.0F,   motor::DistanceUnit::MM);
mot.moveBy(-10.0F,  motor::DistanceUnit::MM);
```

`MM` and `CM` need `setStepsPerMm` first. `DEGREES` needs
`setStepsPerDegree`. Position tracking is ISR-precise, no overshoot.

### Use case: Limit-switch homing

```cpp
#include <motor/homing/limit_switch_homing_strategy.h>

motor::LimitSwitchHomingStrategy::Config cfg;
cfg.homingDirection = motor::Direction::BACKWARD;
cfg.fastSpeedSps    = 3000;
cfg.slowSpeedSps    = 500;
cfg.backoffSteps    = 300;

motor::LimitSwitchHomingStrategy strategy(cfg);

void setup() {
    // ... driver, pins, profiles ...
    mot.addLimitBackward(/*pin=*/32);
    mot.setHomingStrategy(&strategy);     // strategy must outlive mot
    mot.setHomingTimeout(10000);          // 0 = strategy decides
    mot.begin();                          // seeds isHomed() from pin state
    mot.enable();
}

void onHomeButton() { mot.home(); }       // non-blocking
```

### Use case: Stall homing (no limit switch, hard stop)

```cpp
#include <motor/homing/stall_homing_strategy.h>

motor::StallHomingStrategy::Config cfg;
cfg.homingDirection = motor::Direction::BACKWARD;
cfg.fastSpeedSps    = 2000;
cfg.slowSpeedSps    = 500;
cfg.backoffSteps    = 200;
cfg.finalApproach   = true;               // false = single-touch

motor::StallHomingStrategy strategy(cfg);
mot.setHomingStrategy(&strategy);
// strategy forces setAutoStopOnStall(true) inside begin().
// isHomed() always starts false after reboot — you must home() once.
```

### Use case: Parallel multi-axis homing

```cpp
mot_x.home();
mot_y.home();
while (mot_x.isHoming() || mot_y.isHoming()) {
    ungula::TimeControl::delayMs(5);
}
if (!mot_x.isHomed() || !mot_y.isHomed()) { /* report */ }
```

### Use case: Speed-proportional run current

```cpp
motor::CurrentCurve curve;
curve.minSps = 200;  curve.maxSps = 3000;
curve.minMa  = 1300; curve.maxMa  = 900;
mot.setCurrentCurve(curve);
mot.setCurrentCurveEnabled(true);   // off by default
```

`mot` calls `driver.setRunCurrent(mA)` on every commanded speed change,
linearly interpolated. Outside `[minSps, maxSps]` clamps to the nearest
endpoint.

### Use case: Subscribe to motor events

```cpp
class Listener : public motor::IMotorEventListener {
    public:
        void onMotorEvent(const motor::MotorEvent& ev) override {
            if (ev.type == motor::MotorEventType::LimitSwitchHit) { /* … */ }
        }
};

Listener listener;
mot.subscribe(&listener);    // up to MAX_MOTOR_EVENT_LISTENERS = 4
```

Events fire from the 10 ms service timer — keep handlers short and
non-blocking.

### Use case: Remote motor proxy across an ESP-NOW link

```cpp
#include <motor/remote_motor.h>

class MyRouter : public motor::IMotorCommandSink {
    public:
        bool send(uint8_t motorId, motor::MotorCommandType cmd) override          { /* serialise + transport.send */ return true; }
        bool sendMove(uint8_t motorId, motor::MotorCommandType cmd,
                      const motor::MotorMoveParams& p) override                   { return true; }
        bool sendProfile(uint8_t motorId,
                         const motor::MotionProfileSpec& profile) override        { return true; }
};

MyRouter           router;
motor::RemoteMotor mot(router, /*motorId=*/3);

void onPeerStateMessage(motor::MotorFsmState s, int32_t pos) {
    mot.updateState(s, pos);     // refresh cache from remote events
}
```

When to use this: one ESP32 commands motors physically attached to
another. The proxy implements `IMotor` — coordinators don't need to know
the motor is remote.

### Use case: Multi-motor coordinator

```cpp
#include <motor/motor_coordinator.h>

motor::MotorCoordinator coord;
coord.addMotor(&mot_x);   // up to MAX_COORDINATOR_MOTORS = 8
coord.addMotor(&mot_y);
mot_x.subscribe(&coord);  // coordinator listens to events
mot_y.subscribe(&coord);

coord.enableAll();
// later
coord.emergencyStopAll();
```

---

## Public types

| Type | Header | Purpose |
| ---- | ------ | ------- |
| `motor::IMotor` | `motor/i_motor.h` | Top-level motor interface |
| `motor::IHomeableMotor` | `motor/homing/i_homeable_motor.h` | `IMotor` + homing-strategy hooks |
| `motor::IMotorDriver` | `motor/i_motor_driver.h` | Driver chip abstraction |
| `motor::IMotorCommandSink` | `motor/i_motor_command_sink.h` | Wire transport for `RemoteMotor` |
| `motor::IMotorEventListener` | `motor/i_motor_event_listener.h` | Event sink |
| `motor::IHomingStrategy` | `motor/homing/i_homing_strategy.h` | Homing policy |
| `motor::LocalMotor` | `motor/local_motor.h` | The autonomous motor |
| `motor::RemoteMotor` | `motor/remote_motor.h` | Cross-board proxy |
| `motor::MotorCoordinator` | `motor/motor_coordinator.h` | Multi-motor orchestrator |
| `motor::MotorEventPublisher<N>` | `motor/motor_event_publisher.h` | Listener fan-out (used internally; safe to reuse) |
| `motor::MotorEvent`, `MotorEventType` | `motor/motor_event.h` | Event payload |
| `motor::MotorFsmState`, `StopReason` | `motor/motor_state.h` | FSM enums + helpers |
| `motor::Direction`, `MotionProfile`, `DistanceUnit`, `SpeedUnit`, `SpeedValue`, `ProfileConfig`, `CurrentCurve` | `motor/motor_types.h` | Value types |
| `motor::MotionProfileSpec`, `ProfileShape` | `motor/motion_profile.h` | Autonomous profile |
| `motor::MotorCommandType`, `MotorMoveParams` | `motor/i_motor_command_sink.h` | Wire commands |
| `motor::LimitSwitchHomingStrategy::Config` / `motor::StallHomingStrategy::Config` | `motor/homing/*.h` | Strategy tuning |
| `motor::HomingRunner` | `motor/homing/homing_runner.h` | Standalone runner (advanced) |
| `tmc::Tmc2209`, `tmc::StallConfig` | `motor/drivers/tmc2209.h` | TMC2209 driver |

Internals exposed only because they back `LocalMotor` (don't instantiate
from app code): `motor::MotorFsm`, `motor::StepGenerator`,
`motor::LimitSwitch`, `motor::StallDetector`.

Constants: `MAX_MOTOR_EVENT_LISTENERS = 4`,
`MAX_COORDINATOR_MOTORS = 8`, `PROFILE_COUNT = 3`,
`limit::MAX_PER_DIRECTION = 2`, `limit::DEBOUNCE_MS = 20`,
`step::TIMER_FREQ_HZ = 1'000'000`, `svc::MOTOR_SERVICE_US = 10'000`,
`GPIO_NONE = 0xFF`.

---

## Public functions / methods

### `motor::IMotor` (and therefore `LocalMotor` and `RemoteMotor`)

| Member | Notes |
| ------ | ----- |
| `void enable() / disable()` | `disable` stops motion + enters `Disabled`. |
| `void moveForward() / moveBackward()` | Continuous, uses active profile. |
| `void moveTo(float target, DistanceUnit = STEPS)` | Absolute. Needs `setStepsPerMm` / `setStepsPerDegree` for non-step units. |
| `void moveBy(float delta, DistanceUnit = STEPS)` | Relative. |
| `void executeProfile(const MotionProfileSpec&)` | Currently honours `startTimeMs`, `targetPosition`, `maxVelocitySps`. Other fields reserved. |
| `void stop()` | Soft, ramps using configured decel. |
| `void emergencyStop()` | Hard, no ramp. Forces FSM to `Idle` from any state, including `Stall` / `Fault`. Aborts homing. |
| `MotorFsmState state()` | Raw FSM. **Prefer the black-box getters below.** |
| `int32_t positionSteps()` | ISR step counter; safe anytime. |
| `bool isMoving()` | True for `Starting`, `RunningForward`, `RunningBackward`, `Decelerating`, `WaitingStart`. Stable. |
| `bool isIdle()` | True only for FSM `Idle`. False in `Stall` / `Fault` until acked. |
| `bool isStalling()` | Driver-asserts-stall OR FSM latched in `Stall`. |
| `StopReason lastStopReason()` | Latched on motion-end, kept across FSM auto-clear. Reset to `None` when next motion starts. |
| `bool wasLimitHit()` | Sticky; clears once axis moves and **all** registered limits release. |
| `bool isLimitActive(Direction)` / `(Direction, int32_t index)` | Debounced, polarity-aware read. False for unregistered switches. |
| `int32_t limitCount(Direction)` |  |
| `bool isHoming() / isHomed()` | `isHomed` seeded at `begin()` from strategy's `isAtHomeReference`. |

### `motor::LocalMotor` (additions)

Wiring (call before `begin`):

- `void setDriver(IMotorDriver&)` — required.
- `void addLimitBackward(uint8_t pin)` / `addLimitForward(uint8_t pin)` —
  default polarity is NC. Up to `limit::MAX_PER_DIRECTION = 2` per side.
- `void setHomingStrategy(IHomingStrategy*)` — `nullptr` disables `home()`.
- `void setHomingTimeout(int64_t ms)` — `0` = no timeout.
- `bool subscribe(IMotorEventListener*)` / `bool unsubscribe(...)`.

Configuration (anytime):

- `void setAutoStopOnStall(bool)` — when on, stall ⇒ FSM `Stall`.
- `void setMicrosteps(uint16_t)`, `void setRunCurrent(uint16_t mA)`.
- `void setCurrentCurve(const CurrentCurve&)` /
  `void setCurrentCurveEnabled(bool)` — opt-in; default off.
- `void setStepsPerMm(float)`, `void setStepsPerDegree(float)`.
- `void setProfileSpeed(MotionProfile, int32_t sps)` /
  `void setProfileSpeed(MotionProfile, SpeedValue)`.
- `void setProfileAccel(MotionProfile, uint32_t ms)` /
  `void setProfileDecel(MotionProfile, uint32_t ms)` — `0` = instant.
- `void setActiveProfile(MotionProfile)`.

Lifecycle:

- `bool begin()` — initializes driver, GPTimer, `esp_timer` service.
  Returns `false` on timer setup failure.
- `void end()` — stop + release. Safe even if `begin` was never called.

Mid-motion adjustments:

- `void updateSpeed(int32_t sps, uint32_t accelMs, uint32_t decelMs)` /
  `(SpeedValue, uint32_t, uint32_t)` — smooth ramp; no-op when not moving.
- `void clearStall()` / `void clearFault()` — required to leave the
  hard terminal states. `emergencyStop()` is the hammer.
- `void resetPosition()` — only allowed when not moving.
- `void home()` — fire-and-forget; cancels in-progress motion first.

Diagnostics:

- `float currentSpeed()` (steps/s).
- `Direction direction()`.

`handleServiceTimer()` is exposed only because the `esp_timer` callback
must reach it — never call from app code.

### `motor::IMotorDriver`

`begin`, `enable`, `disable`, `setDirection(Direction)`,
`uint8_t stepPin()`, `setDirectionInverted(bool)`,
`setMicrosteps(uint16_t)`, `setRunCurrent(uint16_t mA)`,
`bool isStalling()`, `clearStall()`, plus optional overrides
`prepareStallDetection(speedSps, accelMs)`,
`updateStallDetectionSpeed(speedSps)`, `serviceStallDetection()`,
`uint32_t drvStatus()`, `uint8_t version()`, and informational
`const char* module() / info()`.

Drivers without stall detection return `false` from `isStalling()` and
implement `clearStall` as empty.

### `tmc::Tmc2209`

```cpp
Tmc2209(ungula::uart::Uart&, float rSenseOhms,
        uint8_t stepPin, uint8_t enablePin, uint8_t dirPin,
        uint8_t address = 0);
```

Implements the full `IMotorDriver`. Extras:

- `void configureStall(const StallConfig&)` — call **before** `begin`.
- `void setStallSensitivity(uint8_t 0..255)` — runtime tweak.
- Diagnostics: `uint16_t lastStallGuardResult()`, `int32_t diagScore()`,
  `int32_t sgScore()`, `uint16_t sgThreshold()`,
  `uint16_t sgBaseline()`, `uint8_t stallSensitivity()`,
  `int32_t diagScoreLimit()`.
- Register I/O: `void writeRegister(uint8_t, uint32_t)`,
  `uint32_t readRegister(uint8_t)`,
  `static uint8_t calcCrc(const uint8_t*, uint8_t)`,
  `uint16_t readStallGuardResult()`, `uint32_t clearGstat()`.
- Fine-grained config: `setRunCurrent(mA, holdFrac)`,
  `setHoldFraction`, `setInternalRsense`, `setToff`, `setBlankTime`,
  `setSpreadCycle`, `setPdnDisable`, `setIScaleAnalog`, `setShaft`,
  `setInterpol`, `setPwmAutoscale`, `setPwmAutograd`, `setIholddelay`,
  `setTpowerdown`, `setTpwmthrs`, `setTcoolthrs`,
  `setStallGuardThreshold`. Most projects don't need these — defaults
  applied by `begin()` are sane.

`StallConfig` defaults: `diagPin = GPIO_NONE`, `sensitivity = 0`
(stall detection off until set), `diagConfirmCount = 8`, `sgSlope = 0.17`,
`sgMaxBaseline = 300`, `sgDropFraction = 0.65`, `sgConfirmCount = 10`.

### `motor::IHomingStrategy`

`begin(IHomeableMotor&)`, `bool tick(IHomeableMotor&)` (returns `true`
when finished), `finish(IHomeableMotor&, bool succeeded)`,
`bool succeeded()`, `bool isAtHomeReference(const IHomeableMotor&)`.

Limit-switch and stall implementations described above. Common rule:
strategies talk only via `IHomeableMotor`, never directly to GPIO.

### `motor::HomingRunner` (advanced)

```cpp
HomingRunner(IHomeableMotor&, IHomingStrategy&, uint32_t timeoutMs = 0);
void start(); bool step(); void abort();
bool isRunning(); bool succeeded(); uint32_t elapsedMs();
```

Use this only when you want to drive a strategy against an
`IHomeableMotor` other than `LocalMotor` (tests, custom motor backends).
`LocalMotor::home()` already runs the runner internally, in-phase with
the service timer — for production code use that.

### `motor::RemoteMotor`

```cpp
RemoteMotor(IMotorCommandSink&, uint8_t motorId);
void updateState(MotorFsmState, int32_t position);
```

Implements `IMotor`. Black-box flags (`isStalling`, `lastStopReason`,
`wasLimitHit`, `isHoming`, `isHomed`, `isLimitActive`, `limitCount`)
report conservative defaults — the wire protocol does not currently
ship those fields.

### `motor::MotorCoordinator`

Inline in the header. `bool addMotor(IMotor*)`,
`IMotor* motor(uint8_t)`, `uint8_t motorCount()`, `void enableAll()`,
`void emergencyStopAll()`. Implements `IMotorEventListener`; cache the
last event in `lastEvent_` (no public getter — use a custom listener if
you need the stream).

### `motor::MotorEventPublisher<N>` (template)

Generic listener fan-out. `bool subscribe(IMotorEventListener*)` (false
when full or null), `bool unsubscribe(...)`, `void publish(const MotorEvent&)`,
`uint8_t listenerCount()`. Used by `LocalMotor` with `N = 4`. Re-use it
if you need event broadcasting on something else; no heap.

---

## Lifecycle

`LocalMotor`:

1. Construct UART (for TMC2209), driver, motor.
2. Configure UART (`uart.begin(...)`), then `driver.configureStall(...)` if
   needed.
3. Wiring: `setDriver`, `addLimit*`, `setHomingStrategy`,
   `subscribe`, scaling (`setStepsPerMm`, `setStepsPerDegree`),
   profiles, `setAutoStopOnStall`.
4. `begin()` — once. Boots GPTimer + `esp_timer` service. Seeds
   `isHomed` from the strategy. `enable()` once before motion.
5. Operate via `move*` / `stop` / `home`. The motor is autonomous from
   here; main loop has no timing obligation.
6. `end()` if you need to free timer resources (rarely needed; embedded
   firmware typically runs to power-off).

Stall detection sequence inside `LocalMotor`: every motion start calls
`driver.prepareStallDetection(speed, accel)`; every speed change calls
`updateStallDetectionSpeed(speed)`; the 10 ms service tick calls
`serviceStallDetection()` and reads `isStalling()`. Don't replicate this
in app code.

`RemoteMotor`: built on top of an application-supplied
`IMotorCommandSink`. The application's transport layer feeds
`updateState(...)` whenever a state message arrives from the remote
node. Black-box state flags surface only the FSM state — extend the
wire protocol if you need full fidelity.

---

## Error handling

- Motor commands never throw and never block. Invalid sequences (move
  while disabled, etc.) are absorbed by the FSM and surface as no-ops.
- Hard terminal states `Stall` / `Fault` require `clearStall()` /
  `clearFault()` — operator/host explicitly acknowledges. `emergencyStop()`
  is the override that bypasses the ack.
- Soft terminal states `TargetReached` / `LimitReached` auto-clear back
  to `Idle` within one service tick. Use `lastStopReason()` to check
  *why* the motor stopped without a polling race.
- `LocalMotor::begin()` returns `false` if GPTimer or `esp_timer` setup
  fails — treat as fatal.
- `subscribe(nullptr)` and listener-list overflow both return `false`.
- TMC2209 register reads time out (`REPLY_TIMEOUT_MS = 20`); a missing
  reply produces `0` — pair with `version()` reads at boot to detect a
  cabling fault.
- Homing watchdog (`setHomingTimeout`) aborts via the runner; the
  strategy's `finish(motor, false)` gets called and `isHomed()` stays
  false. `isHoming()` becomes false.

---

## Threading / timing / hardware notes

- **Step pulses + ramp**: GPTimer ISR + 10 ms `esp_timer` callback.
  Motion runs on the core where `begin()` was called.
- **Service timer**: 10 ms (`svc::MOTOR_SERVICE_US`). Calls the driver's
  stall service, debounces limit switches, drives the FSM, ticks the
  homing strategy in-phase. Listeners' `onMotorEvent` runs from this
  context — keep handlers fast.
- **Critical sections**: `LocalMotor` uses a portMUX (`g_motorMux`) to
  guard `direction_`, `moveTarget_`, `decelerating_`, `pendingProfile_`,
  etc. Application code can call commands from any task.
- **`StepGenerator` getters**: `position()`, `currentSpeed()`,
  `isPulsing()` rely on 32-bit hardware atomicity. Safe to read from any
  context.
- **Limit switch debounce**: 20 ms on a 10 ms tick — so first stable
  reading is 2–3 ticks after the change. Configure NO wiring with
  `LimitSwitch::configure(pin, /*invertPolarity=*/true)`. `LocalMotor`
  uses NC by default; the per-pin polarity setter is on the
  `LimitSwitch` directly (not exposed through `LocalMotor`).
- **TMC2209 UART**: blocking calls per register access (~3 ms round
  trip). Polled at most every `svc::SG_POLL_INTERVAL_MS = 50` ms by the
  driver, so the service tick stays responsive. Don't share the UART
  port across drivers.
- **Stall detection at low speed**: `stall::LOW_SPEED_SPS = 1200`
  suppresses DIAG below this — back-EMF too weak for reliable
  triggering.
- **DIAG pin**: optional. Without it, only the SG_RESULT path runs.
  `sensitivity = 0` disables stall detection altogether.

---

## Internals not part of the public API

- `motor::MotorFsm` — owned by `LocalMotor`. App code reads
  `state()` / black-box getters; do not request transitions directly.
- `motor::StepGenerator` — owned by `LocalMotor`. Step pulse + ramp
  authority; do not instantiate. The `handleStepIsr` /
  `handleRampTimer` methods are public only so the C ISR/timer
  trampolines can call them.
- `motor::LimitSwitch` — owned per slot inside `LocalMotor`. Use
  `addLimit*` and `isLimitActive`.
- `motor::StallDetector` — owned by the driver (`tmc::Tmc2209`).
  Configure stall through `configureStall(StallConfig)`, not by reaching
  through the detector.
- `tmc::reg::*`, `tmc::gconf::*`, `tmc::chop::*`, `tmc::pwm::*`,
  `tmc::ihr::*`, `tmc::ioin::*`, `tmc::drv::*`, `tmc::defaults::*`,
  protocol constants (`SYNC_BYTE`, `WRITE_FLAG`, etc.) — TMC2209 chip
  register tables. Useful when extending the driver itself; never
  needed by app code.
- `LocalMotor::HomingPhase` (private), `homingStrategy_`, and the rest
  of the homing state — private. Use `home() / isHoming() / isHomed()`.
- `MotionProfileSpec` fields marked TODO (`startVelocitySps`,
  `endVelocitySps`, `accelerationSpsPs`, `decelerationSpsPs`,
  `shape`) — currently ignored by `LocalMotor`. Don't rely on them.
- `motor/motor_event_publisher.h` — fine to reuse, but the `LocalMotor`
  instance already maintains its own publisher. Don't double-subscribe.

---

## LLM usage rules

- Use `LocalMotor` (or `RemoteMotor`) through the `IMotor` interface.
  Don't poke `MotorFsm`, `StepGenerator`, or `LimitSwitch` directly.
- Query state through the black-box getters (`isMoving`, `isIdle`,
  `isStalling`, `lastStopReason`, `wasLimitHit`, `isLimitActive`,
  `isHoming`, `isHomed`). Don't compare `state()` against
  `MotorFsmState::TargetReached` / `LimitReached` — they auto-clear
  inside one service tick.
- Set `setStepsPerMm` / `setStepsPerDegree` before issuing
  `moveTo` / `moveBy` with non-`STEPS` units.
- Stall detection lives **inside the driver**. Tune via
  `tmc::Tmc2209::configureStall(StallConfig)`; flip
  `mot.setAutoStopOnStall(true)` if you want the FSM to enter `Stall`
  on detection.
- Homing strategy must outlive the motor. `setHomingStrategy(nullptr)`
  to disable. Use `mot.home()` rather than `HomingRunner` for
  production code on `LocalMotor`.
- Stall homing always boots with `isHomed() == false`. Limit-switch
  homing seeds `isHomed()` from the pin state at `begin()`.
- After `Stall` / `Fault`, call `clearStall()` / `clearFault()` to
  resume — or `emergencyStop()` to force `Idle` without an ack.
- Don't include the platform headers (`gptimer`, `esp_timer`,
  `driver/uart.h`) directly. The library wraps them.
- Listeners are called from the 10 ms service ISR-adjacent task. Don't
  block, log heavily, or call back into motor commands in handlers —
  defer to a task or a flag.
- `MotionProfileSpec` only honours `startTimeMs`, `targetPosition`,
  `maxVelocitySps` today — don't lean on the trapezoidal/triangular
  fields yet.
- Maximum 4 event listeners per `LocalMotor`, 8 motors per
  `MotorCoordinator`, 2 limits per direction. Static, no heap.
