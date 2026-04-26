# UngulaMotor

> **High-performance embedded C++ libraries for ESP32, STM32 and other MCUs** — stepper motor control with autonomous ISR motion and 11-state FSM. Supported targets: ESP32 only.

Stepper motor control for ESP32. Provides autonomous motion with a finite state machine, acceleration ramps, limit switches, stall detection, and an event system — all driven by hardware timer ISRs with no polling required from your main loop.

## How it works

You create a `LocalMotor`, wire it to an `IMotorDriver` (e.g. `Tmc2209`), call `begin()`, and the motor runs itself. Step pulse generation, acceleration ramps, limit switch monitoring, and stall detection all happen inside timer ISRs. Your application just sends commands (`moveForward()`, `stop()`, `moveTo(...)`) and reacts to events through a small set of stable getters — see [Black-box status getters](#black-box-status-getters).

## Minimal project — pins + defaults, just run

The motor seeds every motion profile that you did not configure with a safe default at `begin()` time (500 SPS, 500 ms ramp). That means a bare-bones project — driver + pins + `begin()` + `enable()` + `moveForward()` — actually moves at a slow, visible speed without any extra setup. Override any field with `setProfileSpeed/Accel/Decel()` before motion if you want a different feel.

```cpp
#include <motor/local_motor.h>
#include <motor/drivers/tmc2209.h>

// Driver owns STEP, EN, DIR pins and the UART
tmc::Tmc2209 driver(uart, 0.11F, PIN_STEP, PIN_EN, PIN_DIR, /*addr=*/0);
motor::LocalMotor mot;

void setup() {
    mot.setDriver(driver);
    mot.begin();          // seeds default 500 SPS / 500 ms ramps
    mot.enable();
    mot.moveForward();    // starts moving at the default profile
}

void loop() {
    // Nothing to do. The motor runs itself.
}
```

That is the smallest project that compiles and runs on real hardware. From there, add limits, stall, homing, and faster profiles as the application needs them.

## Realistic project — limits, stall, and a homing strategy

```cpp
#include <motor/local_motor.h>
#include <motor/drivers/tmc2209.h>
#include <motor/homing/limit_switch_homing_strategy.h>

tmc::Tmc2209 driver(uart, 0.11F, PIN_STEP, PIN_EN, PIN_DIR, /*addr=*/0);
motor::LocalMotor mot;

motor::LimitSwitchHomingStrategy::Config homingCfg;
motor::LimitSwitchHomingStrategy homing(homingCfg);

void setup() {
    mot.setDriver(driver);
    mot.setRunCurrent(1100);
    mot.setMicrosteps(16);
    mot.setStepsPerMm(200.0F);

    // Limits (NC by default; pass invertPolarity=true for NO wiring)
    mot.addLimitBackward(PIN_HOME);   // index 0 on the BACKWARD side
    mot.addLimitForward(PIN_END);     // index 0 on the FORWARD side

    // Stall behaviour — motor goes to FSM Stall and stops on detection
    mot.setAutoStopOnStall(true);

    // Profiles — override the defaults for a faster jog
    mot.setProfileSpeed(motor::MotionProfile::JOG, 10000);
    mot.setProfileAccel(motor::MotionProfile::JOG, 500);

    // Homing strategy
    homingCfg.homingDirection = motor::Direction::BACKWARD;
    homingCfg.fastSpeedSps    = 3000;
    homingCfg.slowSpeedSps    = 500;
    homingCfg.backoffSteps    = 300;
    mot.setHomingStrategy(&homing);
    mot.setHomingTimeout(10000);

    mot.begin();
    mot.enable();
}

void loop() {
    // Do the application's work. Nothing motor-specific belongs here.
}

void onJogForward() {
    mot.setActiveProfile(motor::MotionProfile::JOG);
    mot.moveForward();
}
void onStop() { mot.stop(); }
void onHome() { mot.home(); }
```

## Architecture

```text
┌─────────────────────────────────────────────────────┐
│  Application                                        │
│  (HTTP handlers, protocol, UI)                      │
└────────────────────┬────────────────────────────────┘
                     │ IMotor interface
         ┌───────────┴───────────┐
         │                       │
   LocalMotor              RemoteMotor
   (real hardware)         (proxy over transport)
         │
         ├── StepGenerator    GPTimer ISR for step pulses + ramp
         ├── MotorFsm         11-state machine with event publishing
         ├── LimitSwitch[]    Debounced, checked every 10 ms
         └── IMotorDriver     Hardware abstraction
               │
           Tmc2209            UART register access, stall detection
           (or any driver)
```

**LocalMotor** is the real thing — it generates step pulses via a hardware timer ISR and monitors safety (limits, stalls) via a 10 ms software timer. After `begin()`, it needs zero attention from your main loop.

**RemoteMotor** implements the same `IMotor` interface but sends commands through an `IMotorCommandSink` (ESP-NOW, UART, whatever). Useful when one board controls a motor on another board.

**MotorCoordinator** manages multiple `IMotor` instances for multi-axis systems.

## Black-box status getters

The host should never need to reason about which FSM state the motor is currently in. Every question worth asking has a stable getter that survives the FSM's auto-clear of the soft terminal states (`TargetReached`, `LimitReached`):

| Question | Getter | Notes |
| --- | --- | --- |
| Is the motor running? | `isMoving()` | True for `Starting`, `RunningForward`, `RunningBackward`, `Decelerating`, `WaitingStart`. |
| Is it idle? | `isIdle()` | True only for FSM `Idle`. False while moving and false in `Stall` / `Fault` (the host has not acknowledged yet). |
| Is it stalling? | `isStalling()` | Either the driver is asserting stall right now, or the FSM is latched in `Stall` waiting for `clearStall()`. |
| Why did the last motion end? | `lastStopReason()` | Returns a `StopReason`. Latched on the motion-ending event and kept across the FSM auto-clear. Reset to `None` on the next motion command. |
| Did a limit cause the stop? | `wasLimitHit()` | Sticky — auto-clears once the axis moves again *and* every limit has released. |
| Is limit N currently asserted? | `isLimitActive(dir, index)` | Per-switch query. `isLimitActive(dir)` answers "any switch on that side". `limitCount(dir)` for iterating. |
| Is a homing run in progress? | `isHoming()` | Goes false on success, failure, user stop, e-stop, or watchdog. |
| Is the axis homed? | `isHomed()` | True after a successful homing run. Seeded at `begin()` from limit-switch strategies; always false at boot for stall strategies. |

### `StopReason` values

| Value | Meaning |
| --- | --- |
| `None` | No motion has run yet, or one is in progress. |
| `TargetReached` | A `moveTo` / `moveBy` / profile move completed cleanly. |
| `UserStop` | `stop()` was called and the deceleration ramp finished. |
| `LimitHit` | A limit switch fired during motion. |
| `Stall` | The driver reported a stall. |
| `Fault` | A hardware fault was raised. |
| `EmergencyStop` | `emergencyStop()` was called. |
| `Disabled` | `disable()` was called. |

`stopReasonName(reason)` returns the human-readable name for logs.

### Status query cookbook

```cpp
// "Is the motor idle because it just completed a move, or because the
//  user pressed stop, or because it stalled?"
if (mot.isIdle()) {
    switch (mot.lastStopReason()) {
        case motor::StopReason::TargetReached: handleCycleComplete(); break;
        case motor::StopReason::UserStop:      handleUserCancel();    break;
        case motor::StopReason::Stall:         handleStallRecovery(); break;
        case motor::StopReason::LimitHit:      handleLimitRecovery(); break;
        case motor::StopReason::Fault:         handleFault();         break;
        default: /* None / Disabled / EmergencyStop — nothing to do */ break;
    }
}

// "Is limit switch 0 on the backward side closed right now?"
if (mot.isLimitActive(motor::Direction::BACKWARD, 0)) {
    showHomeIndicator();
}

// "Sweep every registered limit on the forward side."
for (int32_t i = 0; i < mot.limitCount(motor::Direction::FORWARD); ++i) {
    if (mot.isLimitActive(motor::Direction::FORWARD, i)) {
        // ...
    }
}

// "Is anything happening on the motor right now that should block a new
//  move command?"
const bool busy = mot.isMoving() || mot.isHoming() || mot.isStalling();
```

Stall and Fault still require an explicit `clearStall()` / `clearFault()` from the host — that is intentional, the operator must acknowledge before the FSM accepts new commands. `emergencyStop()` is the override: it forces the FSM back to `Idle` from any reachable state.

## IMotorDriver — writing your own driver

The driver interface handles everything between the ESP32 and the motor driver chip. Pins, registers, communication protocol — all behind this wall.

```cpp
class IMotorDriver {
public:
    virtual void begin() = 0;                        // init GPIO, UART, registers
    virtual void enable() = 0;                       // energise the motor
    virtual void disable() = 0;                      // de-energise
    virtual void setDirection(Direction dir) = 0;     // FORWARD or BACKWARD
    virtual uint8_t stepPin() const = 0;             // which GPIO gets step pulses
    virtual void setDirectionInverted(bool inv) = 0; // swap direction without rewiring
    virtual void setMicrosteps(uint16_t ms) = 0;
    virtual void setRunCurrent(uint16_t mA) = 0;
    virtual bool isStalling() const = 0;             // driver's verdict
    virtual void clearStall() = 0;

    // Optional — override if your driver supports stall detection
    virtual void prepareStallDetection(int32_t speedSps, uint32_t accelMs) {}
    virtual void updateStallDetectionSpeed(int32_t speedSps) {}
    virtual void serviceStallDetection() {}

    // Optional — override if your driver supports status registers
    virtual uint32_t drvStatus() { return 0; }
    virtual uint8_t version() { return 0; }
};
```

The motor controller never touches GPIO directly. It calls the driver for everything, which means you can swap a TMC2209 for a DRV8825 or an A4988 without touching motor logic.

Stall detection follows a strict split: the driver detects (DIAG pin, SG registers, back-EMF — whatever the chip supports). The motor decides what to do about it (stop, retry, notify).

## TMC2209 driver

The included `Tmc2209` driver talks directly to the chip over UART — no TMCStepper Arduino library needed. It handles register-level communication, current scaling, microstepping, and dual-path stall detection (DIAG pin + SG_RESULT register).

```cpp
#include <motor/drivers/tmc2209.h>

// UART must be set up separately
ungula::uart::Uart tmcUart(2);
tmcUart.begin(115200, PIN_TX, PIN_RX);

// Driver owns STEP, EN, DIR pins
tmc::Tmc2209 driver(tmcUart, 0.11F, PIN_STEP, PIN_EN, PIN_DIR);

// Configure stall detection (optional, all-in-one)
driver.configureStall({
    .diagPin = PIN_DIAG,
    .sensitivity = 40,
    .diagConfirmCount = 8,
    .sgSlope = 0.17F,
    .sgMaxBaseline = 300,
    .sgDropFraction = 0.65F,
    .sgConfirmCount = 10,
});

driver.begin();
```

## Motor FSM states

The motor goes through 11 states automatically. `Decelerating` collapses straight to `Idle` once the ramp reaches zero — there is no separate "Stopped" stage. Soft terminal states (`TargetReached`, `LimitReached`) are auto-cleared back to `Idle` by the service timer one tick after they're observable. Hard terminal states (`Stall`, `Fault`) require an explicit `clearStall()` / `clearFault()` acknowledgement so the host actively decides to recover.

| State | Auto-clears? | Meaning |
| --- | --- | --- |
| `Disabled` | — | Driver output stage off. Call `enable()` to move to Idle. |
| `Idle` | — | Ready for commands. |
| `WaitingStart` | — | Profile loaded, waiting for `startTimeMs`. |
| `Starting` | — | Ramping up. |
| `RunningForward` | — | At target speed, forward. |
| `RunningBackward` | — | At target speed, backward. |
| `Decelerating` | → `Idle` | Ramping down after `stop()`. |
| `TargetReached` | → `Idle` | Positional move completed (visible for one service tick). |
| `LimitReached` | → `Idle` | Limit switch triggered (visible for one service tick). |
| `Stall` | needs `clearStall()` | Stall detected — host acknowledges. |
| `Fault` | needs `clearFault()` | Hardware fault — host acknowledges. |

`emergencyStop()` is the heavy hammer — it reaches `Idle` from any reachable state, including `Stall` and `Fault`, bypassing the explicit acknowledgement (because if the operator hits e-stop they don't care why the motor stopped, only that it does).

## Events

Subscribe to motor events to react without polling:

```cpp
class MyListener : public motor::IMotorEventListener {
public:
    void onMotorEvent(const motor::MotorEvent& event) override {
        if (event.type == motor::MotorEventType::LimitSwitchHit) {
            // handle limit
        }
    }
};

MyListener listener;
mot.subscribe(&listener);
```

Events are delivered synchronously from the 10 ms service timer — keep handlers fast and non-blocking.

## Motion profiles

Three named profiles let you store different speed/ramp combinations:

```cpp
// Fast jog for manual positioning
mot.setProfileSpeed(motor::MotionProfile::JOG, 15000);
mot.setProfileAccel(motor::MotionProfile::JOG, 300);

// Slow homing toward limit switch
mot.setProfileSpeed(motor::MotionProfile::HOMING, 2000);
mot.setProfileAccel(motor::MotionProfile::HOMING, 500);

// Production cycle speed
mot.setProfileSpeed(motor::MotionProfile::CYCLE, 8000);
mot.setProfileAccel(motor::MotionProfile::CYCLE, 400);
mot.setProfileDecel(motor::MotionProfile::CYCLE, 200);

// Select which profile is active
mot.setActiveProfile(motor::MotionProfile::JOG);
mot.moveForward();  // uses JOG speed and ramp
```

## Positional moves

```cpp
// Move to absolute position (in steps, mm, cm, or degrees)
mot.moveTo(1500.0F, motor::DistanceUnit::STEPS);
mot.moveTo(25.0F, motor::DistanceUnit::MM);

// Move relative
mot.moveBy(-10.0F, motor::DistanceUnit::MM);
```

The step generator handles position tracking at ISR level with zero overshoot.

## Speed-proportional run current (optional)

TMC2209-class drivers can change run current at runtime over UART. When the mechanical load varies with speed — a vertical axis holding weight at a slow process move, a rotary axis that only needs torque when accelerating — you can install a linear current-vs-speed curve on `LocalMotor` so the driver current tracks the commanded SPS. Disabled by default, so existing projects are unaffected.

```cpp
// High torque at low speed (vertical axis holding a hot die assembly).
motor::CurrentCurve curve;
curve.minSps = 200;
curve.maxSps = 3000;
curve.minMa  = 1300;   // slow → more current
curve.maxMa  = 900;    // fast → back off

mot.setCurrentCurve(curve);
mot.setCurrentCurveEnabled(true);
```

The motor calls `driver.setRunCurrent(mA)` on every commanded speed change (`moveForward`, `moveTo`, `updateSpeed`, etc.) using linear interpolation between `minSps/minMa` and `maxSps/maxMa`. Speeds outside that range clamp to the nearest endpoint. Works with any driver that honours `setRunCurrent()` — the mechanism lives in `LocalMotor`, not the driver, so a future DAC-on-VREF DRV8825 driver could use the same curve without changes here.

## Homing

Homing is a multi-step dance (approach → stop signal → back off → slow re-approach → zero) and different machines use different stop signals: a mechanical hard stop read via stall detection, a limit switch, an optical sensor, an encoder index pulse. The homing logic is **inside `LocalMotor`** — the application just installs a strategy at setup and calls `motor.home()`. No loop polling, no runner to tick.

### Black-box API

`LocalMotor` exposes a single non-blocking `home()` plus the homing-related flags from the [Black-box status getters](#black-box-status-getters) table above (`isHoming`, `isHomed`, `wasLimitHit`). Nothing else about the FSM needs to leak.

| Method | Meaning |
| --- | --- |
| `motor.home()` | Kick off homing using the installed strategy. Non-blocking. Cancels any in-progress motion first. |

```cpp
#include <motor/homing/limit_switch_homing_strategy.h>

motor::LimitSwitchHomingStrategy::Config cfg;
cfg.homingDirection = motor::Direction::BACKWARD;
cfg.fastSpeedSps    = 3000;
cfg.slowSpeedSps    = 500;
cfg.backoffSteps    = 300;

motor::LimitSwitchHomingStrategy strategy(cfg);

void setup() {
    // ... driver wiring, profiles, limits ...
    mot.setHomingStrategy(&strategy);
    mot.setHomingTimeout(10000);   // 0 = no watchdog; strategy decides
    mot.begin();                   // seeds isHomed() from strategy.isAtHomeReference()
    mot.enable();
}

void onHomeButton() {
    mot.home();                    // fire and forget
}

void onStartCycle() {
    if (!mot.isHomed()) { reportError(); return; }
    mot.moveTo(25.0F, motor::DistanceUnit::MM);
}

void loop() {
    if (!mot.isMoving() && !mot.isHoming()) {
        project_state_ = State::Idle;  // all motion done, back to idle
    }
}
```

`stop()` and `emergencyStop()` automatically abort any in-progress homing — the host never needs to coordinate between "stop the motor" and "stop the homing sequence".

### Strategies

The strategy decides HOW to find the reference. Same motor, different strategy in different projects. The strategy talks only to `IHomeableMotor` (the motor extended with the few hooks it needs — profile setters, clear-stall/fault, reset-position, peek-at-limit). That's how strategies get unit-tested against a mock with no hardware.

**Stall homing** — no limit switch, drive into a hard mechanical stop and read the driver's stall signal (TMC2209 StallGuard, DIAG pin, etc.). The strategy forces `setAutoStopOnStall(true)` in `begin()` so the FSM actually transitions to `Stall` on detection.

```cpp
#include <motor/homing/stall_homing_strategy.h>

motor::StallHomingStrategy::Config cfg;
cfg.homingDirection = motor::Direction::BACKWARD;
cfg.fastSpeedSps    = 2000;
cfg.slowSpeedSps    = 500;
cfg.backoffSteps    = 200;
cfg.finalApproach   = true;  // false = single-touch, faster but less repeatable

motor::StallHomingStrategy strategy(cfg);
mot.setHomingStrategy(&strategy);
```

Stall-based homing has no steady-state signal, so `isHomed()` always starts false after a reboot — the caller must run `home()` before trusting position.

**Limit-switch homing** — real limit switch registered on the motor via `addLimitBackward(pin)` or `addLimitForward(pin)` matching `homingDirection`. The strategy never reads the GPIO directly; `LocalMotor` owns debouncing and the FSM reports `LimitReached` on its own. Because the switch state is a steady-state signal, limit-switch homing can seed `isHomed()` at `begin()` time: if the axis happens to be sitting on the switch after a reboot, the motor already knows it's at home.

### Parallel multi-axis homing

Each motor owns its own homing run, so coordinating two axes is just two calls and two flags:

```cpp
mot_x.home();
mot_y.home();
while (mot_x.isHoming() || mot_y.isHoming()) {
    ungula::TimeControl::delayMs(5);
}
if (!mot_x.isHomed() || !mot_y.isHomed()) { /* report / retry */ }
```

### `HomingRunner` (advanced)

`HomingRunner` still ships as a standalone helper for callers that want to drive a strategy against an arbitrary `IHomeableMotor` implementation without going through `LocalMotor::home()` (tests, custom motor backends). For every production use on real `LocalMotor` the black-box API above is the right path — `LocalMotor` runs the runner internally, in-phase with the service timer.

## Directory structure and contents

```text
src/
  ungula_motor.h            Aggregator header (Arduino discovery)
  motor/                        Main motor system (motor:: namespace)
    motor_types.h               Enums, constants, value types
    motor_state.h               FSM state enum and names
    motor_event.h               Event types and MotorEvent struct
    motion_profile.h            MotionProfileSpec for complex moves
    i_motor.h                   Abstract motor interface
    i_motor_driver.h            Abstract driver interface
    i_motor_command_sink.h      Transport interface for remote control
    i_motor_event_listener.h    Event listener interface
    motor_event_publisher.h     Fixed-capacity event broadcaster
    motor_fsm.h/cpp             Finite state machine
    stall_detector.h            Dual-path stall scoring (DIAG + register)
    limit_switch.h              Debounced NC limit switch (20 ms)
    step_generator.h/cpp        GPTimer ISR + ramp calculator
    local_motor.h/cpp           Autonomous motor controller
    remote_motor.h/cpp          Proxy for cross-board control
    motor_coordinator.h         Multi-motor orchestrator
    drivers/
      tmc2209.h/cpp             TMC2209 UART driver with stall detection
    homing/
      i_homeable_motor.h        IMotor + hooks strategies need
      i_homing_strategy.h       Strategy interface (begin/tick/finish)
      homing_runner.h/cpp       Non-blocking driver for a strategy
      stall_homing_strategy.h/cpp         Hard-stop homing via stall detection
      limit_switch_homing_strategy.h/cpp  Limit-switch homing
```

## Testing

Desktop tests cover the legacy HAL (StepperController, StepperConfig, MotorTimer) and the homing subsystem (stall strategy, limit-switch strategy, runner lifecycle) using `MockHomeableMotor` — no real motor needed:

```shell
cd lib_motor/tests
./1_build.sh
./2_run.sh
```

To fetch dependencies from GitHub instead of using local sibling directories (which I do...):

```shell
cd lib_motor/tests
mkdir build && cd build
cmake .. -DUSE_LOCAL_DEPS=OFF
cmake --build .
ctest --output-on-failure
```

## Dependencies

| Library | Used for |
| --- | --- |
| UngulaCore | `TimeControl` (FSM timestamps), `GpioAccess` (limit switches) |
| UngulaHal | `Uart` (TMC2209 communication), `gpio` (driver pins) |
| EmblogX | Logging in legacy TmcStepper only |

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT — see LICENSE file.
