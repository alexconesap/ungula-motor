# UngulaMotor

> **High-performance embedded C++ libraries for ESP32, STM32 and other MCUs** — stepper motor control with autonomous ISR motion and 12-state FSM. Supported targets: ESP32 only.

Stepper motor control for ESP32. Provides autonomous motion with a finite state machine, acceleration ramps, limit switches, stall detection, and an event system — all driven by hardware timer ISRs with no polling required from your main loop.

## How it works

You create a `LocalMotor`, wire it to an `IMotorDriver` (e.g. `Tmc2209`), call `begin()`, and the motor runs itself. Step pulse generation, acceleration ramps, limit switch monitoring, and stall detection all happen inside timer ISRs. Your application just sends commands (`moveForward()`, `stop()`, `moveTo(...)`) and reacts to events.

```cpp
#include <motor/local_motor.h>
#include <motor/drivers/tmc2209.h>

// Create the driver — owns all GPIO pins and UART communication
tmc::Tmc2209 driver(uart, 0.11F, PIN_STEP, PIN_EN, PIN_DIR, /*addr=*/0);

// Create the motor — owns the FSM, step generator, and limit switches
motor::LocalMotor mot;

void setup() {
    // Wire driver to motor
    mot.setDriver(driver);
    mot.setRunCurrent(1100);
    mot.setMicrosteps(16);
    mot.setStepsPerMm(200.0F);

    // Add limit switches (optional)
    mot.addLimitBackward(PIN_HOME);
    mot.addLimitForward(PIN_END);

    // Configure motion profiles
    mot.setProfileSpeed(motor::MotionProfile::JOG, 10000);
    mot.setProfileAccel(motor::MotionProfile::JOG, 500);

    // Start — after this, the motor is fully autonomous
    mot.begin();
    mot.enable();
}

void loop() {
    // No motor polling needed. Do your application work.
}

// Triggered by a button, HTTP endpoint, or protocol message:
void onJogForward() {
    mot.setActiveProfile(motor::MotionProfile::JOG);
    mot.moveForward();
}

void onStop() {
    mot.stop();  // decelerates using the active profile's ramp
}
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
         ├── MotorFsm         12-state machine with event publishing
         ├── LimitSwitch[]    Debounced, checked every 10 ms
         └── IMotorDriver     Hardware abstraction
               │
           Tmc2209            UART register access, stall detection
           (or any driver)
```

**LocalMotor** is the real thing — it generates step pulses via a hardware timer ISR and monitors safety (limits, stalls) via a 10 ms software timer. After `begin()`, it needs zero attention from your main loop.

**RemoteMotor** implements the same `IMotor` interface but sends commands through an `IMotorCommandSink` (ESP-NOW, UART, whatever). Useful when one board controls a motor on another board.

**MotorCoordinator** manages multiple `IMotor` instances for multi-axis systems.

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

The motor goes through these states automatically:

| State | Meaning |
| --- | --- |
| `Disabled` | Driver output stage off. Call `enable()` to move to Idle. |
| `Idle` | Ready for commands. |
| `WaitingStart` | Command queued, waiting for conditions. |
| `Starting` | Ramping up. |
| `RunningForward` | Moving forward at target speed. |
| `RunningBackward` | Moving backward at target speed. |
| `Decelerating` | Ramping down after `stop()`. |
| `Stopped` | Motion finished (deceleration complete). |
| `TargetReached` | Positional move completed. |
| `LimitReached` | Limit switch triggered — motion stopped. |
| `Stall` | Stall detected — motion stopped. Call `clearStall()`. |
| `Fault` | Hardware fault. Call `clearFault()`. |

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

`LocalMotor` exposes four flags the application cares about — nothing else about the FSM needs to leak:

| Method | Meaning |
| --- | --- |
| `motor.home()` | Kick off homing using the installed strategy. Non-blocking. Cancels any in-progress motion first. |
| `motor.isHoming()` | True while the homing sequence is advancing. Goes false on success, failure, user stop, or watchdog. |
| `motor.isHomed()` | True once a homing run has succeeded. Seeded at `begin()` from the strategy (limit-switch strategies can report "already on the switch" after a reboot; stall strategies always start false). Cleared by any user-initiated motion that moves the axis off home. |
| `motor.wasLimitHit()` | Latches on any limit-triggered stop. Auto-clears once the axis resumes moving AND every limit pin has released. |

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
  basic_motor/                  Legacy HAL (ungula::motor:: namespace)
    i_motor_driver.h            Old driver interface
    stepper_config.h            Old config struct
    stepper_controller.h/cpp    Old stepper with manual service() loop
    tmc_stepper.h/cpp           Old TMC2209 wrapper (needs TMCStepper lib)
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
| TMCStepper `0.7.3` (external) | Legacy `basic_motor/tmc_stepper` driver only. Not used by the modern `motor/` stack. |

## Legacy: basic_motor

The `basic_motor/` directory contains the original motor HAL used by two pieces of one of my projects: RBB1 and RBB2. It requires manual `service()` calls from your main loop and external ISR wiring. It will be removed once all nodes migrate to the new motor system.

If you are starting a new project, use the `motor/` classes. See the quick start example at the top of this file.

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT — see LICENSE file.
