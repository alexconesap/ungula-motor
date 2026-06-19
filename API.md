# `lib_motor` API reference

Public surface of UngulaMotor 1.0.3. For an introductory overview and
copy-pastable examples, read `README.md` first; this file is the
reference you reach for when you already know what you're building.

Every declaration shown here lives in `namespace ungula::motor` unless
explicitly nested further. Concrete drivers live in
`ungula::motor::tmc2209`, `ungula::motor::stepdir`,
`ungula::motor::ypmc`, and `ungula::motor::rmd`.

## Headers you include

```cpp
// Always:
#include <ungula/motor.h>                  // MotorAxis, units, intents, FSM enums

// Per driver family:
#include <ungula/motor/drivers/tmc2209/tmc2209_driver.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_uart_transport.h>

#include <ungula/motor/drivers/generic_stepdir/generic_stepdir_driver.h>

#include <ungula/motor/drivers/ypmc/ypmc_driver.h>

#include <ungula/motor/drivers/rmd/rmd_can_driver.h>

// Step signal generator (only needed when using the pluggable
// constructor; the self-owns ctor pulls it in itself):
#include <ungula/motor/step_signal/rmt_step_signal.h>      // ESP32 RMT, 500+ kSPS
#include <ungula/motor/step_signal/gptimer_step_signal.h>  // ESP32 gptimer, <= 100 kSPS

// Planner (same as above):
#include <ungula/motor/planners/trapezoidal_planner.h>

// Limit system + homing (only when you wire those):
#include <ungula/motor/limits/limit_system.h>
#include <ungula/motor/homing/home_to_limit_strategy.h>
```

## Result + Status

```cpp
enum class ErrorCode : uint8_t {
    Ok = 0,
    // Lifecycle
    NotInitialized, AlreadyInitialized,
    // Configuration
    InvalidConfig, Unsupported,
    // State
    InvalidState, NotEnabled, NotHomed, HomingRequired,
    MotionInProgress, LimitActive,
    // Runtime
    DriverFault, TransportError, Timeout,
    // Resources / internal
    QueueFull, InternalError,
};

class Status {
public:
    static Status Ok();
    static Status Err(ErrorCode);
    bool ok() const;
    ErrorCode error() const;
};

template <typename T> class Result {
public:
    static Result<T> Ok(T value);
    static Result<T> Err(ErrorCode);
    bool ok() const;
    ErrorCode error() const;
    const T &value() const;
    T takeValue();
};
```

Every method that can fail returns `Status` or `Result<T>`. The host
checks `.ok()` and reads `.error()` to discriminate. There is no
exception path.

## `MotorAxis` -- the host-facing class

### Constructor

```cpp
MotorAxis(MotorAxisConfig cfg,
          IMotorDriver  &driver,
          ILimitSystem  *limits = nullptr,    // optional
          IHomingStrategy *homing = nullptr); // optional
```

`driver` must outlive the axis. `limits` and `homing` are optional;
pass `nullptr` when not used.

### Lifecycle

```cpp
Status begin();        // initialise driver + axis FSM. Reads identity
                       //   from chip-side bus where the driver supports
                       //   it; fails if no chip responds.
Status enable();       // energise the coil (or send 0x88 over CAN, etc).
Status disable();      // de-energise. Allowed from Idle / Faulted.
Status clearFault();   // recover from Faulted / EmergencyStopped back
                       //   to Disabled.
```

### Motion verbs

```cpp
Status moveForward();              // indefinite jog, +direction
Status moveBackward();             // indefinite jog, -direction
Status moveTo(DistanceValue);      // absolute position target
Status moveBy(DistanceValue);      // relative move from current position
Status home();                     // run the configured homing strategy
Status stop();                     // controlled decel
Status emergencyStop();            // hard halt
```

All return `Status::Err(InvalidState)` when the FSM is in a state that
can't accept motion (Disabled, Stopping, Faulted, ...). `moveForward`
and `moveBackward` enter the `Jogging` state and stay there until
something stops them (host `stop()`, a limit, a fault). They also
accept the `Homing` state - homing strategies use the jog verbs to
drive the axis, and the FSM stays in `Homing` (not `Jogging`) so
listeners see the homing phase explicitly. `moveBy` / `moveTo` enter
`Moving` and transition back to `Idle` when the planner completes;
those are `Idle`-only by design.

### Runtime tuning

```cpp
Status setSpeed(Speed);             // overrides cfg.limits.maxSpeed
Status setAcceleration(Acceleration);
Status setProfile(MotionProfile);   // see motion_profile.h
Status setIntent(MotorIntent);      // re-applies the intent through driver
```

`setIntent` is the only one that talks back to the chip mid-flight (on
drivers that support it). The others affect future motion commands.

### Service tick

```cpp
void service(int64_t nowMs);
```

Host calls this from its `loop()` or its own FreeRTOS task. The lib
spawns no internal task. `nowMs` is the host's millisecond clock
(`ungula::core::time::millis()` is the canonical source).

`service()` MUST be called from exactly one task; the FSM is not
internally synchronised against itself.

### Queries

```cpp
MotorState       state() const;
StopReason       lastStopReason() const;
Position         positionSteps() const;
float            position(DistanceUnit unit) const;
uint32_t         currentSps() const;
MotorDiagnostics diagnostics() const;
DriverIdentity   identity();
bool             isHomed() const;
bool             isMotionInFlight() const;   // true between arm and motion-done
const MotorAxisConfig &config() const;
```

`positionSteps()` is the open-loop commanded position. `position(unit)`
converts using `cfg.units` and returns 0.0 if the requested unit isn't
configured (e.g. asking for `Mm` when `stepsPerMm` is zero).
`diagnostics()` returns a snapshot suitable for serial / web logging
(see `motor_diagnostics.h`; the `toJson()` free function serialises it).

### Events

```cpp
void subscribe(IMotorEventListener &listener);   // single-slot
void unsubscribe();

class IMotorEventListener {
public:
    virtual void onMotorEvent(const MotorEvent &) = 0;
};
```

One subscriber slot per axis. Hosts that need fan-out implement their
own internal router. The single-slot policy keeps the axis allocation-
free.

`MotorEvent` carries `axisId`, `type` (`StateChanged`, `MotionStarted`,
`MotionStopped`, `MotionCompleted`, `LimitActivated`,
`HomingCompleted`, `HomingFailed`, `FaultRaised`, `FaultCleared`,
`EmergencyStopped`), the current `state`, `commandedPosition`,
`stopReason`, and `faultCode`.

## Units (`motor_units.h`)

### Speed / Distance / Acceleration

```cpp
struct Speed {
    static constexpr Speed stepsPerSec(float);
    static constexpr Speed rpm(float);
    static constexpr Speed degreesPerSec(float);
    static constexpr Speed mmPerSec(float);
    static constexpr Speed cmPerSec(float);
    static constexpr Speed cmPerMin(float);   // needs stepsPerMm
};

struct DistanceValue {
    static constexpr DistanceValue steps(float);
    static constexpr DistanceValue revolutions(float);
    static constexpr DistanceValue degrees(float);
    static constexpr DistanceValue mm(float);
    static constexpr DistanceValue cm(float);
};

struct Acceleration {
    static constexpr Acceleration stepsPerSecSquared(float);
    static constexpr Acceleration rpmPerSec(float);
    static constexpr Acceleration degreesPerSecSquared(float);
    static constexpr Acceleration mmPerSecSquared(float);
    static constexpr Acceleration rampMs(float);  // time 0→cruise, in ms
};
```

All three carry the value + the unit tag. The axis converts at the
boundary using `cfg.units`. Pass any unit that fits your domain; the
math is integer internally so RPM-based motors and mm-based linear
axes use the same code path.

`Speed::cmPerMin(v)` needs `cfg.units.stepsPerMm` configured (same as
`mmPerSec` / `cmPerSec`); it resolves as `v × 10 × stepsPerMm / 60`.
Returns `ErrorCode::InvalidConfig` when `stepsPerMm == 0`.

`Acceleration::rampMs(v)` is special — the value is **time in
milliseconds to ramp from 0 to the axis's configured cruise speed**, not
a rate. The axis resolves it to SPS² at the call boundary using the
**currently resolved cruise SPS** (`resolvedCruiseSps_`):

```
accel_sps² = cruise_sps × 1000 / ramp_ms
```

Practical consequence: if you call `setSpeed(...)` later, an accel that
was set as `rampMs` is **not** auto-recomputed against the new cruise.
The conversion happens at the moment `rampMs` is resolved (either inside
`begin()` for `cfg.limits.accel`, or inside `setAcceleration()` for a
runtime change). To keep a "constant ramp duration" semantics across a
speed change, call `setAcceleration(Acceleration::rampMs(...))` after
`setSpeed(...)`.

Rejected with `ErrorCode::InvalidConfig` when `ramp_ms <= 0`, or when
called inside `begin()` before the cruise speed has been resolved.

### `MotorUnits`

```cpp
struct MotorUnits {
    uint32_t stepsPerRevolution = 0;  // microsteps for one shaft rev
    float    stepsPerMm = 0.0f;       // 0 = linear units not configured
    float    stepsPerDegree = 0.0f;   // 0 = degree units not configured
};
```

`stepsPerRevolution` is required for any axis. `stepsPerMm` /
`stepsPerDegree` are optional; commanding in a unit that resolves to a
zero conversion factor returns `ErrorCode::InvalidConfig`.

## `MotorAxisConfig`

```cpp
struct MotorLimits {
    Speed        maxSpeed = Speed::stepsPerSec(0.0f);
    Acceleration accel    = Acceleration::stepsPerSecSquared(0.0f);
    Acceleration decel    = Acceleration::stepsPerSecSquared(0.0f);
    uint32_t     hardStepRateCeilingSps = 200000;
    // STEP-pulse widths live on the drive's electrical config
    // (`Tmc2209Config::minPulseHighUs`, `GenericStepDirConfig::...`)
    // - not on the axis. Defaults come from `motor_step_timing.h`.
};

struct MotorAxisConfig {
    MotorAxisId   id;
    const char   *name = "axis";   // string literal — not copied
    MotorUnits    units;
    MotorLimits   limits;
    MotionProfile profile = MotionProfile::cruise();
    MotorIntent   intent  = MotorIntent::Default;
    LimitWiring   limits_wiring[MAX_LIMIT_INPUTS];
};
```

`hardStepRateCeilingSps` is a safety net: the planner clamps cruise
SPS to this even when `maxSpeed` resolves higher. Set above your
expected step rate (e.g. 1 MHz for a 2500 RPM x 10000 PPR servo) so
the planner doesn't silently clip.

`limits_wiring` is auto-counted by `LimitSystem::begin` when you use
the array overload; no separate count field is required.

## `MotionProfile`

```cpp
struct MotionProfile {
    /* fields documented in motion_profile.h */

    static constexpr MotionProfile cruise();  // standard finite move
    static constexpr MotionProfile jog();     // indefinite move
    static constexpr MotionProfile home();    // homing pass
};
```

Use the factories. The structure carries planner hints (decel ramp
shape, allowed segment count, accel symmetry) that vary per use case.
`cruise()` is the right default for `moveTo` / `moveBy`.

## `MotorIntent`

```cpp
enum MotorIntent : uint16_t {
    Default          = 0,
    Quiet            = 1u << 0,
    HighTorque       = 1u << 1,
    EnergySaving     = 1u << 2,
    AdaptiveCurrent  = 1u << 3,
    Precision        = 1u << 4,
    Cool             = 1u << 5,
};

constexpr MotorIntent operator|(MotorIntent, MotorIntent);
constexpr bool has(MotorIntent set, MotorIntent flag);

enum class IntentSupport : uint8_t {
    Supported, PartiallySupported, Unsupported, Conflicted,
};
```

Set on `MotorAxisConfig::intent` or via `axis.setIntent(...)`. Per-
driver mapping:

| Intent           | TMC2209                   | YPMC / Generic STEP/DIR | RMD CAN     |
| ---------------- | ------------------------- | ----------------------- | ----------- |
| `Quiet`          | StealthChop               | Unsupported             | Unsupported |
| `HighTorque`     | SpreadCycle               | Unsupported             | Unsupported |
| `AdaptiveCurrent`| CoolStep                  | Unsupported             | Unsupported |
| `Cool`           | CoolStep                  | Unsupported             | Unsupported |
| `EnergySaving`   | CoolStep + reduced IHOLD  | Unsupported             | Unsupported |
| `Precision`      | INTPOL                    | Unsupported             | Unsupported |

Conflicts (`Quiet | HighTorque`): TMC2209 reports
`IntentSupport::PartiallySupported` and applies HighTorque (SpreadCycle
wins; cooler high-speed operation matters more than silence).

## FSM enums (`motor_state.h`)

```cpp
enum class MotorState : uint8_t {
    Uninitialized, Disabled, Idle,
    Moving, Jogging, Homing, Stopping,
    Faulted, EmergencyStopped,
};

enum class StopReason : uint8_t {
    None, TargetReached, UserStop, EmergencyStop,
    TravelLimit, LimitSwitch, StallDetected,
    DriverFault, Timeout,
};

enum class FaultCode : uint8_t {
    None, Stall,
    DriverShortCircuit, DriverOverTemperature,
    DriverUnderVoltage, DriverOpenLoad,
    PulseEngineFault, TransportError, Other,
};

enum class StopMode : uint8_t { Decelerate, Immediate };

enum class Direction : uint8_t { Forward = 0, Backward = 1 };
```

`motorStateToString` / `stopReasonToString` / `faultToString` /
`intentSupportToString` / `motorEventTypeToString` are inline helpers
in the corresponding headers (for `printf`-style debug output).

## Drivers

### TMC2209 (`drivers/tmc2209/`)

```cpp
namespace tmc = ungula::motor::tmc2209;

enum class tmc::MicrostepDepth : uint8_t {
    x1, x2, x4, x8, x16, x32, x64, x128, x256
};
constexpr uint32_t tmc::microstepMultiplier(MicrostepDepth);  // 1, 2, ..., 256

struct tmc::Tmc2209Config {
    uint8_t        slaveAddress = 0;
    uint8_t        enablePin = GPIO_NONE;
    bool           enableActiveLow = true;
    uint16_t       runCurrentMa = 800;
    uint16_t       holdCurrentMa = 400;
    float          senseResistorOhms = 0.11f;
    bool           useHighSensitivity = false;
    MicrostepDepth microsteps = MicrostepDepth::x16;
    bool           interpolate = true;
    // STEP / DIR (self-owns ctor only). Pulse defaults come from
    // `motor_step_timing.h` (1 / 1 / 5 us).
    uint8_t        stepPin = GPIO_NONE;
    uint8_t        dirPin = GPIO_NONE;
    bool           dirActiveHigh = true;
    uint32_t       dirSetupUs     = timing::kDefaultDirSetupUs;
    uint32_t       minPulseHighUs = timing::kDefaultMinPulseHighUs;
    uint32_t       minPulseLowUs  = timing::kDefaultMinPulseLowUs;
    // Stall (optional)
    uint8_t          diagPin = GPIO_NONE;
    StallSensitivity stallSensitivity = StallSensitivity::pct(50);
};

// Tagged sensitivity. Use one of:
//
//   tmc::StallSensitivity::pct(35);          // 0..100, lib maps to SGTHRS
//   tmc::StallSensitivity::rawSgthrs(90);    // 0..255 written to SGTHRS verbatim
//
// The chip trips DIAG when SG_RESULT < SGTHRS * 2. Use raw mode when
// you've read SGTHRS off a working setup and want to pin that exact
// value; use pct mode for "roughly this much sensitivity" tuning.
struct tmc::StallSensitivity {
    enum class Unit : uint8_t { Pct, RawSgthrs };
    uint16_t value;
    Unit     unit;
    static constexpr StallSensitivity pct(uint8_t v);
    static constexpr StallSensitivity rawSgthrs(uint8_t v);
    constexpr uint8_t toSgthrsByte() const;  // resolves either unit to the chip byte
};

class tmc::Tmc2209Driver : public IMotorDriver {
public:
    // Self-owns: builds RmtStepSignal + TrapezoidalPlanner internally.
    Tmc2209Driver(Tmc2209Config, IDeviceTransport &transport);

    // Pluggable: host owns the generator + planner.
    Tmc2209Driver(Tmc2209Config, IDeviceTransport &transport,
                  IStepSignalGenerator &, IMotionPlanner &);

    // Active generator pointer for wiring LimitSystem's ISR halt path.
    IStepSignalGenerator *stepSignalForLimits();

    // Static helpers exposed for tests / diagnostics:
    static uint8_t  milliampsToCs(uint16_t rmsMa, float rSenseOhms, bool useHighSensitivity);
    static uint16_t csToMilliamps(uint8_t cs, float rSenseOhms, bool useHighSensitivity);

    // ... IMotorDriver methods ...
};
```

Concrete transport: `tmc::Tmc2209UartTransport(ungula::hal::uart::Uart &)`.

Identity at `begin()`: reads IOIN[31:24] over the bus. `begin()`
returns `ErrorCode::TransportError` on read failure or when the
version byte is `0x00` / `0xFF` (no chip on this slave address).
Production silicon reports `0x21`.

### Generic STEP/DIR (`drivers/generic_stepdir/`)

```cpp
namespace stepdir = ungula::motor::stepdir;

struct stepdir::GenericStepDirConfig {
    // EN
    uint8_t  enablePin = GPIO_NONE;
    bool     enableActiveLow = true;
    // STEP / DIR (self-owns ctor only). Pulse defaults come from
    // `motor_step_timing.h` (1 / 1 / 5 us).
    uint8_t  stepPin = GPIO_NONE;
    uint8_t  dirPin = GPIO_NONE;
    bool     dirActiveHigh = true;
    uint32_t dirSetupUs     = timing::kDefaultDirSetupUs;
    uint32_t minPulseHighUs = timing::kDefaultMinPulseHighUs;
    uint32_t minPulseLowUs  = timing::kDefaultMinPulseLowUs;
};

class stepdir::GenericStepDirDriver : public IMotorDriver {
public:
    // Self-owns:
    explicit GenericStepDirDriver(GenericStepDirConfig);

    // Pluggable:
    GenericStepDirDriver(GenericStepDirConfig,
                         IStepSignalGenerator &, IMotionPlanner &);

    IStepSignalGenerator *stepSignalForLimits();
    // ... IMotorDriver methods ...

protected:
    // Subclass hooks for chip-specific brand specialisations.
    virtual void onBeforeArm(Direction);    // default no-op
    virtual void onMotionStarted();         // default no-op
    virtual void onMotionStopped();         // default no-op
};
```

Identity: `"Unknown" / "Generic STEP/DIR"`, hardcoded (no readback).
`applyIntent` returns `Unsupported` for any non-default intent --
plain STEP/DIR has nothing to negotiate at runtime.

### YPMC + S2SVD15 (`drivers/ypmc/`)

```cpp
namespace ypmc = ungula::motor::ypmc;

struct ypmc::YpmcConfig {
    stepdir::GenericStepDirConfig generic;  // EN + STEP/DIR + timing

    // Tandem: second DIR pin driven in parallel with `generic.dirPin`.
    uint8_t  secondaryDirPin = GPIO_NONE;
    bool     secondaryDirActiveHigh = true;
    bool     secondaryDirInverted = false;  // true for face-to-face mount

    // Optional 24 V holding-brake relay.
    uint8_t  brakePin = GPIO_NONE;
    bool     brakeReleaseActiveHigh = true;
    uint16_t brakeReleaseSettleMs = 50;
    uint16_t brakeEngageSettleMs = 50;
    bool     autoEngageOnMotionEnd = true;
};

class ypmc::YpmcStepDirDriver : public stepdir::GenericStepDirDriver {
public:
    // Self-owns:
    explicit YpmcStepDirDriver(YpmcConfig);

    // Pluggable:
    YpmcStepDirDriver(YpmcConfig,
                      IStepSignalGenerator &, IMotionPlanner &);

    DriverIdentity identity() override;  // "RATTMOTOR" / "YPMC + S2SVD15"
};
```

The tandem `secondaryDirPin` is the lib's accommodation of the
**Wendy** project (catheter coil winder) where two YPMC-400W servos
mount face-to-face on the same mandrel shaft. Both drives receive the
same STEP train through `generic.stepPin`; each gets its own DIR pin.
`secondaryDirInverted = true` makes drive B see the opposite
electrical level so the shaft rotates the same physical way despite
the face-to-face mounting. Outside that use case the field stays at
its `GPIO_NONE` / `false` defaults.

Brake control runs through `onBeforeArm` (release) and
`onMotionStopped` (engage); `stop(Immediate)` and any fault path
force-engage the brake regardless of `autoEngageOnMotionEnd`.

The drive's ALM / COIN inputs are NOT modelled in the driver. Wire
ALM as `LimitKind::EmergencyLimit` on `MotorAxisConfig::limits_wiring[]`;
read COIN host-side if the host needs it.

### MyActuator RMD CAN (`drivers/rmd/`)

**Optional driver.** Compiled only when `UNGULA_USE_CANBUS` is defined
at build time. Without that macro lib_motor has no dependency on
lib_canbus and the RMD types are absent from the binary. Enable with
`-DUNGULA_USE_CANBUS` and add lib_canbus to the project.

```cpp
namespace rmd = ungula::motor::rmd;

struct rmd::RmdConfig {
    uint8_t  motorId = 1;          // 1..32; CAN ID = 0x140 + motorId
    uint32_t stepsPerRevolution = 36000;  // RMD-X8: 0.01 deg = 1 step
    uint32_t commandTimeoutMs = 50;
    bool     releaseBrakeOnDisable = false;
};

class rmd::RmdCanDriver : public IMotorDriver {
public:
    // Takes a lib_hal::can::ICan & (typically a lib_hal::can::Can
    // instance, or a FakeCan in tests). The wire protocol lives in
    // lib_canbus (`ungula::canbus::rmd::*`); this driver is a thin
    // adapter from `IMotorDriver` verbs to those free functions.
    RmdCanDriver(RmdConfig, ungula::hal::can::ICan &);

    // Static helpers (exposed for diagnostics):
    static uint32_t canIdFor(uint8_t motorId);
    static int32_t  spsToCentideg(uint32_t sps, uint32_t stepsPerRevolution);
    static int32_t  stepsToCentideg(int32_t steps, uint32_t stepsPerRevolution);

    // ... IMotorDriver methods ...
};
```

No step signal generator, no planner. The motor closes its own
position / velocity loop on board. Verb mapping:

| `MotorAxis` verb            | RMD command (CAN byte 0)              |
| --------------------------- | ------------------------------------- |
| `moveTo` / `moveBy`         | 0xA4 (absolute position + max speed)  |
| `moveForward` / `Backward`  | 0xA2 (signed speed setpoint)          |
| `stop()` (Decelerate)       | 0x81 (controlled decel, brake on)     |
| `emergencyStop()`           | 0x80 (shutdown, optional brake release) |
| `enable()` / `clearFault()` | 0x88 (motor running / reset)          |
| `disable()`                 | 0x81 or 0x80 per `releaseBrakeOnDisable` |
| `identity()`                | 0x12 (read model + firmware), cached at begin |

`begin()` blocks for up to `commandTimeoutMs` waiting for the 0x12
reply; no reply means no RMD on this motor ID, and `begin()` returns
`ErrorCode::TransportError`.

`applyIntent` returns `Unsupported` for every flag: RMD's chopper /
current settings are firmware-baked on the drive.

## Step signal generators

`IStepSignalGenerator` is the pluggable interface:

```cpp
class IStepSignalGenerator {
public:
    virtual Status begin(uint8_t stepPin, uint8_t dirPin,
                         bool dirActiveHigh,
                         uint32_t dirSetupUs,
                         uint32_t minPulseHighUs,
                         uint32_t minPulseLowUs) = 0;
    virtual void   end() = 0;
    virtual Status armMove(const PlannedMove &) = 0;
    virtual Status stop(StopMode) = 0;
    virtual StepSignalStatus status() const = 0;
    virtual Position commandedPosition() const = 0;
    virtual uint32_t commandedSpsNow() const = 0;
    virtual Status   resetPosition(Position) = 0;
    virtual Status   clearFault() = 0;
    virtual uint32_t timerResolutionHz() const = 0;
    virtual uint32_t minTimerTicks() const = 0;
};
```

Two concrete generators ship with the lib:

### `RmtStepSignal` (default in self-owns mode)

ESP32 RMT peripheral. Hardware pulse generator: the CPU loads a
symbol buffer at `armMove()`; the RMT emits STEP edges autonomously
with zero per-pulse ISR cost. Practical ceiling 500+ kSPS.

```cpp
RmtStepSignal({ /*resolutionHz=*/1'000'000u,
                /*memBlockSymbols=*/64u,
                /*transQueueDepth=*/32u });
```

Position counter advances live as the encoder pushes symbols; wraps
through INT32 modular arithmetic on indefinite jogs (well-defined,
not UB).

### `GptimerStepSignal` (fallback)

ESP32 gptimer with ISR rearm. Practical ceiling around 100 kSPS
(the alarm ISR has to fire per pulse; CPU runs out at higher rates).

```cpp
ungula::hal::timer::drivers::HwTimer timer;
GptimerStepSignal({ /*timerResolutionHz=*/1'000'000u,
                    /*timerMinTicks=*/5u }, timer);
```

Use when:

- All RMT TX channels are claimed by another peripheral (IR, DShot,
  WS2812).
- The host explicitly wants its own gptimer instance for other use.

Hosts can also write their own `IStepSignalGenerator`. The interface
is small on purpose; PCNT-backed, DMA-fed, or RS-485-frame-driven
implementations are reasonable extensions.

## Motion planner

```cpp
class IMotionPlanner {
public:
    virtual PlannedMove planMove(Position fromSteps, Position toSteps,
                                 const PlannerLimits &limits,
                                 uint32_t timerResolutionHz,
                                 uint32_t minTimerTicks) = 0;
    virtual PlannedMove planJog(Direction dir, uint32_t safetyCapSteps,
                                const PlannerLimits &limits,
                                uint32_t timerResolutionHz,
                                uint32_t minTimerTicks) = 0;
};
```

`TrapezoidalPlanner` (default) emits 32-segment trapezoidal /
triangular profiles, integer-only, with exact step-count guarantee.

## Limit system (`limits/limit_system.h`)

```cpp
enum class LimitKind : uint8_t {
    TravelLimit, EmergencyLimit, HomeSensor, StallSensor,
};

enum class SwitchPolarity : uint8_t {
    NormallyOpen, NormallyClosed,
};

enum class LimitPinPullMode : uint8_t {
    HardwareResistors,  // no MCU pull — rely on external resistors
    McU,                // MCU internal pull, direction inferred: NO→down, NC→up
    Polarity,           // same as McU, explicit intent label
    Input,              // floating pin, no pull
    InternalPullUp,     // force MCU pull-up regardless of polarity
    InternalPullDown,   // force MCU pull-down regardless of polarity
};
// Behaviour groups (maps to gpio::PullMode):
//   { HardwareResistors, Input }          → NONE
//   { McU, Polarity }                     → infer from polarity: NO→DOWN, NC→UP
//   { InternalPullUp }                     → UP (ignores polarity)
//   { InternalPullDown }                   → DOWN (ignores polarity)

struct LimitWiring {
    uint8_t           pin = GPIO_NONE;
    LimitKind         kind = LimitKind::TravelLimit;
    Direction         direction = Direction::Forward;
    SwitchPolarity    polarity = SwitchPolarity::NormallyOpen;
    LimitPinPullMode  pullMode = LimitPinPullMode::Polarity;
    uint16_t          debounceMs = 20;
    // Stall-only timing (sensitivity is a chip-side concern -
    // see `Tmc2209Config::stallSensitivity`).
    uint16_t       stallArmDelayMs = 200;
    uint8_t        stallHitsToTrigger = 1;
};

constexpr uint8_t MAX_LIMIT_INPUTS = 8;

class LimitSystem : public ILimitSystem {
public:
    Status begin(const LimitWiring *wirings, uint8_t count,
                 IStepSignalGenerator *engineForIsr);
    Status begin(const LimitWiring (&wirings)[MAX_LIMIT_INPUTS],
                 IStepSignalGenerator *engineForIsr);
    // ... ILimitSystem methods ...
};
```

`engineForIsr` is the active step signal generator. Pass
`driver.stepSignalForLimits()` (TMC2209 / Generic / YPMC) so the limit
system can call `stop(Immediate)` from interrupt context when an
EmergencyLimit or StallSensor fires.

`polarity` (NO/NC) and `pullMode` are **orthogonal**:

- `polarity` drives the **GPIO interrupt edge**: NO → Rising, NC → Falling.
- `pullMode` drives the **GPIO pull direction**: UP, DOWN, or NONE.

`McU` / `Polarity` infer the pull from `polarity` (NO→down, NC→up).
`InternalPullUp` / `InternalPullDown` force the pull regardless of
polarity — needed for fail-safe wiring schemes where the polarity
setting required for edge detection doesn't match the required pull
direction (e.g. NC-to-GND switch read active-HIGH:
`polarity=NormallyOpen` + `InternalPullUp`).

`TravelLimit` rows are direction-tagged. The axis halts only when
moving in the matched direction, leaving the host free to back away.

`TravelLimit` activation is **ISR-edge-latched** since 1.0.2: a single
asserting edge (rising for `NormallyOpen`, falling for
`NormallyClosed`) sets `stableActive=true` on the next service tick,
independent of polling cadence. Deactivation stays polled
level-debounce — the lib only un-asserts after the pin reads inactive
for `debounceMs` continuously, so release-bounce can't flicker the
latch off. `EmergencyLimit` and `StallSensor` continue to call
`engineForIsr_->stop(Immediate)` directly from the ISR (engine
reference is required for those two; `TravelLimit` works with
`engineForIsr == nullptr` because its stop is issued task-side from
`MotorAxis::pumpLimits`).

### Boot-time level seed and host-side pre-flight check

`LimitSystem::begin()` reads every `TravelLimit` pin once after
attaching the ISR. Pressed at boot → `stableActive=true` immediately,
so `MotorAxis::armJog` / `armMove` refuse to drive INTO it with
`ErrorCode::LimitActive`. This is what catches the "powered off at
end-of-line, powered back on" case: the lib will not start motion
that would crash past the already-asserted switch.

That seed read happens at exactly one instant, though. On a floating
pin (ESP32 GPIO 34-39, no internal pull-up/down) or during mid-bounce
at the exact microsecond of the read, the seed can miss. For
safety-critical axes the host should query the limit system itself
before the first motion command:

```cpp
if (limits.isActive(LimitKind::TravelLimit, Direction::Forward)) {
    // Forward-side TravelLimit is currently asserted - refuse to
    // start, log a meaningful diagnostic, and either bail out or
    // route into a recovery routine that jogs Backward only.
    return /* host-level error */;
}
const auto s = axis.moveForward();
```

The lib will reject the verb internally even without this check
(`ErrorCode::LimitActive`), but doing the check at the host level
gives a place to emit the right diagnostic, hands the recovery
decision to the host, and prevents tight retry loops against an
asserted limit. Always do this in `setup()` (or whichever boot
routine arms the axis) when the axis could plausibly be parked at
an end-stop between power cycles.

## Homing (`homing/home_to_limit_strategy.h`)

```cpp
class HomeToLimitStrategy : public IHomingStrategy {
public:
    explicit HomeToLimitStrategy(ILimitSystem &limits);
    Status start(MotorAxis &axis) override;
    void tick(MotorAxis &axis, int64_t nowMs) override;
    bool isActive() const override;
    bool succeeded() const override;
    StopReason failureReason() const override;
};
```

Single-phase backward-seek homing (drives toward the configured
direction until `LimitKind::HomeSensor` activates, then declares
position zero). Multi-pass refinement (fast approach, backoff, slow
re-approach) is a documented follow-up; the current implementation
covers typical lead-screw / belt setups.

## Diagnostics (`motor_diagnostics.h`)

```cpp
struct MotorDiagnostics {
    MotorAxisId axisId;
    MotorState  state;
    Position    commandedPosition;
    uint32_t    currentSps;
    uint32_t    targetSps;
    int32_t     stepsToTarget;
    StopReason  lastStopReason;
    FaultCode   lastFault;
    uint32_t    totalStepsIssued;
    bool        homed;
    DriverIdentity identity;

    bool        stall_valid;
    uint8_t     stallSensitivityPct;
    uint8_t     stallReadingPct;   // 0=free, 100=fully loaded
    uint32_t    stallHitsSinceClear;

    bool        adaptive_current_valid;
    uint16_t    adaptiveCurrentMa;

    const char *driverRawDiagnostics;
};

// Free function — serialises the snapshot to a JSON string in `out`.
size_t toJson(const MotorDiagnostics &, char *out, size_t outSize);
```

Flat struct, value-copy semantics. `toJson` writes a stable
representation suitable for serial / web UIs.

## Identity (`driver_identity.h`)

```cpp
struct DriverIdentity {
    const char *vendor = "Unknown";
    const char *model  = "Unknown";
    uint8_t     firmwareMajor = 0;
    uint8_t     firmwareMinor = 0;
    uint32_t    rawId = 0;          // chip-side raw byte capture
};
```

Available via `MotorAxis::identity()` or directly on the driver
(`driver.identity()`). The host never assigns these; the driver
either reads them from the chip (TMC2209 IOIN, RMD 0x12 reply) or
hardcodes them (YPMC, generic STEP/DIR).
