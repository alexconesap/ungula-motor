# `lib_motor` API reference

Public surface of UngulaMotor 1.10.0. Headers, types, signatures,
lifecycle and per-driver constraints are all here, so an agent can wire
an axis from this file alone — no need to read `README.md` or the
sources. `README.md` is the human-facing version of the same ground.

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
    const T &value() const &;
    T       &value() &;
    T takeValue();                        // moves out; do not reuse after
    template <typename U = T> U valueOr(U fallback) const;  // copyable T only
};
```

Every method that can fail returns `Status` or `Result<T>`. The host
checks `.ok()` and reads `.error()` to discriminate. There is no
exception path.

## Foundation types (`motor_units.h`)

```cpp
constexpr uint8_t GPIO_NONE = 0xFF;   // "no pin wired" in any pin field

using Position    = int32_t;   // signed step position
using StepCount   = int32_t;
using Velocity    = uint32_t;  // steps per second (SPS)
using Microseconds = int64_t;

enum class Direction : uint8_t { Forward = 0, Backward = 1 };
constexpr Direction reverseOf(Direction);

struct MotorAxisId {
    uint8_t value = 0;
    constexpr MotorAxisId() = default;
    constexpr explicit MotorAxisId(uint8_t);
};
constexpr bool operator==(MotorAxisId, MotorAxisId);
constexpr bool operator!=(MotorAxisId, MotorAxisId);
```

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
Status disable();      // de-energise.
Status clearFault();   // recover from Faulted / EmergencyStopped back
                       //   to Disabled.
```

State rules, exactly as implemented:

| Method | Accepted from | Result elsewhere |
| --- | --- | --- |
| `begin()` | `Uninitialized` only | `AlreadyInitialized` |
| `enable()` | `Disabled`, `Idle` | `NotInitialized` from `Uninitialized`; `DriverFault` from `Faulted` / `EmergencyStopped`; `InvalidState` otherwise |
| `disable()` | any state except `Uninitialized` | `NotInitialized`. From `Moving` / `Jogging` / `Homing` it calls `stop()` first, then de-energises and lands in `Disabled` |
| `clearFault()` | `Faulted`, `EmergencyStopped` | `InvalidState` |

`clearFault()` also drains the limit system's pending emergency / stall
latches and resets the stall-hit counter, so a fault cannot immediately
re-fire on the next motion command.

### Motion verbs

```cpp
Status moveForward (StallPolicy = StallPolicy::Fault); // indefinite jog, +direction
Status moveBackward(StallPolicy = StallPolicy::Fault); // indefinite jog, -direction
Status moveTo(DistanceValue, StallPolicy = StallPolicy::Fault); // absolute target
Status moveBy(DistanceValue, StallPolicy = StallPolicy::Fault); // relative move
Status home();                     // run the configured homing strategy
Status declareHomeHere();          // zero the position where the axis stands
Status stop();                     // controlled decel
Status softStop();                 // gentle rampdown (jog cancel)
Status emergencyStop();            // hard halt
```

**`declareHomeHere()`** gives the same post-condition as a successful
`home()` — position zeroed, `isHomed()` true — without moving. Use it when
the carriage is already known to sit on its reference (a jog deliberately
run into the backward hard stop, or an operator calibrating by eye).
Returns `MotionInProgress` while a motion is in flight, and propagates
whatever the driver's `resetPosition(0)` returned.

**`stop()` vs `softStop()`.** Both abort an active homing strategy and both
route the axis through `Stopping` back to `Idle` on a later service tick.

- `stop()` asks the driver for `StopMode::Decelerate`; when the driver
  reports `Unsupported` (which both shipped step-signal generators do) the
  axis falls back to `StopMode::Immediate`. In practice this is a hard halt
  on STEP/DIR drives.
- `softStop()` asks the driver for `decelStop(rate)` — a real rampdown
  spliced onto the in-flight move. The rate is `4 x` the configured
  acceleration rate, i.e. a stop in a QUARTER of the acceleration ramp
  TIME, unless a distinct deceleration rate was resolved at `begin()`
  (`cfg.limits.decel` different from `cfg.limits.accel`), in which case
  that rate is used as-is. With no ramp configured at all it degrades to a
  hard stop.

> Caveat: `setAcceleration()` and `setProfile()` overwrite BOTH the accel
> and the decel rate with the same value, so any distinct deceleration
> configured through `cfg.limits.decel` is lost the first time either
> setter runs, and `softStop()` reverts to the `4 x accel` rule.

Both return `Ok()` and do nothing when the axis is `Idle` or `Disabled`,
and `InvalidState` from `Faulted` / `EmergencyStopped` / `Uninitialized`.
`emergencyStop()` is accepted from every state and always returns `Ok()`.

**`StallPolicy` — what a stall MEANS for this move.** StallGuard cannot tell
"reached the mechanical hard stop" from "something jammed"; only the caller
knows which it expects, so it is declared per move.

```cpp
enum class StallPolicy : uint8_t {
    Fault = 0,     // default & historical: a stall aborts the move, axis -> Faulted
    AcceptAsEnd,   // a stall is a legitimate end for THIS move, axis -> Idle
};
```

`moveBy` / `moveTo` with `AcceptAsEnd` is the **"travel at most this far, but
stop early if you hit the end"** primitive — the distance bounds the motion,
the hard stop terminates it, whichever comes first wins. Read
`lastStopReason()` to find out which did:

| `lastStopReason()` | Meaning |
| --- | --- |
| `TargetReached` | the full commanded distance was travelled |
| `StallDetected` | the hard stop arrived first |

Use it on machines with no limit switches: a run to end-of-stroke, or a
travel-to-hard-stop pass. Without it every such move latches a fault the host
must clear on every cycle.

The policy applies **only to the move it was armed with** — it is reset to
`Fault` on every motion end, so a stale `AcceptAsEnd` can never leak forward and
silently swallow a real jam.

> `AcceptAsEnd` only stops the LIBRARY from faulting. It does not assert the
> stall happened where you wanted. A host that knows its stroke length should
> still check the end position and raise its own obstruction error when the
> stall came early — the library cannot know what "early" means.

Homing is unaffected: a stall while `state == Homing` has always been treated as
the expected home trigger, regardless of policy.

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

Three things worth knowing before you rely on these:

- `setAcceleration(a)` writes the resolved rate into BOTH the acceleration
  and the deceleration knob. There is no separate `setDeceleration`; a
  distinct decel can only be set once, through `cfg.limits.decel`, and only
  survives until the first `setAcceleration` / `setProfile` call.
- `setProfile(p)` applies `p.maxSpeed` (when non-zero) and `p.accel` (when
  non-zero or `NoRamp`), then re-applies the intent. **`p.decel` is
  ignored.** Set deceleration through `cfg.limits.decel` at construction if
  you need it asymmetric.
- `setIntent` / `setProfile` return `Ok()` even when the driver answers
  `IntentSupport::Unsupported` or `Conflicted` — the axis discards the
  driver's answer. Call `driver.applyIntent(...)` directly if the host
  needs to know whether the chip honoured the request.

### Service tick

```cpp
void service(int64_t nowMs);
```

Host calls this from its `loop()` or its own FreeRTOS task. The lib
spawns no internal task. `nowMs` is the host's millisecond clock
(`ungula::core::time::millis()` is the canonical source).

`service()` MUST be called from exactly one task; the FSM is not
internally synchronised against itself.

> `nowMs` must come from the SAME clock as `ungula::core::time::millis()`.
> The axis stamps the limit system's stall arm-window with
> `ungula::core::time::millis()` internally at every motion start, while
> the window is *checked* against the `nowMs` you pass here. Feed
> `service()` a different time base (an application uptime counter, a tick
> count) and the stall arm window and the limit debounce both misbehave.

### Queries

```cpp
MotorState       state() const;
StopReason       lastStopReason() const;
Position         positionSteps() const;
float            position(DistanceUnit unit) const;
uint32_t         currentSps() const;        // live rate, 0 when stopped
uint32_t         actualCruiseSps() const;   // quantized steady-state rate
MotorDiagnostics diagnostics() const;
DriverIdentity   identity();                // NOT const
bool             isHomed() const;
bool             isMotionInFlight() const;  // true between arm and motion-done
const MotorAxisConfig &config() const;
```

`positionSteps()` is the open-loop commanded position. `position(unit)`
converts using `cfg.units` and returns the raw step count if the requested
unit isn't configured (e.g. asking for `Mm` when `stepsPerMm` is zero) —
it does NOT return 0.0 and it does NOT report an error, so validate
`cfg.units` yourself if a wrong-unit read would be dangerous.

`actualCruiseSps()` is the rate the NEXT `moveTo` / `moveBy` /
`moveForward` will actually reach, after the planner quantizes the request
to a half-period the generator's timer can hit. Always `<=` the requested
speed. Use it for honest "commanded speed" displays instead of echoing the
un-quantized `setSpeed()` argument. Drivers with no host-visible timer
(RMD CAN) return the un-quantized resolved cruise.

`diagnostics()` returns a snapshot suitable for serial / web logging
(see `motor_diagnostics.h`; the `toJson()` free function serialises it).
On TMC2209 with a DIAG pin wired, `diagnostics()` performs a blocking UART
register read — do not call it from a hot loop.

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

**All three take INTEGER arguments.** Speed and Acceleration are
`uint32_t` (magnitude only — direction is carried separately); Distance is
`int32_t` (sign = direction). Passing a float literal compiles through an
implicit narrowing conversion and silently truncates, so write `mm(50)`,
not `mm(50.0f)`. Hosts that need sub-unit precision pick a finer unit
(`Um` instead of `Mm`, `MmPerSec` instead of `CmPerSec`).

```cpp
struct Speed {                      // uint32_t value + SpeedUnit tag
    static constexpr Speed stepsPerSec(uint32_t);
    static constexpr Speed rpm(uint32_t);
    static constexpr Speed degreesPerSec(uint32_t);
    static constexpr Speed mmPerSec(uint32_t);
    static constexpr Speed cmPerSec(uint32_t);
    static constexpr Speed cmPerMin(uint32_t);   // needs stepsPerMm
};

struct DistanceValue {              // int32_t value + DistanceUnit tag
    static constexpr DistanceValue steps(int32_t);
    static constexpr DistanceValue revolutions(int32_t);
    static constexpr DistanceValue degrees(int32_t);
    static constexpr DistanceValue mm(int32_t);
    static constexpr DistanceValue cm(int32_t);
    static constexpr DistanceValue um(int32_t);  // micrometres; 1 mm = 1000 um
};

struct Acceleration {               // uint32_t value + AccelUnit tag
    static constexpr Acceleration stepsPerSecSquared(uint32_t);
    static constexpr Acceleration rpmPerSec(uint32_t);
    static constexpr Acceleration degreesPerSecSquared(uint32_t);
    static constexpr Acceleration mmPerSecSquared(uint32_t);
    static constexpr Acceleration rampMs(uint32_t);  // time 0->cruise, in ms
    static constexpr Acceleration noRamp();          // explicit "no ramp"
};

enum class SpeedUnit    : uint8_t { StepsPerSec, Rpm, DegreesPerSec,
                                    MmPerSec, CmPerSec, CmPerMin };
enum class DistanceUnit : uint8_t { Steps, Revolutions, Degrees, Mm, Cm, Um };
enum class AccelUnit    : uint8_t { StepsPerSecSquared, RpmPerSec,
                                    DegreesPerSecSquared, MmPerSecSquared,
                                    RampMs, NoRamp };
```

All three carry the value + the unit tag. The axis converts at the
boundary using `cfg.units`. Pass any unit that fits your domain; the
conversion is float internally (the mechanical factors are fractional) but
lands on an integer SPS / step count, so RPM-based motors and mm-based
linear axes use the same code path.

`Acceleration::noRamp()` is the sentinel for "explicitly disable ramping":
the planner emits a single cruise segment and the motor jumps to cruise on
the first edge. It exists because `MotionProfile` reserves a plain
`value == 0` for "no override, leave the current accel alone" — without the
sentinel a host could not express "I want zero accel" through a profile.
Direct `setAcceleration()` callers can pass either `noRamp()` or
`stepsPerSecSquared(0)` and get the same effect. No ramp is safe at low
step rates; expect lost steps if paired with a high cruise rate.

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
    Speed        maxSpeed = Speed::stepsPerSec(0);
    Acceleration accel    = Acceleration::stepsPerSecSquared(0);
    Acceleration decel    = Acceleration::stepsPerSecSquared(0);
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

`begin()` resolves `limits` (or the profile's overrides, when non-zero)
into SPS and returns `InvalidConfig` if the cruise speed resolves to zero —
a zero-cruise motion can never progress. Acceleration and deceleration ARE
allowed to resolve to zero; the planner reads that as "no ramp".

## `MotionProfile`

```cpp
struct MotionProfile {
    Speed        maxSpeed = Speed::stepsPerSec(0);
    Acceleration accel    = Acceleration::stepsPerSecSquared(0);
    Acceleration decel    = Acceleration::stepsPerSecSquared(0);
    MotorIntent  intent   = MotorIntent::Default;

    static constexpr MotionProfile cruise();  // intent = Default
    static constexpr MotionProfile jog();     // intent = Quiet
    static constexpr MotionProfile home();    // intent = Precision
};
```

The three factories differ ONLY in the intent they carry — they leave
`maxSpeed` / `accel` / `decel` at zero, which the axis reads as "no
override, keep the axis's configured `MotorLimits`". `cruise()` is the
right default for `moveTo` / `moveBy`.

Each speed / accel knob has three encodings:

| Field value | Meaning |
| --- | --- |
| `value > 0` | use this value |
| `value == 0`, unit not `NoRamp` | no override, leave the current setting alone |
| `unit == AccelUnit::NoRamp` | explicitly disable ramping |

Two behaviours to plan around:

- **`decel` is not applied by `setProfile()`.** Only `maxSpeed` and
  `accel` are; the deceleration knob is written from `accel` (see
  `setAcceleration`). A profile carrying an asymmetric decel is silently
  ignored.
- Switching profiles mid-motion does not reshape the running move. The new
  profile takes effect on the next motion command.

Profile-level `intent` overrides the axis-level intent whenever it is not
`Default`, for as long as the profile is active.

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

`IntentSupport::Conflicted` has exactly one producer: `Tmc2209Driver`
returns it when an intent containing `HighTorque` arrives AFTER `begin()`
on a driver with a DIAG pin wired. SpreadCycle switches off StallGuard4, so
the driver refuses the flip, writes nothing, and leaves `activeIntent_`
unchanged. Asking for the same combination BEFORE `begin()` is not
conflicted at apply time — `begin()` itself then fails with
`InvalidConfig`.

Remember that `MotorAxis::setIntent()` throws the driver's answer away and
returns `Ok()` regardless. Call `driver.applyIntent(...)` directly if the
result matters.

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

## `IMotorDriver` — the driver contract

Writing a new chip driver means implementing this and nothing else.
Everything arrives already resolved to the step domain; the axis does unit
conversion and range checking before it calls down.

```cpp
struct DriverMotionStatus {
    bool       running = false;
    bool       faulted = false;
    StopReason finishedReason = StopReason::None;
};

class IMotorDriver {
public:
    virtual Status begin() = 0;
    virtual Status enable() = 0;
    virtual Status disable() = 0;
    virtual Status clearFault() = 0;

    virtual Status armMove(Direction dir, uint32_t targetSteps,
                           uint32_t cruiseSps, uint32_t accelSps2,
                           uint32_t decelSps2) = 0;
    virtual Status armJog(Direction dir, uint32_t cruiseSps,
                          uint32_t accelSps2) = 0;
    virtual Status stop(StopMode mode) = 0;
    virtual Status decelStop(uint32_t decelSps2);   // default: stop(Immediate)

    virtual DriverMotionStatus motionStatus() const = 0;
    virtual Position commandedPositionSteps() const = 0;
    virtual uint32_t commandedSpsNow() const = 0;
    virtual uint32_t timerResolutionHz() const;     // default 0 = "unknown"
    virtual Status   resetPosition(Position newSteps) = 0;

    virtual DriverIdentity identity() = 0;
    virtual IntentSupport  applyIntent(MotorIntent) = 0;
    virtual void fillDriverDiagnostics(MotorDiagnostics &out) const = 0;
};
```

The axis polls `motionStatus()` every service tick and drives its FSM off
`running`. **A driver MUST eventually report `running == false`**, either
because the move finished or because `stop()` was called — the axis has no
timeout of its own and will sit in `Moving` forever otherwise.

`timerResolutionHz()` returning 0 means "no host-visible timer"; the axis
then reports the un-quantized cruise from `actualCruiseSps()`.

## Drivers

### TMC2209 (`drivers/tmc2209/`)

```cpp
namespace tmc = ungula::motor::tmc2209;

// NOTE: the enumerator VALUES are the chip's MRES field, which runs
// backwards: x256 = 0, x128 = 1, ... x1 = 8. Never assume x1 == 0 or that
// the values are sequential in the human-readable direction.
enum class tmc::MicrostepDepth : uint8_t {
    x1 = 8, x2 = 7, x4 = 6, x8 = 5, x16 = 4,
    x32 = 3, x64 = 2, x128 = 1, x256 = 0,
};
constexpr uint32_t tmc::microstepMultiplier(MicrostepDepth);  // 1, 2, ..., 256

struct tmc::Tmc2209Config {
    uint8_t        slaveAddress = 0;          // 0..3, rejected above 3
    uint8_t        enablePin = GPIO_NONE;
    bool           enableActiveLow = true;
    uint16_t       runCurrentMa = 800;        // 0 rejected at begin()
    uint16_t       holdCurrentMa = 400;       // > runCurrentMa rejected
    float          senseResistorOhms = 0.11f; // <= 0 rejected
    bool           useHighSensitivity = false;
    MicrostepDepth microsteps = MicrostepDepth::x16;
    bool           interpolate = true;
    // Chopper (CHOPCONF). Validated at begin().
    uint8_t        toff = 5;                  // 0..15
    uint8_t        blankTimeClk = 24;         // must be 16 / 24 / 36 / 54
    // StealthChop chopper baseline (PWMCONF), written at every begin()
    // so a warm boot can't inherit stale coefficients.
    bool           pwmAutoscale = true;
    bool           pwmAutograd = true;
    uint8_t        pwmFreq = 1;               // 0..3
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
    uint32_t         stallMinStepRateSps = 0;  // 0 = armed at any velocity
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

    // Retune StallGuard4 at runtime (one SGTHRS datagram, ~1 ms; safe
    // while moving). SG_RESULT rises with velocity, so a threshold
    // tuned at low speed under-triggers at high speed — a host that
    // varies speed should keep a speed→sensitivity table and call this
    // on every speed change. Returns NotInitialized before begin() and
    // InvalidConfig in SpreadCycle (StallGuard4 needs StealthChop).
    Status           setStallSensitivity(StallSensitivity);
    StallSensitivity stallSensitivity() const;

    // Re-gate StallGuard on a minimum STEP rate at runtime (one TCOOLTHRS
    // datagram, ~1 ms; safe while moving; skipped when the value is
    // unchanged). Returns NotInitialized before begin(); Ok and no-op
    // when no DIAG pin is wired.
    Status   setStallMinStepRate(uint32_t sps);
    uint32_t stallMinStepRate() const;

    // Blocking register dump for stall tuning (7 reads, see note below).
    Result<StallSnapshot> readStallSnapshot();

    // Static helpers exposed for tests / diagnostics:
    static uint8_t  milliampsToCs(uint16_t rmsMa, float rSenseOhms, bool useHighSensitivity);
    static uint16_t csToMilliamps(uint8_t cs, float rSenseOhms, bool useHighSensitivity);

    // ... IMotorDriver methods ...
};
```

**`stallMinStepRateSps` / `setStallMinStepRate` is the knob that stops
every bounded move from false-tripping on its own deceleration ramp.**
StallGuard4 measures back-EMF, so `SG_RESULT` collapses toward zero as the
motor slows — indistinguishable from a jam. The chip gates detection on
`TSTEP < TCOOLTHRS`, and the driver converts this field into that register.
With the default `0` the gate is wide open and *every* `moveTo` / `moveBy`
reports a stall as it stops. Any host that combines bounded moves with
stall detection must set a non-zero value: below the slowest speed it ever
commands, but well above zero. Setting a rate above the commanded cruise
disables detection for that move, which is the correct behaviour below the
speed where StallGuard4 is trustworthy at all.

`setStallSensitivity` is the companion knob: `SG_RESULT` rises with
velocity, so one threshold tuned at low speed under-triggers at high speed.
Hosts that vary speed keep a speed-to-sensitivity table and call both
setters whenever the cruise changes.

```cpp
struct tmc::Tmc2209Driver::StallSnapshot {
    // Write-only on the chip; the lib reports what it last wrote.
    uint8_t  sgthrsWritten;
    uint16_t diagTripsBelowSgResult;  // = sgthrsWritten * 2
    uint32_t tcoolthrsWritten;
    uint32_t tcoolthrsClkCycles;      // = tcoolthrsWritten & 0xFFFFF
    uint32_t coolconfWritten;
    // Live reads.
    uint32_t sgResultRaw;
    uint16_t sgResult;                // = sgResultRaw & 0x3FF; 0 = stalled
    uint32_t gconf, chopconf, ioin, tstep, drvStatus, ifcnt;
    // Decoded.
    bool diagLevelHigh;               // IOIN bit 4
    bool stealthChopActive;           // GCONF bit 2 clear; StallGuard4 needs it
    bool ok;                          // true only if every chip read succeeded
};
```

SGTHRS, TCOOLTHRS, COOLCONF and TPWMTHRS are **write-only** on this chip —
reading them returns 0 — so the driver caches what it wrote and the
snapshot reports the cache. Everything else comes straight off the chip.
`readStallSnapshot()` issues 7 read datagrams and blocks for the duration;
budget tens of milliseconds and keep it out of any hot loop. `ok == false`
means at least one read failed; the fields that did land are still valid.

Concrete transport: `tmc::Tmc2209UartTransport(ungula::hal::uart::Uart &)`.

```cpp
class IDeviceTransport {
public:
    virtual Status begin() = 0;
    virtual Status writeFrame(const uint8_t *data, size_t len) = 0;
    // Interface contract: timeoutMs == 0 means "non-blocking, return what
    // is available right now".
    virtual Result<size_t> readFrame(uint8_t *dst, size_t maxLen,
                                     uint32_t timeoutMs) = 0;
};

class tmc::Tmc2209UartTransport : public IDeviceTransport {
public:
    explicit Tmc2209UartTransport(ungula::hal::uart::Uart &);
    void     setDefaultTimeoutMs(uint32_t ms);   // ignored when ms == 0
    uint32_t defaultTimeoutMs() const;           // 50 ms out of the box
};
```

> Contract mismatch worth knowing before you write your own transport:
> `Tmc2209UartTransport::readFrame` treats `timeoutMs == 0` as "use
> `defaultTimeoutMs()`", NOT as non-blocking. `Tmc2209Driver::readRegister`
> passes 0 for both the echo drain and the reply read, so a single register
> read can block for up to twice the default timeout on a dead bus.
> Shorten it with `setDefaultTimeoutMs()` if the host cannot afford that.

The host owns the UART's baud / pin configuration; `Tmc2209UartTransport::begin()`
only flags itself ready, so call `uart.begin(...)` yourself first.
`begin()` is idempotent — two drivers on one bus each call it.

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
    // Owned RmtStepSignal settings (self-owns ctor only; see the step
    // signal generator section for what they cost and buy).
    uint32_t resolutionHz = 1'000'000u;
    bool     useDma = false;
    uint32_t dmaBufferSymbols = 1024u;
};

class stepdir::GenericStepDirDriver : public IMotorDriver {
public:
    // Self-owns:
    explicit GenericStepDirDriver(GenericStepDirConfig);

    // Pluggable:
    GenericStepDirDriver(GenericStepDirConfig,
                         IStepSignalGenerator &, IMotionPlanner &);

    IStepSignalGenerator *stepSignalForLimits();
    bool stepSignalUsingDma() const;   // valid after begin(); log it
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

`secondaryDirPin` drives a second drive off the same STEP train, each with
its own DIR pin. `secondaryDirInverted = true` gives drive B the opposite
electrical level so a face-to-face mounting turns the shaft the same
physical way. Outside that setup the field stays at `GPIO_NONE`.

Brake control runs through `onBeforeArm` (release, then block for
`brakeReleaseSettleMs`) and `onMotionStopped` (block for
`brakeEngageSettleMs`, then engage); `stop(Immediate)` force-engages
regardless of `autoEngageOnMotionEnd`.

> Both settle waits are **blocking delays on the calling thread**, and
> `engageBrake()` waits BEFORE it drops the brake. That puts
> `brakeEngageSettleMs` of delay inside `MotorAxis::emergencyStop()` and
> inside the service tick's limit-halt path, during which a vertical load
> is unbraked. Set both to 0 on any axis where an emergency stop must not
> block, and manage the brake host-side.

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
    uint8_t  motorId = 1;              // 1..32; CAN ID = 0x140 + motorId
    uint32_t stepsPerRevolution = 0;   // REQUIRED; begin() rejects 0.
                                       // RMD-X8 convention: 36000 (0.01 deg
                                       // = 1 step) makes the wire units 1:1
    uint32_t commandTimeoutMs = 50;    // currently unused by the driver
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

`begin()` blocks for up to 100 ms (the timeout baked into
`canbus::rmd::readModel`, not `commandTimeoutMs`) waiting for the 0x12
reply; no reply means no RMD on this motor ID, and `begin()` returns
`ErrorCode::TransportError`. `commandTimeoutMs` is carried on the config
but no code path reads it today.

`applyIntent` returns `Unsupported` for every flag: RMD's chopper /
current settings are firmware-baked on the drive.

Two behaviours specific to this driver that differ from the STEP/DIR ones:

- **`commandedPositionSteps()` jumps to the target the instant `armMove`
  is accepted**, before the motor has moved. It tracks what was commanded
  over the wire, not where the shaft is. Read the real position with
  `ungula::canbus::rmd::readPosition` (outside the `IMotorDriver` surface)
  or a separate encoder channel.
- **`motionStatus().running` is set true by `armMove` / `armJog` and only
  cleared by `stop()` / `disable()`.** The driver does not poll the motor
  for completion, so `MotorAxis` never observes a bounded move finishing
  on its own: the axis stays in `Moving` and rejects further motion verbs
  with `InvalidState` until the host calls `stop()`. Sequence RMD moves
  with an explicit `stop()`, or watch the real position yourself and stop
  when it arrives.

`armMove` converts cruise SPS to the 0xA4 frame's uint16 **deg/s** field
by integer-dividing centideg/s by 100, so any cruise resolving below
1 deg/s lands on a max-speed of 0 and the motor will not turn. Keep
`stepsPerRevolution` and the axis's `units.stepsPerRevolution` identical —
nothing cross-checks them.

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
    // Splice a precomputed decel ramp onto the in-flight move. The ramp
    // MUST start near the current step rate for a velocity-continuous
    // handoff. Default falls back to stop(Immediate) so a caller always
    // gets a stop, never a no-op.
    virtual Status armDecelStop(const PlannedMove &decelMove);
    virtual StepSignalStatus status() const = 0;
    virtual Position commandedPosition() const = 0;
    virtual uint32_t commandedSpsNow() const = 0;
    virtual Status   resetPosition(Position) = 0;   // MotionInProgress while running
    virtual Status   clearFault() = 0;              // MotionInProgress while running
    virtual uint32_t timerResolutionHz() const = 0;
    virtual uint32_t minTimerTicks() const = 0;
    virtual bool     usingDma() const;              // default false
};

struct StepSignalStatus {
    bool       running = false;
    bool       faulted = false;
    StopReason finishedReason = StopReason::None;
};
```

**Neither shipped generator supports `stop(StopMode::Decelerate)`** — both
return `Unsupported` and leave the pulse train running. A controlled
rampdown goes through `armDecelStop()` instead (which is what
`MotorAxis::softStop()` -> `IMotorDriver::decelStop()` uses). `MotorAxis::stop()`
tries `Decelerate`, sees `Unsupported`, and re-issues `Immediate`.

`resetPosition()` and `clearFault()` refuse with `MotionInProgress` while
the generator is running. Stop first.

## Planned moves (`motion_segment.h`)

What the planner produces and the generator executes.

```cpp
struct MotionSegment {
    uint32_t stepCount = 0;        // 0 = unused slot
    uint32_t halfPeriodTicks = 0;  // in generator-resolution ticks
};

constexpr uint8_t MAX_PLANNED_SEGMENTS = 32;

struct PlannedMove {
    Direction direction = Direction::Forward;
    uint32_t  totalSteps = 0;      // MUST equal sum(segments[i].stepCount)
    uint8_t   segmentCount = 0;
    uint32_t  cruiseSps = 0;
    MotionSegment segments[MAX_PLANNED_SEGMENTS]{};
};

struct PlannerLimits {
    uint32_t maxVelocitySps = 0;
    uint32_t accelSpsPerSec = 0;   // 0 on EITHER knob = no ramp at all
    uint32_t decelSpsPerSec = 0;
    uint32_t hardStepRateCeilingSps = 0;
    uint32_t minPulseHighUs = timing::kDefaultMinPulseHighUs;
    uint32_t minPulseLowUs  = timing::kDefaultMinPulseLowUs;
};
```

One step = two timer alarms (rising edge, falling edge), so a segment of
N steps consumes 2N alarms at `halfPeriodTicks` each. Fixed half-periods
per segment are what keep the step ISR free of floating point.

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

**Turn DMA on for any axis that sustains a high step rate.** By default
the CPU refills the channel's hardware block from an ISR — 64 symbols at
500 kSPS is a refill every 128 µs for the whole move, and WiFi on core 0
or a flash cache miss is enough to starve it and glitch the step train.
On parts with `SOC_RMT_SUPPORT_DMA` (ESP32-S3/C6/H2) GDMA can feed the
channel from a RAM ping-pong buffer instead:

```cpp
RmtStepSignal::Config cfg;
cfg.resolutionHz = 10'000'000u;
cfg.useDma = true;             // ignored on the classic ESP32 (no DMA-capable RMT)
cfg.dmaBufferSymbols = 1024u;  // 512 per half → ~1 ms refill deadline at 500 kSPS
RmtStepSignal signal(cfg);
```

Only the **last** TX channel of a group is DMA-capable, so at most one
axis per group gets DMA — give it to the fastest one and `begin()` that
one first. `begin()` returns `DriverFault` if the DMA channel is gone
rather than falling back to a CPU-refilled channel, because a silent
fallback would only surface later as a glitch at full speed.
`usingDma()` reports what the channel actually got — it is on
`IStepSignalGenerator` (default `false`), and hosts that own the driver
rather than the generator read it through
`GenericStepDirDriver::stepSignalUsingDma()`. Log it after `begin()`:

```cpp
if (!driver.stepSignalUsingDma())
        // axis is on the CPU refill path — healthy now, glitches at speed
```

`dmaBufferSymbols`
must be even, ≥ `SOC_RMT_MEM_WORDS_PER_CHANNEL` and ≤ 2046, else
`InvalidConfig`.

One cost worth knowing: the encoder runs up to `dmaBufferSymbols` steps
ahead of the STEP pin, so an abrupt `stop()` leaves
`commandedPosition()` up to that many steps past the truth. Natural
completions and decel stops drain the buffer and stay exact.

### `GptimerStepSignal` (fallback)

ESP32 gptimer with ISR rearm. Practical ceiling around 100 kSPS
(the alarm ISR has to fire per pulse; CPU runs out at higher rates).

The timer comes FIRST in the constructor — the generator does not own it:

```cpp
ungula::hal::timer::drivers::HwTimer timer;
GptimerStepSignal signal(timer, { /*timerResolutionHz=*/1'000'000u,
                                  /*timerMinTicks=*/5u });
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
    // Decel-only ramp from currentSps down to zero. Default returns an
    // empty move, which makes the generator fall back to a hard halt.
    virtual PlannedMove planStop(Direction dir, uint32_t currentSps,
                                 const PlannerLimits &limits,
                                 uint32_t timerResolutionHz,
                                 uint32_t minTimerTicks) const;
};

class TrapezoidalPlanner : public IMotionPlanner {
public:
    static constexpr uint8_t MAX_RAMP_SUB_SEGMENTS = 14;

    // Actual step rate the planner will emit for a request at a given
    // generator resolution. Always <= requestedSps (the half-period is
    // ceiling-rounded, so the motor never runs faster than asked).
    static uint32_t actualSpsFor(uint32_t requestedSps, uint32_t resolutionHz);
};
```

`TrapezoidalPlanner` is the default: stateless, integer-only (one integer
sqrt), up to 14 accel + 1 cruise + 14 decel sub-segments inside the
32-segment array, with an exact step-count guarantee —
`sum(segments[i].stepCount)` always equals the requested distance.
Rounding error is folded into the cruise segment for trapezoidal profiles
and into the last decel sub-segment for triangular / stop ramps.

Shapes it picks, in order:

| Condition | Result |
| --- | --- |
| distance < 4 steps | one constant-velocity segment at the capped speed, no ramp |
| `accelSpsPerSec == 0` OR `decelSpsPerSec == 0` | one cruise segment covering the whole distance, no ramp either end |
| accel + decel distance fits the move | trapezoidal: ramp up, cruise, ramp down |
| it does not fit | triangular: peak velocity reduced so accel + decel exactly cover the distance |

Peak velocity is capped by whichever is smaller: `maxVelocitySps`,
`hardStepRateCeilingSps`, or the rate implied by the pulse-width floor
(`max(minPulseHighUs, minPulseLowUs)`) and the generator's `minTimerTicks`.

`planJog` is a finite move under the hood: the STEP/DIR drivers pass a
`safetyCapSteps` of `0x7FFFFFFF`, so an "indefinite" jog does eventually
run out — roughly 3 h at 200 kSPS, 70 min at 500 kSPS.

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

class ILimitSystem {
public:
    virtual Status begin(const LimitWiring *wirings, uint8_t count,
                         IStepSignalGenerator *engineForIsr) = 0;
    virtual void end() = 0;
    virtual void service(int64_t nowMs) = 0;        // task context, NOT ISR

    virtual bool isActive(LimitKind kind) const = 0;
    virtual bool isActive(LimitKind kind, Direction dir) const = 0;

    // ISR latches. Each returns true exactly once per activation.
    virtual bool consumeEmergencyActivation() = 0;
    virtual bool consumeStallActivation() = 0;

    // Motion bookends — the axis calls these so the stall arm window
    // opens and closes with the move.
    virtual void notifyMotionStart(int64_t nowMs) = 0;
    virtual void notifyMotionEnd() = 0;

    virtual uint32_t totalStallHits() const = 0;
    virtual void     resetStallHitsTotal() = 0;
};

class LimitSystem : public ILimitSystem {
public:
    Status begin(const LimitWiring *wirings, uint8_t count,
                 IStepSignalGenerator *engineForIsr) override;
    // Auto-count overload: walks the array, treats pin == GPIO_NONE as
    // an unused slot. This is the one hosts want.
    Status begin(const LimitWiring (&wirings)[MAX_LIMIT_INPUTS],
                 IStepSignalGenerator *engineForIsr);

    // Diagnostics / test helpers.
    bool    isAssertedLive(LimitKind kind) const;  // raw pin read, no debounce
    uint8_t homePin() const;                       // GPIO_NONE when unwired
    void    simulateIsrEdgeForTesting(LimitKind kind);
    // ... remaining ILimitSystem methods ...
};
```

`begin()` rejects a second `HomeSensor` row or a second `StallSensor` row
with `InvalidConfig` — at most one of each per axis. `EmergencyLimit` and
`StallSensor` rows also require a non-null `engineForIsr`, because their
ISR calls `stop(Immediate)` on it directly; `TravelLimit` and `HomeSensor`
work with `nullptr`.

The stall row's `stallArmDelayMs` / `stallHitsToTrigger` are read from
whichever `StallSensor` row is configured. `debounceMs` on that row is the
level-hold time DIAG must sustain before the polled path latches a stall.

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

### What each kind actually does

| Kind | ISR attached | ISR halts the engine | Activation seen by the axis |
| --- | --- | --- | --- |
| `EmergencyLimit` | yes | **yes**, immediately | latch, drained by `consumeEmergencyActivation()` -> `EmergencyStopped` |
| `StallSensor` | yes | **yes**, after arm window + hit count | latch, drained by `consumeStallActivation()` |
| `TravelLimit` | yes (edge latch only) | no | polled `isActive(kind, dir)`; stop issued task-side |
| `HomeSensor` | yes (edge latch only) | no | polled `isActive(kind, dir)` |

**`HomeSensor` is a motion-halt input outside homing.** `armMove` /
`armJog` refuse with `LimitActive` when the home sensor guarding that
direction is asserted, and `MotorAxis::pumpLimits` halts any NON-homing
motion that runs onto it, reporting `StopReason::TravelLimit`. During
`MotorState::Homing` the axis ignores it and lets the strategy own the
stop. Treat it as "the travel limit at the home end that homing is allowed
to touch", not as an inert sensor.

### TravelLimit / HomeSensor activation: edge plus TWO active reads

Promotion to active needs all three of:

1. the ISR observed an asserting edge (rising for `NormallyOpen`, falling
   for `NormallyClosed`) at some point, AND
2. the polled read at this `service()` tick is active, AND
3. the polled read at the PREVIOUS tick was also active.

So the earliest a press can latch is the **second** service tick after the
edge, not the first. At a ~10 ms service cadence that is ~10-20 ms of extra
travel — about 250 steps at 25 kSPS. The two-tick rule is deliberate: it is
what rejects the EMI bursts that fire the ISR and happen to coincide with a
single active read. An edge followed by one inactive read disarms.

A slow fallback covers a missed edge (e.g. the switch was already pressed
before `addIsrHandler` ran): the pin reading active continuously for
`debounceMs` latches on its own.

Deactivation is polled level-debounce only — the lib un-asserts after the
pin reads inactive for `debounceMs` continuously, so release-bounce cannot
flicker the latch off.

**If you need a faster halt than two service ticks, wire the input as
`EmergencyLimit`** — that one stops the engine from the ISR with no polling
latency.

### Boot-time level seed and host-side pre-flight check

`LimitSystem::begin()` reads every `TravelLimit` and `HomeSensor` pin once
after attaching the ISR, so a switch already pressed at boot starts
`stableActive` and `armJog` / `armMove` refuse to drive into it with
`LimitActive`. That covers "powered off at end-of-line, powered back on".

The seed is a single instant, though: on a floating pin (ESP32 GPIO 34-39
have no internal pull) or mid-bounce it can miss. Query the limit system
yourself before the first motion command on any axis that could be parked
at an end-stop:

```cpp
if (limits.isActive(LimitKind::TravelLimit, Direction::Forward)) {
    return /* host-level error + recovery that jogs Backward only */;
}
const auto s = axis.moveForward();
```

The lib rejects the verb anyway; the host-level check just gives you a
place to emit a real diagnostic instead of a generic verb failure, and
stops a retry loop hammering an asserted limit.

## Homing (`homing/home_to_limit_strategy.h`)

```cpp
class IHomingStrategy {
public:
    virtual Status start(MotorAxis &axis) = 0;      // called by MotorAxis::home()
    virtual void   tick(MotorAxis &axis, int64_t nowMs) = 0;  // from service()
    virtual bool   isActive() const = 0;
    virtual bool   succeeded() const = 0;
    virtual StopReason failureReason() const = 0;
    virtual void   cancel();                        // default no-op
};

class HomeToLimitStrategy : public IHomingStrategy {
public:
    explicit HomeToLimitStrategy(ILimitSystem &limits);
    // ... all IHomingStrategy methods, including cancel() ...
};
```

Single-phase homing. `start()` short-circuits to success when
`LimitKind::HomeSensor` already reads active; otherwise it issues
`axis.moveBackward()` — **the seek direction is hardcoded Backward**,
there is no configurable homing direction. Each `tick()` polls the home
sensor; on activation it calls `axis.stop()` and reports success, and the
axis then zeroes the position, sets `isHomed()`, and emits
`HomingCompleted`. If the jog ends for any other reason (travel limit,
fault, emergency) the strategy fails with the axis's `lastStopReason()`.

Things to plan around before relying on it:

- **There is no homing timeout.** `tick()` ignores its `nowMs` argument
  entirely. If the home sensor never asserts, the backward jog runs to the
  planner's `0x7FFFFFFF`-step cap (hours) and the axis stays in `Homing`
  the whole time. A host that cannot tolerate that must supervise `home()`
  itself and call `stop()` on its own deadline.
- **Success does not re-verify the sensor.** Position is zeroed on the
  strategy's say-so; it does not re-read `isActive(HomeSensor)` at the
  moment of the reset.
- Homing accuracy inherits the sensor's two-tick promotion latency (see
  the limit section) plus the stopping distance, so the zero point is
  repeatable to within roughly one service tick of travel, not to a step.
- `MotorAxis::stop()`, `softStop()` and `emergencyStop()` all call
  `cancel()` on an active strategy, so a manual stop aborts homing rather
  than leaving it to fight the next command.

Multi-pass refinement (fast approach, backoff, slow re-approach) is a
follow-up; write another `IHomingStrategy` and pass it at axis
construction. Nothing in `MotorAxis` needs to change for it.

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
representation suitable for serial / web UIs into a caller-owned buffer;
it returns the byte count excluding the NUL, or 0 (with `out[0] = '\0'`)
when the buffer is too small. Allocation-free, `snprintf` only — safe from
any task context, NOT from an ISR.

Fields that are **not populated today** and always read zero, whatever the
driver: `stepsToTarget`, `totalStepsIssued`, `stallHitsSinceClear`. Do not
build host logic on them. For a live stall-hit count read
`ILimitSystem::totalStallHits()` instead; for TMC2209 register-level detail
use `Tmc2209Driver::readStallSnapshot()`.

`stall_valid` / `adaptive_current_valid` gate their groups: only TMC2209
sets them, and only with a DIAG pin wired (stall) or a CoolStep intent
active (adaptive current). Filling the stall group costs one blocking UART
read inside `diagnostics()`.

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
