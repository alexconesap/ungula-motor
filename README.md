# UngulaMotor (`lib_motor`)

Embedded C++17 motor and axis control for ESP32-on-Arduino-on-ESP-IDF.
One host-facing class (`MotorAxis`) drives every supported motor
family behind a verb-based API. Chip jargon (StealthChop, SpreadCycle,
CoolStep, StallGuard, register maps, RMT timing) stays inside the
driver implementation; host code does not see it.

Supported motor families:

- **TMC2209** stepper drives over UART (BTT, Watterott, FYSETC carriers).
- **YPMC + S2SVD15** STEP/DIR servo drives (RATTMOTOR).
- **Generic STEP/DIR** (DM542, TB6600, any plain STEP/DIR drive).
- **MyActuator RMD** CAN servos.

> **LLM usage note:** if this library is consumed from a coding AI workflow, explicitly point the agent to `API.md` first. `API.md` is the LLM-facing contract (public API + examples + constraints) and avoids wasting time/tokens scanning source files and this human-oriented README.

> **Warning - Active Development:** This library is under active architecture work to support multiple projects in parallel. Its structure is not finalized yet and may change without notice while this work is in progress. Updates are currently frequent (often daily). Target for structural freeze and stable `v1.0.0`: **June 2026**.

## Design

Five rules the library doesn't break:

1. **One public class.** `MotorAxis` exposes the verbs an operator
   would say out loud: `moveForward`, `moveBackward`, `moveTo`,
   `moveBy`, `home`, `stop`, `emergencyStop`, plus `setSpeed`,
   `setProfile`, `setIntent`. ~25 methods total.
2. **One internal unit.** Everything inside the lib (planner, ISR,
   pulse engine) works in steps-per-second (SPS). The public API
   takes strong-typed `Speed` / `Distance` / `Acceleration` values
   (RPM, mm/s, cm/s, deg/s, steps/s for speed; mm, cm, degrees,
   revolutions, steps for distance) and converts at the axis
   boundary using per-axis `MotorUnits`.
3. **Behaviour intents, not chip flags.** Composable
   `MotorIntent::Quiet | HighTorque | Cool | AdaptiveCurrent |
   Precision | EnergySaving` bits route to whatever each driver
   supports best. TMC2209 maps `Quiet → StealthChop`, `HighTorque →
   SpreadCycle`, `AdaptiveCurrent → CoolStep` (independent from DIAG
   wiring); YPMC and CAN-servo drivers map the same intents to whatever
   their hardware offers.
4. **One file per driver.** Writing a new chip driver is implementing
   `IMotorDriver` (15 methods) in `drivers/<chip>/<chip>_driver.{h,cpp}`,
   shipping a private `<chip>_config.h`. No configurator + stallguard +
   coolstep + kit + identity-provider zoo.
5. **Host-driven service, no heap after begin.** The lib spawns no
   internal task. The host calls `axis.service(nowMs)` from its
   `loop()` or its own FreeRTOS task at the priority it chooses. All
   buffers are fixed-size arrays sized at construction.

`lib_hal` is the only HAL the lib's public headers depend on. No
ESP-IDF or vendor SDK types appear in any header under `src/`.

## Two ways to wire a STEP/DIR driver

Every STEP/DIR driver (TMC2209, YPMC, Generic) takes two constructor
shapes:

### Self-owns

```cpp
tmc::Tmc2209Driver driver(chipCfg, transport);
```

The driver allocates an `RmtStepSignal` and a `TrapezoidalPlanner`
internally. The pin / timing fields on `chipCfg` (`stepPin`, `dirPin`,
`dirActiveHigh`, `dirSetupUs`, `minPulseHighUs`, `minPulseLowUs`)
tell the internal step signal what to do. `begin()` initialises both
internal pieces as part of its own chain. This is the form every
ordinary host wants.

### Pluggable

```cpp
RmtStepSignal      stepSignal({ /*resolutionHz=*/1'000'000u, 64u, 32u });
TrapezoidalPlanner planner;
stepSignal.begin(stepPin, dirPin, /*dirActiveHigh=*/true,
                 /*dirSetupUs=*/5u, /*minPulseHighUs=*/1u,
                 /*minPulseLowUs=*/1u);
tmc::Tmc2209Driver driver(chipCfg, transport, stepSignal, planner);
```

The host owns the step signal generator and the planner; the driver
holds references. Pin fields on the config are ignored (the external
generator already has them). Use this when:

- Tests inject a `FakeStepSignal` / `FakeMotionPlanner`.
- All RMT channels are in use by another peripheral and the host
  needs `GptimerStepSignal` instead.
- The host writes its own `IStepSignalGenerator` implementation. The
  interface is small and stable on purpose; PCNT-based, DMA-based,
  RS-485-frame-based generators are reasonable extensions.
- One planner needs to be shared across many axes (rare, but legal).

The two forms compose with the same `MotorAxis`; nothing downstream
changes.

## Examples (real hardware, real `#include`s)

Every example below is the full motor in one snippet, copy-pastable
into an ESP-IDF `main/main.cpp`. The full sketches live under
`../../../test_code/` if you want context (lifecycle, listeners, loop
structure).

### TMC2209 over UART (single motor)

```cpp
#include <ungula/core/time/time.h>
#include <ungula/hal/uart/uart.h>
#include <ungula/motor.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_driver.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_uart_transport.h>

using namespace ungula::motor;
namespace tmc = ungula::motor::tmc2209;
namespace tc  = ungula::core::time;

ungula::hal::uart::Uart    motorsUart(/*port=*/2);
tmc::Tmc2209UartTransport  tmcBus(motorsUart);

tmc::Tmc2209Config chip;
// Bus + chip
chip.slaveAddress      = 0;
chip.runCurrentMa      = 800;
chip.holdCurrentMa     = 200;
chip.senseResistorOhms = 0.11f;      // BTT TMC2209 V2.0 carrier
chip.microsteps        = tmc::MicrostepDepth::x16;
chip.interpolate       = true;
chip.diagPin           = 26;          // DIAG wired -> stall detection on
chip.stallSensitivity  = tmc::StallSensitivity::pct(50);
// or, to write a specific SGTHRS byte verbatim:
//   chip.stallSensitivity = tmc::StallSensitivity::rawSgthrs(90);
// Pins (self-owns ctor reads these to bring up its internal RMT signal)
chip.enablePin         = 5;
chip.stepPin           = 13;
chip.dirPin            = 14;

tmc::Tmc2209Driver driver(chip, tmcBus);

MotorAxisConfig axisCfg;
axisCfg.id   = MotorAxisId{ 0 };
axisCfg.units.stepsPerRevolution = 3200;  // 1.8 deg motor at 16x
axisCfg.units.stepsPerMm = 80.0f;          // 20-tooth GT2 pulley
axisCfg.limits.maxSpeed = Speed::rpm(1500.0f);
axisCfg.limits.accel    = Acceleration::rpmPerSec(6000.0f);
axisCfg.limits.decel    = axisCfg.limits.accel;

MotorAxis motor(axisCfg, driver);

void setup()
{
    motorsUart.begin(115200, /*tx=*/16, /*rx=*/17);
    motor.begin();
    motor.enable();
    motor.setIntent(MotorIntent::Quiet | MotorIntent::AdaptiveCurrent);
    motor.moveBy(DistanceValue::mm(50.0f));
}

void loop() { motor.service(tc::millis()); }
```

The driver's `begin()` reads IOIN[31:24] over UART; a `0x00` or `0xFF`
version byte (floating bus, wrong slave address, dead cable) makes
`begin()` return `ErrorCode::TransportError` so the host fails fast
instead of arming motion against a phantom drive. The legitimate
production-silicon version byte is `0x21`.

### TMC2209 stall-threshold tuning with `SG_RESULT` dump

This is the same pattern used in
`test_code/rachel_board/main/main.cpp`: configure stall either by
percentage or raw `SGTHRS`, then dump `SG_RESULT` periodically to tune
the trip point.

```cpp
#include <emblogx/logger.h>
#include <emblogx/sinks/console_sink.h>

#include <ungula/core/time/time.h>
#include <ungula/hal/uart/uart.h>
#include <ungula/motor.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_driver.h>
#include <ungula/motor/drivers/tmc2209/tmc2209_uart_transport.h>

using namespace ungula::motor;
namespace tmc = ungula::motor::tmc2209;
namespace tc = ungula::core::time;

constexpr uint8_t V90_STALL_PCT = 35U;
constexpr uint8_t V90_STALL_SGTHRS = 90U;

ungula::hal::uart::Uart motorsUart(2);
tmc::Tmc2209UartTransport tmcBus(motorsUart);

tmc::Tmc2209Config chip;
chip.slaveAddress = 1;
chip.enablePin = 32;
chip.stepPin = 18;
chip.dirPin = 25;
chip.diagPin = 23;

// Option A: host-friendly percentage -> lib maps to SGTHRS.
//chip.stallSensitivity = tmc::StallSensitivity::pct(V90_STALL_PCT);
// Option B: raw SGTHRS byte (what we use while bench tuning).
chip.stallSensitivity = tmc::StallSensitivity::rawSgthrs(V90_STALL_SGTHRS);

tmc::Tmc2209Driver driver(chip, tmcBus);

MotorAxisConfig cfg;
cfg.id = MotorAxisId{ 0 };
cfg.units.stepsPerRevolution = 3200;
cfg.limits.maxSpeed = Speed::rpm(60.0f);
cfg.limits.accel = Acceleration::rpmPerSec(120.0f);
cfg.limits.decel = cfg.limits.accel;

LimitSystem limits;
cfg.limits_wiring[0].pin = chip.diagPin;
cfg.limits_wiring[0].kind = LimitKind::StallSensor;
cfg.limits_wiring[0].stallArmDelayMs = 750;
cfg.limits_wiring[0].stallHitsToTrigger = 1;
limits.begin(cfg.limits_wiring, driver.stepSignalForLimits());

MotorAxis axis(cfg, driver, &limits);

void setup()
{
    static emblogx::ConsoleSink console;
    emblogx::register_sink(&console);
    emblogx::set_rate_limit_ms(1000);
    emblogx::init();

    motorsUart.begin(115200, /*tx=*/16, /*rx=*/17);
    axis.begin();
    axis.enable();
    axis.setIntent(MotorIntent::Quiet); // StallGuard4 needs StealthChop
    axis.moveForward();

    // One-shot register dump at boot.
    const auto r = driver.readStallSnapshot();
    if (r.ok()) {
        const auto s = r.value();
        log_info_force("V90 stall regs: SGTHRS=%u trip<=%u DIAG=%s stealth=%s IFCNT=%lu",
                       static_cast<unsigned>(s.sgthrsWritten),
                       static_cast<unsigned>(s.diagTripsBelowSgResult),
                       s.diagLevelHigh ? "HIGH" : "low",
                       s.stealthChopActive ? "yes" : "NO",
                       static_cast<unsigned long>(s.ifcnt & 0xFFu));
    }
}

void loop()
{
    const int64_t now = tc::millis();
    axis.service(now);

    static int64_t lastSgDumpMs = 0;
    if (now - lastSgDumpMs >= 1000) {
        lastSgDumpMs = now;
        const auto r = driver.readStallSnapshot();
        if (r.ok()) {
            const auto s = r.value();
            log_info_force("[V90 SG] SG_RESULT=%u (trip<=%u) DIAG=%s stealth=%s hits=%lu state=%s",
                           static_cast<unsigned>(s.sgResult),
                           static_cast<unsigned>(s.diagTripsBelowSgResult),
                           s.diagLevelHigh ? "HIGH" : "low",
                           s.stealthChopActive ? "yes" : "NO",
                           static_cast<unsigned long>(limits.totalStallHits()),
                           motorStateToString(axis.state()));
        }
    }
}
```

Tuning rule used on the bench:

- free-running motor: `SG_RESULT` usually sits high (often 700-1000),
- loaded/blocked motor: `SG_RESULT` drops,
- trip condition is `SG_RESULT < SGTHRS * 2` (`trip<=` in the dump),
- pick an `SGTHRS` that lands between free-running and loaded readings.

### Generic STEP/DIR (DM542, TB6600, ...)

```cpp
#include <ungula/core/time/time.h>
#include <ungula/motor.h>
#include <ungula/motor/drivers/generic_stepdir/generic_stepdir_driver.h>

using namespace ungula::motor;
namespace stepdir = ungula::motor::stepdir;
namespace tc      = ungula::core::time;

stepdir::GenericStepDirConfig dm;
dm.enablePin = 33;          // DM542 ENA-, active-LOW
dm.enableActiveLow = true;
dm.stepPin   = 18;
dm.dirPin    = 19;
dm.dirActiveHigh = true;
// Pulse-width defaults come from `motor_step_timing.h`
// (kDefaultDirSetupUs = 5, kDefaultMinPulseHighUs / LowUs = 1) and
// suit every drive in the lib's target list. Override only for
// older optoisolated boards that spec 2+ us pulses.

stepdir::GenericStepDirDriver driver(dm);

MotorAxisConfig axisCfg;
axisCfg.id   = MotorAxisId{ 0 };
axisCfg.units.stepsPerRevolution = 1600;     // DM542 at 8x microsteps
axisCfg.limits.maxSpeed = Speed::rpm(600.0f);
axisCfg.limits.accel    = Acceleration::rpmPerSec(2400.0f);
axisCfg.limits.decel    = axisCfg.limits.accel;

MotorAxis motor(axisCfg, driver);

void setup() { motor.begin(); motor.enable(); motor.moveForward(); }
void loop()  { motor.service(tc::millis()); }
```

`GenericStepDirDriver` reports an honest identity of `"Unknown" /
"Generic STEP/DIR"` since the drive has no readback channel. To
detect a faulted drive at boot, wire its ALM output (if it has one)
as a `LimitKind::EmergencyLimit` in `axisCfg.limits_wiring[]` and the
axis refuses motion when ALM is active.

### YPMC + S2SVD15 (single servo)

```cpp
#include <ungula/core/time/time.h>
#include <ungula/motor.h>
#include <ungula/motor/drivers/ypmc/ypmc_driver.h>

using namespace ungula::motor;
namespace ypmc = ungula::motor::ypmc;
namespace tc   = ungula::core::time;

ypmc::YpmcConfig cfg;
cfg.generic.stepPin = 18;
cfg.generic.dirPin  = 19;
cfg.generic.dirActiveHigh = false;       // S2SVD15: Forward = DIR LOW
cfg.generic.enablePin = 23;              // S2SVD15 SRV-ON, active-HIGH
cfg.generic.enableActiveLow = false;
// No tandem, no brake on this rig.
cfg.secondaryDirPin = GPIO_NONE;
cfg.brakePin        = GPIO_NONE;

ypmc::YpmcStepDirDriver driver(cfg);

MotorAxisConfig axisCfg;
axisCfg.units.stepsPerRevolution = 10'000;   // S2SVD15 factory e-gear
axisCfg.limits.maxSpeed = Speed::rpm(150.0f);
axisCfg.limits.accel    = Acceleration::rpmPerSec(600.0f);
axisCfg.limits.decel    = axisCfg.limits.accel;

MotorAxis motor(axisCfg, driver);

void setup() { motor.begin(); motor.enable(); tc::delayMs(60); motor.moveForward(); }
void loop()  { motor.service(tc::millis()); }
```

The 60 ms delay after `enable()` lets SRV-ON latch in the S2SVD15
(the drive ignores STEP pulses during that window). It's a hardware
quirk, not a lib requirement.

### YPMC tandem (Wendy catheter coil winder)

In the **Wendy** project (a catheter coil winder) two YPMC-400W
servos drive a single mandrel from opposite ends. Both motors run at
the same RPM but in mirrored electrical directions so they pull the
mandrel into tension from both sides at once. They share one STEP
pin (driven by a single RMT channel) and have independent DIR pins.
The `YpmcConfig` represents both motors as one logical `MotorAxis`:

```cpp
#include <ungula/motor.h>
#include <ungula/motor/drivers/ypmc/ypmc_driver.h>

namespace ypmc = ungula::motor::ypmc;
using namespace ungula::motor;

ypmc::YpmcConfig cfg;
// Shared STEP pin + primary DIR (drive A).
cfg.generic.stepPin = 18;
cfg.generic.dirPin  = 25;
cfg.generic.dirActiveHigh = true;
cfg.generic.enablePin = 33;
cfg.generic.enableActiveLow = false;

// Secondary DIR (drive B). Driver writes both DIR pins before each
// move, BEFORE the step signal's dirSetupUs window starts, so both
// drives see stable DIR levels when the first STEP edge arrives.
cfg.secondaryDirPin = 32;
cfg.secondaryDirActiveHigh = true;
// Face-to-face mounting: drive B sees the OPPOSITE level to produce
// the same physical shaft rotation as drive A. Set to false if both
// servos sit on the same side of the shaft.
cfg.secondaryDirInverted = true;

ypmc::YpmcStepDirDriver driver(cfg);

MotorAxisConfig axisCfg;
axisCfg.units.stepsPerRevolution = 10'000;
axisCfg.limits.maxSpeed = Speed::rpm(2500.0f);
axisCfg.limits.accel    = Acceleration::rpmPerSec(5000.0f);
axisCfg.limits.decel    = axisCfg.limits.accel;
axisCfg.limits.hardStepRateCeilingSps = 1'000'000;  // > 2500 RPM * 10000 PPR

MotorAxis mandrel(axisCfg, driver);
```

This is not a typical motor wiring. It is the project that drove the
addition of `secondaryDirPin` + `secondaryDirInverted` to the lib.
The single STEP pin keeps the two drives mechanically synchronised
to the same pulse train; the DIR polarity inversion makes them turn
the shaft the same way despite facing each other. ALM lines from the
two S2SVD15 drives, when wired, go on `axisCfg.limits_wiring[]` as
two separate `LimitKind::EmergencyLimit` entries so a fault on
either drive halts the axis.

`YpmcConfig` also carries optional brake control (`brakePin`,
`brakeReleaseActiveHigh`, `brakeReleaseSettleMs`,
`brakeEngageSettleMs`, `autoEngageOnMotionEnd`) for installations
that wire a 24 V holding-brake relay. The driver releases the brake
before motion starts and engages it on motion end (and always on
faults / emergency stop, regardless of `autoEngageOnMotionEnd`).

### Two TMC2209s on a shared UART

```cpp
ungula::hal::uart::Uart    motorsUart(2);
tmc::Tmc2209UartTransport  tmcBus(motorsUart);
motorsUart.begin(115200, /*tx=*/16, /*rx=*/17);

tmc::Tmc2209Config v90 = makeChipConfig(/*addr=*/1, /*step=*/18, /*dir=*/25, /*en=*/32);
tmc::Tmc2209Config h360 = makeChipConfig(/*addr=*/0, /*step=*/13, /*dir=*/14, /*en=*/5);

tmc::Tmc2209Driver v90Drv(v90, tmcBus);    // each driver allocates
tmc::Tmc2209Driver h360Drv(h360, tmcBus);  // its own RMT TX channel
```

Each driver allocates one RMT TX channel. The ESP32 has 8 channels;
two motors fit comfortably. The shared `Tmc2209UartTransport` is fine
under contention: the two slave addresses keep the chips separable
on the same bus.

### RMD CAN servo

The RMD-CAN driver is **optional**: lib_motor does not depend on
lib_canbus by default. To compile it, define `UNGULA_USE_CANBUS` at
build time and add lib_canbus to your project. Without the macro the
`rmd_can_driver.{h,cpp}` translation unit is empty and the symbols
are absent.

PlatformIO: `build_flags = -DUNGULA_USE_CANBUS`
CMake: `add_compile_definitions(UNGULA_USE_CANBUS)`
Arduino CLI: `--build-property "compiler.cpp.extra_flags=-DUNGULA_USE_CANBUS"`

```cpp
#include <ungula/hal/can/can.h>
#include <ungula/motor.h>
#include <ungula/motor/drivers/rmd/rmd_can_driver.h>

namespace rmd = ungula::motor::rmd;
using namespace ungula::motor;

ungula::hal::can::Can can(/*controller=*/0);  // implements lib_hal::can::ICan

rmd::RmdConfig cfg;
cfg.motorId = 1;                  // CAN ID = 0x140 + motorId
cfg.stepsPerRevolution = 36'000;  // RMD-X8: 0.01 deg = 1 step
cfg.releaseBrakeOnDisable = false;

rmd::RmdCanDriver driver(cfg, can);   // takes lib_hal::can::ICan&

MotorAxisConfig axisCfg;
axisCfg.units.stepsPerRevolution = cfg.stepsPerRevolution;
axisCfg.limits.maxSpeed = Speed::rpm(300.0f);

MotorAxis motor(axisCfg, driver);

void setup()
{
    can.begin(/*kbps=*/1000);
    motor.begin();   // sends 0x12, waits for reply -> caches identity
    motor.enable();  // 0x88 (motor running)
    motor.moveForward();
}
```

RMD drivers do not use the step signal generator: the motor has its
own onboard control loop and accepts CAN frames per the MyActuator V3
protocol. The lib translates verb commands into CAN bytes
(`armMove` to 0xA4 absolute-position, `armJog` to 0xA2 signed-speed,
`stop(Decelerate)` to 0x81, `stop(Immediate)` to 0x80, `enable` to
0x88). `begin()` blocks for up to 100 ms waiting for the 0x12 reply
and returns `TransportError` if no RMD responds.

`MotorIntent` flags report `Unsupported` on RMD: the motor's chopper
configuration and current loop live on the drive's firmware and the
host can't change them at runtime.

## Identity is per-driver, not per-developer

`MotorAxis::identity()` and `driver.identity()` both return a
`DriverIdentity { vendor, model, firmwareMajor, firmwareMinor, rawId }`.
The string fields are hardcoded by the driver, never assigned by
host code. Firmware bytes are read from the chip where there's a
readback channel:

| Driver                | Vendor      | Model              | Firmware source             |
| --------------------- | ----------- | ------------------ | --------------------------- |
| `Tmc2209Driver`       | Trinamic    | TMC2209            | IOIN[31:24] over UART       |
| `RmdCanDriver`        | MyActuator  | RMD                | 0x12 reply over CAN         |
| `YpmcStepDirDriver`   | RATTMOTOR   | YPMC + S2SVD15     | hardcoded (no readback)     |
| `GenericStepDirDriver`| Unknown     | Generic STEP/DIR   | hardcoded (no readback)     |

You can query identity directly on a driver without constructing a
`MotorAxis`:

```cpp
tmc::Tmc2209Driver d(chip, bus);
d.begin();                  // identity populated from IOIN here
auto id = d.identity();     // { "Trinamic", "TMC2209", 0x21, 0, 0x2100...0 }
```

## Position tracking

Open-loop "commanded position" only: the lib does not pull encoder
feedback into the FSM. `MotorAxis::positionSteps()` and
`MotorAxisConfig::units` (`stepsPerMm`, `stepsPerDegree`,
`stepsPerRevolution`) give you commanded position in steps and the
conversion to physical units. For closed-loop position the host
wires a quadrature encoder via `lib_hal::Quadrature` (ESP32 PCNT)
and reads it independently of the motor lib.

For `RmtStepSignal` (the default step generator for STEP/DIR drives),
the commanded position advances live as the RMT peripheral emits
symbols. The position counter is allowed to wrap past `INT32_MAX`
during a long indefinite jog and keeps counting (well-defined
modular wrap, no signed-overflow UB). At 416 kSPS the wrap point is
around 170 minutes of continuous motion.

## Per-axis limit system

Every axis carries a `MotorAxisConfig::limits_wiring[]` table of
sensors that can halt motion. Four kinds:

- `EmergencyLimit` -- hard emergency stop; refuses motion when
  active, requires `clearFault()` to recover.
- `TravelLimit` -- end-of-travel switch, direction-tagged. Halts
  motion only when moving in the matched direction; lets the host
  back away in the opposite direction.
- `HomeSensor` -- read by the homing strategy; not a motion-halt
  input. If the host system does not implement a 'homing' operation, use
  a plain `TravelLimit` instead.
- `StallSensor` -- mid-motion stall detection (typically the
  TMC2209's DIAG pin), debounced + windowed by `stallArmDelayMs` and
  `stallHitsToTrigger`.

`LimitSystem` is wired into the `MotorAxis` constructor:

```cpp
LimitSystem limits;
limits.begin(cfg.limits_wiring,
             driver.stepSignalForLimits()); // auto-count overload
MotorAxis motor(cfg, driver, &limits);
```

The `stepSignalForLimits()` pointer lets the limit system call
`stepSignal.stop(StopMode::Immediate)` from the GPIO ISR when an
emergency or stall fires. Without it the halt path is
service-tick-bound (still safe, just higher latency).

### Belt-and-suspenders: pre-flight check before the first move

`LimitSystem::begin()` does a one-shot live read of every
`TravelLimit` pin right after attaching the ISR, so a switch that is
already pressed at boot starts with `stableActive = true` and
`armJog` / `armMove` correctly refuse to drive INTO it. This covers
the realistic "powered off at end of line, powered back on" case.

The read happens at a single instant, though. On a pin with no pull
resistor (ESP32 GPIO 34-39, no internal pull available), or with a
switch that happens to be mid-bounce at the exact microsecond of the
seed read, the seed can miss. Probability is small but non-zero for
safety-critical axes.

If you want a second line of defence, query the limit system yourself
before the first motion command and refuse to start if it reports the
direction-guarding limit as active:

```cpp
if (limits.isActive(LimitKind::TravelLimit, Direction::Forward)) {
    log_error_force("Forward TravelLimit is asserted at startup; "
                    "back the axis off the end-of-line switch before "
                    "starting motion.");
    return; // or enter a recovery routine that jogs Backward only
}
const auto s = wendy->moveForward();
```

The lib will catch this case during `armJog` / `armMove` anyway
(returns `ErrorCode::LimitActive`), but a host-level check lets you
emit a meaningful diagnostic instead of just seeing a generic verb
failure, and prevents the host from going into a tight retry loop
against an asserted limit.

For axes where pre-flight position is non-deterministic (anything that
parks at an end-stop in normal operation), make this check part of
your `setup()` after `MotorAxis::begin()` and before the first
`moveForward` / `moveBackward`.

## What goes where

```text
src/ungula/motor/
  motor_axis.{h,cpp}             # the black-box class + FSM
  motor_axis_config.h            # plain-struct host config
  motion_profile.h               # cruise() / jog() / home() factories
  motor_intent.h                 # flag bits + helpers
  motor_state.h                  # 9-state FSM + StopReason / FaultCode
  motor_units.h                  # Speed / Distance / Acceleration + MotorUnits
  motor_event.h                  # MotorEvent + IMotorEventListener
  motor_diagnostics.h            # MotorDiagnostics + toJson()
  motion_segment.h               # PlannedMove + MotionSegment
  driver_identity.h              # DriverIdentity
  result.h                       # Result<T> / Status / ErrorCode

  i_motor_driver.h               # the driver contract (15 methods)
  i_device_transport.h           # UART / SPI byte-pipe abstraction
  i_step_signal_generator.h      # STEP / DIR pulse generator contract
  i_motion_planner.h             # trajectory planner contract
  i_limit_system.h               # limit / stall / home sensor system
  i_homing_strategy.h            # homing FSM contract

  planners/    trapezoidal_planner.{h,cpp}     # default planner
  step_signal/ gptimer_step_signal.{h,cpp}     # gptimer-based, <= 100 kSPS
               rmt_step_signal.{h,cpp}         # RMT-based, 500+ kSPS (default)
  limits/      limit_system.{h,cpp}            # default limit system
  homing/      home_to_limit_strategy.{h,cpp}  # backward-seek homing
  drivers/
    tmc2209/         tmc2209_config.h
                     tmc2209_driver.{h,cpp}
                     tmc2209_uart_transport.{h,cpp}
    generic_stepdir/ generic_stepdir_config.h
                     generic_stepdir_driver.{h,cpp}
    ypmc/            ypmc_config.h
                     ypmc_driver.{h,cpp}
    rmd/             rmd_config.h
                     rmd_can_driver.{h,cpp}   # adapter over lib_canbus::rmd

tests/
  test_motor_axis_fsm.cpp          # FSM transitions via fakes
  test_units_conversion.cpp        # Speed/Distance unit conversion
  test_trapezoidal_planner.cpp     # planner math
  test_tmc2209_driver.cpp          # TMC2209 driver against fakes
  test_stepdir_drivers.cpp         # generic + YPMC driver tests
  test_rmd_can_driver.cpp          # RMD-over-CAN driver tests
  fakes/                           # FakeMotorDriver + FakeLimitSystem +
                                   # FakeStepSignal + FakeTmcTransport
                                   # (RMD tests use lib_canbus's FakeCan)
```

101 host tests, all passing. Tests are platform-agnostic (no ESP-IDF
build needed) and run from `tests/build/` after `cmake --build .`.

## See also

- `API.md` -- public-API reference for `MotorAxis`, every driver
  config struct, the four pluggable interfaces, units, intents,
  events, diagnostics.
- `code/docs/ARCHITECTURE.md` -- cross-library invariants and the
  library-vs-host responsibility split.
- `code/docs/LIBRARY_VERSIONS.md` -- per-version change history.

## Acknowledgements

- Thanks to Claude and ChatGPT for helping on generating this documentation and the tests.

## License

MIT — Copyright (c) 2025-2026 Alex Conesa

---

## Arduino CLI symlink note (rarely relevant)

This library ships a flat forwarder header at `src/ungula_motor.h` that
just `#include`s `ungula/motor.h`. `library.properties` `includes=` points
at the forwarder.

It only exists to work around an Arduino CLI quirk: when the library is
consumed through a symlink, the CLI sometimes fails to discover headers
nested under `src/ungula/`. The flat forwarder fixes that scan.

**Host code keeps including the real header**:

```cpp
#include <ungula/motor.h>
```

PlatformIO, ESP-IDF component builds, and plain CMake setups can ignore
the forwarder.
