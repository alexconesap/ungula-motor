# YPMC (RATTMOTOR) servo driver integration

Header: `ungula/motor/drivers/ypmc/ypmc_servo.h`.

Target motors: RATTMOTOR YPMC-series AC servo (`A2M-60SV01330P5Z` and
siblings), paired with the **S2SVD15** drive that ships in the same
kit. STEP/DIR interface.

## What this module provides

The S2SVD15 is a conventional industrial STEP/DIR servo controller —
it does its own position / velocity / torque loops and the host just
provides a pulse train. UngulaMotor already supports that wire shape
through `Axis::createStepDirServo`; this driver does not replace the
pulse engine, the actuator, or the axis facade.

What lives here:

- `kDefaultDriveTiming` / `kDefaultDrivePolarity` — documented
  S2SVD15 numbers (DIR setup ≥ 5 µs, STEP min-high 1 µs, max pulse rate
  500 kpps in CMD+DIR mode, SRV-ON active-HIGH, ALM− active-LOW,
  COIN active-HIGH).
- `applyDriveDefaults(StepDirServoAxisConfig&)` — populates the
  timing and polarity fields and wires `ALM` (if set) into the
  `SensorBank` as a `CrashLimit` role for sub-µs ISR halt on drive
  faults.
- `BrakeController` — coordinates a host-driven 24 V holding-brake
  relay with the axis lifecycle. Implements `IAxisEventListener` and
  auto-engages on motion-end / fault.

What does **not** live here yet:

- Modbus side-channel (RS-232 / RS-485). The kit ships without the
  CN-port cable; the basic STEP/DIR path doesn't need it. When a
  diagnostics layer is added it will land in this folder as
  `ypmc_modbus_uart.h/.cpp` + `ypmc_diagnostics.h/.cpp`, mirroring the
  `tmc2209` split.

## Wiring (S2SVD15, CN1 connector)

Mandatory (host → drive):

| Pin pair         | Purpose                                |
| ---------------- | -------------------------------------- |
| `PUL+` / `PUL−`  | STEP pulse input                       |
| `DIR+` / `DIR−`  | Direction input                        |
| `SRV-ON`         | Servo enable, active-HIGH              |

Optional (drive → host, open-collector):

| Pin       | Purpose                                       | Wire as                                |
| --------- | --------------------------------------------- | -------------------------------------- |
| `ALM−`    | Alarm, active-LOW                             | `SensorRole::CrashLimit`, NC polarity  |
| `COIN`    | In-position, active-HIGH                      | Polled via `feedback().inPosition`     |
| `BRK+/−`  | 24 V brake coil, when drive sequences brake   | Drive-managed, no host wiring         |

When the host owns the brake relay (recommended for ESP32 setups
where the drive's BRK output isn't routed through the panel), wire a
GPIO through an opto-MOSFET / SSR to the 24 V coil and use
`BrakeController` to sequence it.

## Operating modes

The S2SVD15 supports three pulse-input modes selectable from the
front-panel parameter (`P0-04`):

1. **CMD+DIR** (default) — the one this driver targets.
2. CW/CCW pulses — separate input pairs.
3. Quadrature (90°-shifted) pulses.

UngulaMotor's pulse engine emits CMD+DIR only. Configure the drive to
match before wiring.

## Tuning checklist

Drive front panel (per S2SVD15 manual; numbers vary by firmware
revision):

- `P0-02` = 0 (position mode).
- `P0-04` = 0 (CMD+DIR pulse mode).
- `P0-09` / `P0-10` = electronic gear (default 10000 pulses/rev for
  the 17-bit encoder). UngulaMotor's `stepsPerMm` / `stepsPerDegree`
  resolves to this — keep the drive's gear ratio fixed and adjust on
  the host side.
- `P0-21` = encoder-output direction (set to match `Direction::Forward`).

Drive-side alarms map to UngulaMotor as `FaultCode::LimitExceeded`
(via the actuator's `LimitSwitch` / `TravelLimit` mapping in 3.1.x);
hosts that need finer fault breakdown will need the future Modbus
diagnostics layer to read the drive's alarm register.

## Brake sequencing

Brake-release must precede the first STEP pulse with enough delay for
the mechanical brake to release (60–150 ms typical). Brake-engage
follows motion stop after a settling delay (10–50 ms).

`BrakeController` does NOT auto-release on `MotionStarted` — by the
time that event fires, the engine has already armed. Hosts call
`release()` explicitly between `enable()` and the first motion command:

```cpp
ypmc::BrakeController brake({
    .brakeReleasePin = 25,
    .releaseSettleMs = 120,
    .engageSettleMs  = 30,
});
brake.begin();
axis->subscribe(&brake);  // auto-engages on motion-end / fault

axis->enable();
brake.release();          // explicit, blocks for releaseSettleMs
axis->moveBy(10000);      // safe to step now
```

Auto-engage on motion-end is on by default. Disable it for horizontal
axes that prefer to stay free between moves:
`brake.setAutoEngage(false)`.

## Example

`examples/ypmc_servo_brake/` — minimal sketch with STEP/DIR + SRV-ON
+ ALM-as-CrashLimit + brake controller wiring.
