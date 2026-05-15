# TMC2209 Motor Driver Notes

## Summary

For reliable TMC2209 sensorless homing:

```text
1. Enable StealthChop.
2. Use DIAG as the stall signal.
3. Ensure the motor is moving fast enough for StallGuard to work.
4. Tune SGTHRS at the actual homing speed.
5. Increase SGTHRS to make stall detection more sensitive.
6. Decrease SGTHRS to make stall detection less sensitive.
```

## Sensorless Homing Notes

The TMC2209 can detect motor stalls using StallGuard (`SG`) and report them through the `DIAG` pin. This is commonly used for sensorless homing, where the motor moves until it mechanically reaches an end stop and the driver detects the stall.

### Required Mode: StealthChop

For stall detection on the TMC2209, `StealthChop` must be enabled.

Without `StealthChop` configured correctly, `StallGuard`-based sensorless homing may not behave as expected.

### DIAG Pin Behavior

The driver exposes stall detection through the `DIAG` pin.

When the StallGuard result falls below the configured threshold, the `DIAG` pin is asserted.

For StallGuard 4:

```text
SG_RESULT < 2 × SGTHRS
```

When this condition is true, the `DIAG` pin is set `HIGH`.

### SG_RESULT and SGTHRS

`SG_RESULT` is the measured StallGuard load value.

`SGTHRS` is the configured StallGuard threshold.

The relationship is:

```text
Lower SG_RESULT = higher detected load / closer to stall
Higher SGTHRS   = more sensitive stall detection
```

So:

```text
Increase SGTHRS → more sensitive stall detection
Decrease SGTHRS → less sensitive stall detection
```

If the `DIAG` pin triggers too early, reduce `SGTHRS`.

If the `DIAG` pin does not trigger when the motor stalls, increase `SGTHRS`.

### TCOOLTHRS / TSTEP Threshold

`TCOOLTHRS` defines the velocity range where StallGuard is active.

The relationship is inverse because `TSTEP` represents the time between steps:

`StallGuard` active when:

`TSTEP` < `TCOOLTHRS`

This means `StallGuard` is active only when the motor is moving faster than the equivalent threshold speed.

In practical terms:

```text
Shorter step interval = higher motor speed
Longer step interval  = lower motor speed
```

Therefore, `StallGuard` generally works only above a minimum speed.

If the motor moves too slowly, `StallGuard` may not detect stalls reliably.

### Speed Dependency

`StallGuard` readings are speed-dependent.

If you observe a roughly linear relationship between motor speed and the SG_RESULT values returned by the driver, tune the `StallGuard` threshold around the actual homing speed you intend to use.

Do not tune `StallGuard` at one speed and assume it will behave identically at another speed.

### Practical Tuning Rule

Start with a moderate homing speed and adjust `SGTHRS` experimentally:

```text
Motor stalls but DIAG does not trigger:
    Increase SGTHRS

DIAG triggers before reaching the end stop:
    Decrease SGTHRS

DIAG is unstable or noisy:
    Adjust speed, current, acceleration, and SGTHRS together
```
