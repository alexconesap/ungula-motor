# UngulaMotor Driver Authoring Guide

This guide is for adding a new motor-driver integration under:

`src/ungula/motor/drivers/<vendor_model>/`

It covers both:

- STEP/DIR chip drivers (for example: new stepper driver IC over UART/SPI).
- Native bus servo drivers (for example: CAN protocol implementation).

## 1) Pick the integration path first

Use this decision tree before writing code:

1. If the drive accepts STEP/DIR pulses, do not replace the pulse engine.
   - Keep `Axis` + `StepDirActuator` + `HalPulseEngine`.
   - Add vendor-specific configuration/diagnostics modules in `drivers/<model>/`.
2. If the drive is bus-native (CAN, RS485, Ethernet, vendor fieldbus), use a protocol layer.
   - For CAN servo, implement `ICanServoProtocol` first.
   - Keep motion semantics (`Status`, `ErrorCode`, `StopMode`) consistent with `Axis`.

If you skip this step, you will end up duplicating actuator logic that already exists.

## 2) Non-negotiable architecture rules

Your new driver must follow the existing contracts:

- No timing ownership outside the pulse engine for STEP/DIR.
- No blocking calls in ISR context.
- No UART/SPI/I2C reads in the fast motion path (`Axis::service` or ISR).
- No direct Arduino API usage inside library code.
- No logging from library internals; return `Status` / `Result<T>` instead.
- Keep public APIs explicit and typed (`ErrorCode`, `FaultCode`, `StopReason`).

## 3) Create the folder scaffold

Create `src/ungula/motor/drivers/<vendor_model>/` with at least:

```text
i_<model>_transport.h          // transport abstraction
fake_<model>_transport.h       // host-test fake
<model>_hal_<bus>.h/.cpp       // HAL-backed adapter (uart/spi/can)
<model>_registers.h            // register addresses and bit masks
<model>_configurator.h/.cpp    // boot/runtime write path
<model>_diagnostics.h/.cpp     // optional read path (off motion path)
README.md                      // driver-specific wiring + tuning notes
```

For CAN-native servos, replace register files with protocol-command objects.

## 4) Implement transport abstraction first

Define a narrow interface for what upper layers need, not raw HAL details.

Example shape:

- `writeRegister(reg, value)`
- `readRegister(reg) -> Result<uint32_t>`

Then implement:

1. HAL adapter using `ungula::hal` only.
2. Fake transport with deterministic failure injection for tests.

Do not let configurator/diagnostics depend directly on `ungula::hal::<bus>` types.

## 5) Implement the configurator (write path)

The configurator sets the driver into a known good state.

Requirements:

- Validate config up front and return `InvalidConfig` on bad input.
- Apply writes in a deterministic order.
- Keep a shadow copy of relevant written fields.
- Do not perform periodic reads here.

If the chip has W1C flags, implement clear-by-write correctly and test it.

## 6) Implement diagnostics as a separate module

Diagnostics are opt-in and slow by design.

Requirements:

- Expose `refresh()` that updates a cached snapshot.
- Keep register reads out of the motion-critical path.
- Return `TransportError` on bus failures; do not silently retry forever.

This separation prevents timing regressions in motion control.

## 7) Safety and fault mapping

Map hardware events into library fault semantics:

- Emergency inputs -> `EmergencyStop`
- Hard limits / crash inputs -> `LimitSwitch` and `FaultCode::LimitExceeded`
- Driver-reported stalls -> `StopReason::StallDetected`, `FaultCode::Stall`
- Transport failures -> `ErrorCode::TransportError` / `FaultCode::TransportFault`

If the driver has a DIAG output, route it through `SensorRole` in axis config.

## 8) Axis/actuator integration

Only change actuator/facade code when strictly required.

- For STEP/DIR chips: usually no actuator change needed.
- For CAN servos: implement protocol calls so `armMotion/startMotion/stop` contracts are honored.
- Never silently downgrade stop semantics (`Decelerate` must not become `Immediate`).

### 8a) Provide a `<vendor>_kit` factory

Each driver SHOULD ship a kit alongside its other files
(`<vendor>_kit.{h,cpp}`). The kit bundles every helper a typical
host would otherwise construct by hand — UART/transport, configurator,
diagnostics (when relevant), the `Axis` itself, optional brake or
limit-switch controllers — into one factory.

Conventions:

- Factory returns `Result<std::unique_ptr<Kit>>`.
- Kit members are `std::unique_ptr` — no file-scope state, so N kits
  coexist in one project.
- For bus-shared transports (UART, CAN), provide a second factory
  `make<Vendor>KitOn<Bus>(bus&, slaveAddress, cfg)` that takes a
  pre-`begin()`-ed bus reference and does NOT own it. The owned-bus
  factory remains the simple-case entry point.
- `Kit::begin()` runs the lifecycle in order: bring up the bus (if
  owned), configure the chip, attach optional helpers, then call
  `axis->begin()`.
- The compose-by-hand path must keep working — kits sit alongside,
  they do not replace.

See `drivers/tmc2209/tmc2209_kit.{h,cpp}` and
`drivers/ypmc/ypmc_kit.{h,cpp}` for reference implementations.

## 9) Test requirements (host side)

Add tests under `tests/` for all new behavior.

Minimum test set:

1. Transport framing/CRC and error propagation.
2. Configurator write order and config validation.
3. W1C behavior where applicable.
4. Diagnostics snapshot refresh and partial-failure behavior.
5. Fault mapping to `FaultCode` / `StopReason`.
6. Example compile smoke test (Arduino sketch compiles).

Every new branch in driver logic needs at least one deterministic test.

## 10) Example sketch requirements

Add or update an example under `examples/` that shows:

- Minimal wiring.
- Driver bring-up.
- Axis creation and lifecycle (`begin`, `enable`, motion command, `service`).
- Expected failure handling path.

Examples must compile with `arduino-cli` using workspace libraries.

## 11) Documentation requirements

When adding a new driver module, update:

1. `lib_motor/README.md` (user-facing overview).
2. `lib_motor/API.md` (public API usage and contracts).
3. `drivers/<model>/README.md` (wiring, tuning, quirks).

If API and implementation diverge, fix docs in the same change.

## 12) Done checklist

You are done only when all are true:

- [ ] New driver compiles on target toolchain.
- [ ] Host tests pass for new modules.
- [ ] Motion path timing rules were not violated.
- [ ] Errors/faults map to existing `ErrorCode`/`FaultCode` semantics.
- [ ] Example compiles.
- [ ] README/API docs are updated.
