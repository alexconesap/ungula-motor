# UngulaDisplay Examples

Two example sketches that show how to use the display and UI library.

| Example | Display | Board | Description |
| --- | --- | --- | --- |
| `demo_800x480` | Waveshare 7" RGB (800x480) | ESP32-S3 | Full library usage with IO expander and touch widgets |
| `demo_320x240` | ILI9341 SPI TFT (320x240) | ESP32 | Standalone LGFX config, uses theme colors only |

## Prerequisites

- [arduino-cli](https://arduino.github.io/arduino-cli/) installed
- ESP32 board package (`esp32:esp32`) version 3.0.7 or later
- [LovyanGFX](https://github.com/lovyan03/LovyanGFX) library downloaded locally

## Compiling demo_800x480

This example uses the full library (display driver, widgets, IO expander) so it needs all of UngulaDisplay's dependencies.

Required libraries:

| Library | Path (relative to project root) | Purpose |
| --- | --- | --- |
| UngulaDisplay | `lib_display/` | Display driver, UI widgets |
| UngulaCore | `lib/` | Logger (transitive dependency) |
| LovyanGFX | your local copy | Graphics engine |
| ESP32_IO_Expander | your local copy | CH422G IO expander (backlight, reset) |

```shell
arduino-cli compile \
    --fqbn "esp32:esp32:esp32s3:CDCOnBoot=default,FlashSize=8M,PartitionScheme=default,PSRAM=opi" \
    --build-property "build.extra_flags=-DEMBEDDED_UI" \
    --library /path/to/lib_display \
    --library /path/to/lib \
    --library /path/to/LovyanGFX \
    --library /path/to/ESP32_IO_Expander \
    lib_display/examples/demo_800x480/demo_800x480.ino
```

If you are inside the necking-station project, the paths are already there:

```shell
arduino-cli compile \
    --fqbn "esp32:esp32:esp32s3:CDCOnBoot=default,FlashSize=8M,PartitionScheme=default,PSRAM=opi" \
    --build-property "build.extra_flags=-DEMBEDDED_UI" \
    --library lib_display \
    --library lib \
    --library ICB/libraries/LovyanGFX \
    --library ICB/libraries/ESP32_IO_Expander \
    lib_display/examples/demo_800x480/demo_800x480.ino
```

## Compiling demo_320x240

This example defines its own LGFX class and only uses `ui_theme.h` for color constants. It does not need UngulaCore or the IO expander library.

Required libraries:

| Library | Purpose |
| --- | --- |
| LovyanGFX | Graphics engine |

```shell
arduino-cli compile \
    --fqbn "esp32:esp32:esp32" \
    --build-property "build.extra_flags=-DEMBEDDED_UI" \
    --library /path/to/LovyanGFX \
    lib_display/examples/demo_320x240/demo_320x240.ino
```

## Notes

- The `EMBEDDED_UI` define is required for all display code to compile. Pass it via `--build-property "build.extra_flags=-DEMBEDDED_UI"`.
- The 320x240 example also defines `EMBEDDED_UI` directly in the source as a fallback, but passing it via build flags is the recommended approach.
- Pin assignments in the 320x240 example (GPIO 23, 18, 15, 2, 4, 32) are for a common ESP32 + ILI9341 wiring. Adjust them in the LGFX class constructor to match your board.
- arduino-cli discovers library sources through `#include` directives. The `--library` flag tells it where to find each library root. Without these flags, compilation will fail with missing header errors.
