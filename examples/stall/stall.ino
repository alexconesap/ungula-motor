// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

// Stall Motor Demo
//
// This example shows how to use the library's UI widgets on a smaller
// SPI-based display. Since gfx_core.h targets the 7" RGB display,
// this sketch defines its own LGFX class for the ILI9341 and provides
// the `gfx` global that the widget functions expect.
//
// Hardware: ESP32 DevKit + TMC2209 + NEMA 17 motor
// Board:    esp32:esp32:esp32

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("UngulaDisplay demo ready (320x240)");
}

void loop() {
    //
    delay(50);
}
