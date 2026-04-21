#pragma once

#define ARDUINO 100

#include <stdint.h>
#include <stdio.h>

#include <string>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define OUTPUT_OPEN_DRAIN 4

inline void pinMode(int pin, int mode) {
    (void)pin;
    (void)mode;
}
inline void digitalWrite(int pin, int value) {
    (void)pin;
    (void)value;
}
inline int digitalRead(int pin) {
    (void)pin;
    return 1;
}
inline void analogWrite(int pin, int value) {
    (void)pin;
    (void)value;
}
inline void delay(unsigned long ms) {
    (void)ms;
}
inline unsigned long millis() {
    return 0;
}
inline unsigned long micros() {
    return 0;
}

class HardwareSerial {
    public:
        void begin(unsigned long baud) {
            (void)baud;
        }
        void begin(unsigned long baud, uint32_t config, int rxPin, int txPin) {
            (void)baud;
            (void)config;
            (void)rxPin;
            (void)txPin;
        }
};

using String = std::string;
