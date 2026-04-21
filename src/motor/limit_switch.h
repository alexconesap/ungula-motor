// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <hal/gpio/gpio_access.h>
#include <cstdint>
#include "motor_types.h"

/// @brief Debounced limit switch with configurable polarity.
///
/// Default (NC, normally-closed): pin reads LOW when closed (not triggered),
/// HIGH when open (triggered by mechanical contact).
/// Inverted (NO, normally-open): pin reads HIGH when closed, LOW when triggered.
/// 20 ms software debounce prevents false triggers from contact bounce.

namespace motor {

    /// @brief Single debounced limit switch.
    class LimitSwitch {
        public:
            /// @brief Assign GPIO pin and configure as input. Call before first update().
            /// @param pin GPIO number.
            /// @param invertPolarity true for NO (normally-open) wiring. Default: false (NC).
            void configure(uint8_t pin, bool invertPolarity = false) {
                pin_ = pin;
                inverted_ = invertPolarity;
                ungula::gpio::configInput(pin_);
            }

            /// @brief Update debounce filter. Call every service tick (~10 ms).
            /// @param nowMs Current time in milliseconds.
            void update(uint32_t nowMs) {
                if (pin_ == GPIO_NONE) {
                    return;
                }

                bool rawState = ungula::gpio::isHigh(pin_);
                if (rawState != lastRaw_) {
                    lastRaw_ = rawState;
                    lastChangeMs_ = nowMs;
                } else if (stable_ != rawState && (nowMs - lastChangeMs_) >= limit::DEBOUNCE_MS) {
                    stable_ = rawState;
                }
            }

            /// @brief Check if switch is triggered (debounced, polarity-aware).
            bool isTriggered() const {
                return (pin_ != GPIO_NONE) && (stable_ != inverted_);
            }

            /// @brief Check if this switch has a pin assigned.
            bool isConfigured() const {
                return pin_ != GPIO_NONE;
            }

            /// @brief Get assigned GPIO number.
            uint8_t pin() const {
                return pin_;
            }

        private:
            uint8_t pin_ = GPIO_NONE;
            bool inverted_ = false;
            bool stable_ = false;
            bool lastRaw_ = false;
            uint32_t lastChangeMs_ = 0;
    };

}  // namespace motor
