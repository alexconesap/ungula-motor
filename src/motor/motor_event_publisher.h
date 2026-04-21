// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "i_motor_event_listener.h"

/// @brief Fixed-capacity event broadcaster for motor events.
///
/// No heap allocation. Capacity fixed at compile time via template parameter.

namespace motor {

    template <uint8_t MaxListeners>
    class MotorEventPublisher {
        public:
            /// @brief Register a listener. Returns false if full.
            bool subscribe(IMotorEventListener* listener) {
                if (listener == nullptr || count_ >= MaxListeners) {
                    return false;
                }
                for (uint8_t idx = 0; idx < count_; idx++) {
                    if (listeners_[idx] == listener) {
                        return true;  // already registered
                    }
                }
                listeners_[count_] = listener;
                count_++;
                return true;
            }

            /// @brief Remove a listener.
            bool unsubscribe(IMotorEventListener* listener) {
                for (uint8_t idx = 0; idx < count_; idx++) {
                    if (listeners_[idx] == listener) {
                        // Shift remaining listeners down
                        for (uint8_t jdx = idx; jdx < count_ - 1; jdx++) {
                            listeners_[jdx] = listeners_[jdx + 1];
                        }
                        count_--;
                        listeners_[count_] = nullptr;
                        return true;
                    }
                }
                return false;
            }

            /// @brief Publish an event to all registered listeners.
            void publish(const MotorEvent& event) {
                for (uint8_t idx = 0; idx < count_; idx++) {
                    listeners_[idx]->onMotorEvent(event);
                }
            }

            uint8_t listenerCount() const {
                return count_;
            }

        private:
            IMotorEventListener* listeners_[MaxListeners] = {};
            uint8_t count_ = 0;
    };

}  // namespace motor
