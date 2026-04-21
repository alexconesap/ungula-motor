// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

namespace ungula {
    namespace motor {

        /// Interface for motor driver hardware abstraction
        class IMotorDriver {
            public:
                virtual ~IMotorDriver() = default;
                virtual void begin() = 0;
                virtual void setDirection(bool clockwise) = 0;
                virtual void setEnable(bool enable) = 0;

                /// Adjust drive current based on current speed (steps/sec).
                /// Default no-op — override in drivers that support dynamic current control.
                virtual void setCurrentBySps(int sps) {
                    (void)sps;
                }
        };

    }  // namespace motor
}  // namespace ungula
