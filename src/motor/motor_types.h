// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

/// @brief Enums, constants, and lightweight structs for the motor subsystem.
///
namespace motor {

    /// @brief Motor rotation direction.
    enum class Direction : uint8_t {
        FORWARD = 0,  /// Away from home position (DIR pin HIGH).
        BACKWARD = 1  /// Toward home position (DIR pin LOW).
    };

    /// @brief Motion profile selector — each carries independent speed and ramp.
    enum class MotionProfile : uint8_t { JOG = 0, HOMING = 1, CYCLE = 2 };

    /// @brief Homing sub-FSM phases.
    enum class HomingPhase : uint8_t {
        MOVE_TO_HOME = 0,
        BACKOFF = 1,
        FINAL_APPROACH = 2,
        COMPLETE = 3
    };

    /// @brief Distance/position unit for moveTo/moveBy.
    enum class DistanceUnit : uint8_t { STEPS = 0, MM = 1, CM = 2, DEGREES = 3 };

    /// @brief Speed unit for profile configuration.
    enum class SpeedUnit : uint8_t {
        STEPS_PER_SEC = 0,
        MM_PER_SEC = 1,
        CM_PER_SEC = 2,
        DEGREES_PER_SEC = 3
    };

    /// @brief Speed value with associated unit.
    struct SpeedValue {
            SpeedUnit unit = SpeedUnit::STEPS_PER_SEC;
            float value = 0.0F;
    };

    /// @brief Speed + acceleration/deceleration ramps for one motion profile.
    struct ProfileConfig {
            int32_t speedSps = 0;      /// Target speed in steps per second.
            uint32_t accelTimeMs = 0;  /// Ramp duration (ms) from 0 to speedSps. 0 = instant.
            uint32_t decelTimeMs = 0;  /// Ramp duration (ms) from speedSps to 0. 0 = instant stop.
    };

    /// @brief Linear current-vs-speed curve for driver run current.
    ///
    /// Used by LocalMotor when the feature is enabled — on every commanded
    /// speed change the motor maps target SPS to run current via this curve
    /// and calls driver.setRunCurrent(mA). Below minSps → minMa. Above
    /// maxSps → maxMa. Between → linear interpolation.
    ///
    /// The curve is direction-agnostic — put the larger current on whichever
    /// end fits the mechanical need:
    ///   - High torque at low speed (vertical axis holding weight):
    ///     minSps=200, maxSps=3000, minMa=1300, maxMa=900.
    ///   - High torque at high speed (legacy RBB1 behaviour):
    ///     minSps=300, maxSps=2000, minMa=900, maxMa=1300.
    struct CurrentCurve {
            int32_t minSps = 0;
            int32_t maxSps = 0;
            uint16_t minMa = 0;
            uint16_t maxMa = 0;
    };

    /// @brief Map a commanded speed (SPS) to a run current (mA) using a
    /// linear current-vs-speed curve. Direction-agnostic — |sps| is used so
    /// callers don't have to care about signed speeds.
    ///
    /// Rules:
    ///   - Degenerate curve (minSps >= maxSps): returns minMa.
    ///   - sps at or below minSps: returns minMa.
    ///   - sps at or above maxSps: returns maxMa.
    ///   - Otherwise: linear interpolation between the two endpoints.
    inline uint16_t currentMaForSps(const CurrentCurve& curve, int32_t speedSps) {
        const int32_t sps = (speedSps < 0) ? -speedSps : speedSps;
        if (curve.minSps >= curve.maxSps) {
            return curve.minMa;
        }
        if (sps <= curve.minSps) {
            return curve.minMa;
        }
        if (sps >= curve.maxSps) {
            return curve.maxMa;
        }
        const int32_t span = curve.maxSps - curve.minSps;
        const int32_t offset = sps - curve.minSps;
        const int32_t delta = static_cast<int32_t>(curve.maxMa) - static_cast<int32_t>(curve.minMa);
        const int32_t ma = static_cast<int32_t>(curve.minMa) + ((delta * offset) / span);
        return static_cast<uint16_t>(ma);
    }

    /// @brief Conversion constants.
    constexpr float CM_TO_MM = 10.0F;

    // ---- GPIO sentinel ----
    constexpr uint8_t GPIO_NONE = 0xFF;  /// No pin assigned.

    // ---- Limit switch constants ----
    namespace limit {
        constexpr uint32_t DEBOUNCE_MS = 20;
        constexpr int32_t MAX_PER_DIRECTION = 2;  /// Max switches per direction (home + crash).
    }  // namespace limit

    // ---- Stall detection defaults ----
    namespace stall {
        constexpr int32_t DEFAULT_DIAG_SCORE_LIMIT = 8;  /// DIAG score level that confirms a stall.
        constexpr int32_t DEFAULT_SG_SCORE_LIMIT =
                10;  /// SG_RESULT score level that confirms a stall.
        constexpr float DEFAULT_STALL_FRACTION =
                0.65F;  /// SG drop to 65% of baseline = stall reading.
        constexpr float DEFAULT_SG_PER_SPS =
                0.17F;  /// SG_RESULT per step-per-second (motor dependent).
        constexpr uint16_t DEFAULT_SG_BASELINE_CAP =
                300;  /// Max baseline — SG_RESULT saturates at high speeds.
        constexpr int32_t LOW_SPEED_SPS = 1200;  /// Below this speed, DIAG is suppressed.
    }  // namespace stall

    /// @brief Which stall detection path triggered.
    enum class StallCause : uint8_t {
        None,      /// No stall active — snapshot taken during normal operation.
        Diag,      /// DIAG pin path crossed its score limit first.
        Register,  /// SG_RESULT register path crossed its score limit first.
        Both,      /// Both paths were above their limits at snapshot time.
    };

    /// @brief Read-only snapshot of stall state at the moment of the last check.
    /// Capture this immediately after isStalling() returns true — the next
    /// service tick may update the scores.
    struct StallTelemetry {
            StallCause cause     = StallCause::None;
            int32_t    diagScore = 0;   /// DIAG score at snapshot time.
            int32_t    diagLimit = 0;   /// DIAG score limit (trigger threshold).
            int32_t    sgScore   = 0;   /// SG_RESULT score at snapshot time.
            int32_t    sgLimit   = 0;   /// SG_RESULT score limit.
            uint16_t   sgResult  = 0;   /// Last raw SG_RESULT read from chip.
            uint16_t   sgThresh  = 0;   /// Speed-based SG threshold at trigger.
            uint16_t   sgBase    = 0;   /// Expected SG baseline at current speed.
    };

    // ---- Step generator constants ----
    namespace step {
        constexpr uint32_t TIMER_FREQ_HZ = 1'000'000;   /// 1 MHz timer for step pulses.
        constexpr uint32_t MIN_ALARM_TICKS = 2;         /// Fastest allowed timer period.
        constexpr uint32_t IDLE_ALARM_TICKS = 500'000;  /// ~1 SPS — effectively parked.
        constexpr uint32_t RAMP_SERVICE_US = 10'000;    /// Ramp esp_timer period in microseconds.
        constexpr uint32_t RAMP_SERVICE_MS = 10;        /// Ramp update delta in milliseconds.
        constexpr float TOGGLES_PER_STEP = 2.0F;        /// Two pin toggles per full step.
        constexpr float MIN_RUNNING_SPS = 1.0F;         /// Below this, motor considered stopped.
    }  // namespace step

    // ---- Internal service timer ----
    namespace svc {
        constexpr uint32_t MOTOR_SERVICE_US = 10'000;  /// LocalMotor safety check interval (10 ms).
        constexpr uint32_t SG_POLL_INTERVAL_MS =
                50;  /// SG_RESULT UART poll throttle (avoids blocking esp_timer).
    }  // namespace svc

    // ---- Homing constants ----
    namespace homing {
        constexpr int32_t BACKOFF_STEPS = 200;       /// Steps to back away from limit.
        constexpr int32_t APPROACH_SPEED_SPS = 500;  /// Slow final approach speed.
    }  // namespace homing

    /// Number of motion profiles (JOG, HOMING, CYCLE).
    constexpr int32_t PROFILE_COUNT = 3;

}  // namespace motor
