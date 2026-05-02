// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "motor_types.h"

/// @brief Dual-path stall detector for TMC2209 with speed-based threshold.
///
/// Two independent detection channels run in parallel:
///
/// **DIAG pin (fast path)** — hardware threshold via SGTHRS register.
/// Works well at normal speeds. Suppressed at low speeds by LocalMotor
/// because back-EMF is too weak and DIAG produces false positives.
///
/// **SG_RESULT register (speed-scaled path)** — reads the raw 10-bit
/// StallGuard value via UART every service tick (~10 ms).
///
/// ## How SG_RESULT scales with speed
///
/// SG_RESULT is proportional to motor speed — at higher speeds the back-EMF
/// is stronger and SG_RESULT is higher. The host project measures this
/// relationship for their motor and provides a slope factor (sgPerSps):
///
///   baseline = sgPerSps × currentSpeed
///   threshold = baseline × stallFraction
///
/// Example for the TENSES motor (measured):
///   200 sps → SG ~38  → sgPerSps ≈ 0.17
///   600 sps → SG ~110 → sgPerSps ≈ 0.18
///
/// With sgPerSps=0.17 and stallFraction=0.65:
///   At 600 sps: threshold = 0.17 × 600 × 0.65 = 66
///   At 200 sps: threshold = 0.17 × 200 × 0.65 = 22
///
/// ## Score counter (leaky bucket)
///
/// The score counter filters the oscillation inherent to StealthChop mode:
///   - Each reading BELOW threshold: score += 2
///   - Each reading ABOVE threshold: score -= 1 (minimum 0)
///   - When score >= scoreLimit: stall confirmed
///
/// During a real stall, the majority of readings drop below threshold,
/// so the score accumulates despite occasional above-threshold readings.
///
/// ## Configuration
///
/// The host project configures four values:
///
///   `setSgPerSps(0.17)` — SG_RESULT per step-per-second. Measured once
///   for the specific motor at low-to-medium speeds with no load. This is
///   a hardware constant for the motor/driver/voltage combination.
///   The linear model works well at low-to-medium speeds but overestimates
///   the baseline at high speeds where SG_RESULT saturates.
///
///   `setSgBaselineCap(300)` — maximum allowed baseline value. SG_RESULT
///   does not grow linearly at high speeds — it flattens out. Without
///   a cap, the linear model produces thresholds above the actual SG value,
///   causing false stalls. Set to ~80% of observed SG at max operating speed.
///
///   `setStallFraction(0.65)` — how far SG_RESULT must drop (relative to
///   the speed-based baseline) to be counted as a stall reading. Lower
///   values = less sensitive. Higher values = more sensitive.
///
///   `setScoreLimit(10)` — how many net stall-readings before the motor
///   is declared stalled. Higher = slower but more noise-resistant.
///
/// ## Why not auto-calibrate at runtime?
///
/// If the motor starts already blocked (against a hard stop), the first
/// readings are stall values. Auto-calibration would learn those as the
/// baseline and never trigger. The speed-based approach works even when
/// the motor is stalled from the very first step, because the expected
/// baseline comes from the known motor characteristic, not from runtime
/// samples.
///
namespace motor {

    /// @brief Sentinel value indicating SG_RESULT is not available.
    constexpr uint16_t SG_RESULT_NOT_SUPPORTED = 0xFFFF;

    /// @brief Dual-path stall detector: DIAG pin + speed-scaled SG_RESULT.
    class StallDetector {
        public:
            // ---- DIAG pin path ----

            /// @brief Set the DIAG score limit that confirms a stall (default: 8).
            void setDiagScoreLimit(int32_t limit) {
                diagScoreLimit_ = limit;
            }

            // ---- SG_RESULT register path ----

            /// @brief Set the SG_RESULT-per-SPS slope for speed-based threshold.
            ///
            /// Measured once for the specific motor: run at known speed with no load,
            /// read SG_RESULT, divide by speed. Example: at 600 sps SG=110 → 0.18.
            ///
            /// @param slope SG_RESULT per step-per-second. Default: 0.17.
            void setSgPerSps(float slope) {
                sgPerSps_ = slope;
            }

            /// @brief Set the maximum baseline value (SG_RESULT cap).
            /// SG_RESULT saturates at high speeds — the linear model overestimates.
            /// Set to ~80% of observed SG at the highest operating speed.
            void setSgBaselineCap(uint16_t cap) {
                sgBaselineCap_ = cap;
            }

            /// @brief Set how far SG must drop from the speed-based baseline to
            /// count as a stall reading.
            ///
            /// The stall threshold is: sgPerSps × speed × stallFraction.
            /// Readings at or below that threshold fill the score counter.
            ///
            /// Example: sgPerSps=0.17, speed=600, fraction=0.65 → threshold=66.
            /// Normal SG at 600 sps is ~110, so readings must drop to 66 or lower.
            ///
            /// @param fraction 0.0–1.0. Lower = less sensitive. Default: 0.65.
            void setStallFraction(float fraction) {
                stallFraction_ = fraction;
            }

            /// @brief Set how many net stall-readings confirm a stall (default: 10).
            ///
            /// Each SG reading below threshold adds 2 to the score. Each reading
            /// above threshold subtracts 1. When score reaches this limit, stall
            /// is confirmed.
            ///
            /// At 10 ms polling with ~60% of readings below threshold during a
            /// real stall, a limit of 10 triggers in ~200 ms.
            void setScoreLimit(int32_t limit) {
                sgScoreLimit_ = limit;
            }

            // ---- Speed update (call on motion start and speed changes) ----

            /// @brief Recalculate the SG threshold for the current motor speed.
            /// Call this when starting motion or changing speed.
            /// @param speedSps Current target speed in steps per second.
            void configureForSpeed(int32_t speedSps) {
                currentSpeedSps_ = speedSps;
                uint16_t linearBaseline =
                        static_cast<uint16_t>(sgPerSps_ * static_cast<float>(speedSps));
                uint16_t baseline =
                        (linearBaseline < sgBaselineCap_) ? linearBaseline : sgBaselineCap_;
                sgThreshold_ = static_cast<uint16_t>(static_cast<float>(baseline) * stallFraction_);
            }

            // ---- Polling (call from service timer) ----

            /// @brief Feed the current DIAG pin level. Call every service tick.
            void pollPin(bool pinIsHigh) {
                if (pinIsHigh) {
                    diagScore_ = diagScore_ + kDiagIncrement;
                } else {
                    diagScore_ = diagScore_ - kDiagDecrement;
                    if (diagScore_ < 0) {
                        diagScore_ = 0;
                    }
                }
            }

            /// @brief Feed SG_RESULT value read via UART. Call every service tick.
            /// Compares against the speed-based threshold computed by configureForSpeed().
            void pollRegister(uint16_t sgResult) {
                if (sgResult == SG_RESULT_NOT_SUPPORTED) {
                    return;
                }
                if (sgResult <= sgThreshold_) {
                    sgScore_ = sgScore_ + kSgIncrement;
                } else {
                    sgScore_ = sgScore_ - kSgDecrement;
                    if (sgScore_ < 0) {
                        sgScore_ = 0;
                    }
                }
            }

            // ---- State queries ----

            /// @brief Check if either detection path has triggered.
            bool isStalling() const {
                bool diagStall = (diagScore_ >= diagScoreLimit_);
                bool registerStall = (sgScore_ >= sgScoreLimit_);
                return diagStall || registerStall;
            }

            /// @brief Check if the DIAG path specifically triggered.
            bool isDiagStalling() const {
                return diagScore_ >= diagScoreLimit_;
            }

            /// @brief Check if the register path specifically triggered.
            bool isRegisterStalling() const {
                return sgScore_ >= sgScoreLimit_;
            }

            /// @brief Reset both score counters. Call when starting new motion.
            void clear() {
                diagScore_ = 0;
                sgScore_ = 0;
            }

            // ---- Diagnostics ----

            /// @brief Current DIAG score (how close to triggering).
            int32_t diagScoreNow() const {
                return diagScore_;
            }

            /// @brief Current DIAG score limit.
            int32_t diagScoreLimitNow() const {
                return diagScoreLimit_;
            }

            /// @brief Current SG_RESULT score (how close to triggering).
            int32_t sgScoreNow() const {
                return sgScore_;
            }

            /// @brief Current SG_RESULT score limit.
            int32_t sgScoreLimitNow() const {
                return sgScoreLimit_;
            }

            /// @brief Active SG threshold (from speed-based calculation).
            uint16_t sgThresholdNow() const {
                return sgThreshold_;
            }

            /// @brief Expected SG baseline at current speed (capped).
            uint16_t sgBaselineNow() const {
                uint16_t linear =
                        static_cast<uint16_t>(sgPerSps_ * static_cast<float>(currentSpeedSps_));
                return (linear < sgBaselineCap_) ? linear : sgBaselineCap_;
            }

        private:
            // DIAG pin path (fast, normal speeds only)
            static constexpr int32_t kDiagIncrement = 2;
            static constexpr int32_t kDiagDecrement = 1;
            volatile int32_t diagScore_ = 0;
            int32_t diagScoreLimit_ = stall::DEFAULT_DIAG_SCORE_LIMIT;

            // SG_RESULT register path (speed-scaled, all speeds)
            static constexpr int32_t kSgIncrement = 2;
            static constexpr int32_t kSgDecrement = 1;
            int32_t sgScore_ = 0;
            int32_t sgScoreLimit_ = stall::DEFAULT_SG_SCORE_LIMIT;
            float stallFraction_ = stall::DEFAULT_STALL_FRACTION;
            float sgPerSps_ = stall::DEFAULT_SG_PER_SPS;
            uint16_t sgBaselineCap_ = stall::DEFAULT_SG_BASELINE_CAP;

            // Speed-based threshold (recomputed on each configureForSpeed call)
            uint16_t sgThreshold_ = 0;
            int32_t currentSpeedSps_ = 0;
    };

}  // namespace motor
