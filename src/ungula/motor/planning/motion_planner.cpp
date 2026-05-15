// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/planning/motion_planner.h"

#include <cstdint>

namespace ungula::motor
{

namespace
{

        /// Integer square root via Newton's method. ESP-IDF and STM32 both ship
        /// floating-point sqrt, but the planner deliberately keeps the segment
        /// values free of float to stay deterministic across compiler versions
        /// and to make hand-verification of test expectations practical.
        uint32_t isqrt64(uint64_t x)
        {
                if (x == 0)
                        return 0;
                uint64_t r = x;
                uint64_t next = (r + x / r) / 2u;
                while (next < r) {
                        r = next;
                        next = (r + x / r) / 2u;
                }
                return static_cast<uint32_t>(r);
        }

        /// Convert a step rate (SPS) to a timer half-period in ticks, using the
        /// timer's tick rate (`resolutionHz`). One step = two timer alarms (one
        /// rising edge, one falling). 64-bit math because `resolutionHz` could
        /// be 10+ MHz and the unsigned multiply by 2 must not overflow even at
        /// the high end.
        uint32_t spsToHalfPeriodTicks(uint32_t sps, uint32_t resolutionHz)
        {
                if (sps == 0)
                        return UINT32_MAX;
                const uint64_t hp = static_cast<uint64_t>(resolutionHz) / (2ull * sps);
                return (hp > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(hp);
        }

        /// Combine the driver-pulse-width floor and the HAL-timer minTicks
        /// floor. Returns the largest of (the user's `minPulseUs` converted to
        /// ticks at this resolution) and `timerMinTicks`. Both are converted in
        /// 64-bit to avoid overflow at 10+ MHz timer rates.
        uint32_t computeMinHalfPeriodTicks(uint32_t minPulseUs, uint32_t resolutionHz,
                                           uint32_t timerMinTicks)
        {
                const uint64_t fromPulse =
                    (static_cast<uint64_t>(minPulseUs) * resolutionHz) / 1'000'000u;
                const uint32_t pulseTicks =
                    (fromPulse > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(fromPulse);
                return (pulseTicks > timerMinTicks) ? pulseTicks : timerMinTicks;
        }

        /// Apply the half-period floor.
        uint32_t clampHalfPeriod(uint32_t hp, uint32_t floorTicks)
        {
                return hp < floorTicks ? floorTicks : hp;
        }

        bool appendSegment(PlannedMove &move, uint32_t stepCount, uint32_t halfPeriodTicks)
        {
                if (stepCount == 0)
                        return true;
                if (move.segmentCount >= MAX_PLANNED_SEGMENTS)
                        return false;
                move.segments[move.segmentCount].stepCount = stepCount;
                move.segments[move.segmentCount].halfPeriodTicks = halfPeriodTicks;
                move.segmentCount++;
                return true;
        }

        enum class RampDir : uint8_t { Accel, Decel };

        /// Generate a velocity-tiered ramp into `move`. Returns the actual
        /// number of steps emitted (which can differ from the analytical
        /// distance by ±a few because each tier's step count is rounded).
        uint32_t appendRamp(PlannedMove &move, uint32_t peakSps, uint32_t rateSpsPerSec,
                            uint8_t nSubSegments, RampDir dir, uint32_t minHalfPeriodTicks,
                            uint32_t resolutionHz)
        {
                if (peakSps == 0 || nSubSegments == 0 || rateSpsPerSec == 0)
                        return 0;

                uint32_t emitted = 0;
                for (uint8_t i = 1; i <= nSubSegments; ++i) {
                        const uint32_t i_lo = (dir == RampDir::Accel) ? (i - 1u) :
                                                                        (nSubSegments - i + 1u);
                        const uint32_t i_hi = (dir == RampDir::Accel) ? i : (nSubSegments - i + 0u);
                        const uint32_t v_lo = (peakSps * i_lo) / nSubSegments;
                        const uint32_t v_hi = (peakSps * i_hi) / nSubSegments;
                        const uint32_t v_avg = (v_lo + v_hi + 1u) / 2u;
                        if (v_avg == 0)
                                continue;

                        const uint64_t dvel = peakSps / nSubSegments;
                        const uint64_t stepsU =
                            (static_cast<uint64_t>(v_avg) * dvel) / rateSpsPerSec;
                        const uint32_t steps = (stepsU == 0) ? 1u : static_cast<uint32_t>(stepsU);

                        const uint32_t hp = clampHalfPeriod(
                            spsToHalfPeriodTicks(v_avg, resolutionHz), minHalfPeriodTicks);

                        if (!appendSegment(move, steps, hp)) {
                                return emitted; // segment array full
                        }
                        emitted += steps;
                }
                return emitted;
        }

        /// Cap peak velocity so its half-period respects the floor. With a
        /// half-period floor of `floorTicks` and timer resolution `R`, the max
        /// SPS is `R / (2 * floorTicks)`.
        uint32_t capByHalfPeriodFloor(uint32_t requestedSps, uint32_t floorTicks,
                                      uint32_t resolutionHz)
        {
                if (floorTicks == 0)
                        return requestedSps;
                const uint64_t maxSps = static_cast<uint64_t>(resolutionHz) / (2ull * floorTicks);
                const uint32_t cap32 = (maxSps > UINT32_MAX) ? UINT32_MAX :
                                                               static_cast<uint32_t>(maxSps);
                return (requestedSps > cap32) ? cap32 : requestedSps;
        }

        /// Re-balance the move so `sum(segments) == requestedSteps` exactly.
        ///
        /// Approach: the planner emitted accel + (optional cruise placeholder) +
        /// decel by appending in order. The sum drifts by ±a few from the
        /// request due to integer rounding in each ramp tier. To fix:
        ///
        ///   1. Compute `delta = requestedSteps - emittedSum`.
        ///   2. If `cruiseIdx >= 0` (trapezoidal), add `delta` to the cruise
        ///      segment. Cruise stepCount can absorb both positive and negative
        ///      delta as long as the result stays ≥ 0. If it would go negative,
        ///      zero it and roll the remainder forward into the decel ramp's
        ///      last segment.
        ///   3. If `cruiseIdx < 0` (triangular or stop), add `delta` to the
        ///      LAST segment. If that would go to ≤ 0, remove that segment and
        ///      add `delta` to the previous one. Repeat as needed.
        ///
        /// Post-condition: `sum(segments) == requestedSteps` exactly, OR the
        /// move has no segments at all (only possible if requestedSteps == 0).
        void rebalanceForExactStepCount(PlannedMove &move, uint32_t requestedSteps,
                                        int8_t cruiseIdx)
        {
                // Sum what we actually emitted.
                auto sumSegments = [&move]() -> uint32_t {
                        uint32_t s = 0;
                        for (uint8_t i = 0; i < move.segmentCount; ++i)
                                s += move.segments[i].stepCount;
                        return s;
                };

                if (move.segmentCount == 0) {
                        move.totalSteps = 0;
                        return;
                }

                const uint32_t emitted = sumSegments();
                const int32_t delta =
                    static_cast<int32_t>(requestedSteps) - static_cast<int32_t>(emitted);

                if (delta == 0) {
                        // Edge case: the planner inserted a cruise placeholder (set to
                        // stepCount=0 by the caller) and accel+decel happened to emit
                        // exactly the requested distance. The placeholder still has
                        // stepCount=0; the engine's `loadMove` validation rejects any
                        // segment with `stepCount == 0`, so we must remove the empty
                        // cruise before returning.
                        if (cruiseIdx >= 0 && cruiseIdx < static_cast<int8_t>(move.segmentCount) &&
                            move.segments[cruiseIdx].stepCount == 0) {
                                for (uint8_t i = static_cast<uint8_t>(cruiseIdx);
                                     i + 1u < move.segmentCount; ++i) {
                                        move.segments[i] = move.segments[i + 1u];
                                }
                                move.segmentCount--;
                        }
                        move.totalSteps = requestedSteps;
                        return;
                }

                // Strategy A: trapezoidal — try the cruise segment first.
                if (cruiseIdx >= 0 && cruiseIdx < static_cast<int8_t>(move.segmentCount)) {
                        auto &cruise = move.segments[cruiseIdx];
                        const int64_t newCruise = static_cast<int64_t>(cruise.stepCount) + delta;
                        if (newCruise >= 0) {
                                cruise.stepCount = static_cast<uint32_t>(newCruise);
                                if (cruise.stepCount == 0) {
                                        // Cruise reduced to nothing — remove the segment so the
                                        // ISR doesn't have to skip a no-op.
                                        for (uint8_t i = static_cast<uint8_t>(cruiseIdx);
                                             i + 1u < move.segmentCount; ++i) {
                                                move.segments[i] = move.segments[i + 1u];
                                        }
                                        move.segmentCount--;
                                }
                                move.totalSteps = requestedSteps;
                                return;
                        }
                        // Cruise can't absorb the full negative delta — zero it and
                        // pass the remainder to the tail strategy below.
                        const int32_t consumed = static_cast<int32_t>(cruise.stepCount);
                        cruise.stepCount = 0;
                        // Remove the zeroed cruise.
                        for (uint8_t i = static_cast<uint8_t>(cruiseIdx);
                             i + 1u < move.segmentCount; ++i) {
                                move.segments[i] = move.segments[i + 1u];
                        }
                        move.segmentCount--;
                        // Remaining delta still needs to be applied to the tail.
                        const int32_t remaining =
                            delta + consumed; // delta is negative; this is closer to 0
                        if (remaining == 0) {
                                move.totalSteps = requestedSteps;
                                return;
                        }
                        // Fall through to the tail-adjust path with `remaining` as the
                        // residual delta.
                        // We re-enter the tail logic by setting `delta` and falling out
                        // of the if. To avoid duplicating code we do an explicit loop
                        // below that handles both triangular and trapezoidal-overflow
                        // cases uniformly.
                        int32_t residual = remaining;
                        while (residual != 0 && move.segmentCount > 0) {
                                auto &tail = move.segments[move.segmentCount - 1u];
                                const int64_t newTail =
                                    static_cast<int64_t>(tail.stepCount) + residual;
                                if (newTail > 0) {
                                        tail.stepCount = static_cast<uint32_t>(newTail);
                                        residual = 0;
                                } else {
                                        // The tail can't absorb it; consume what it has and
                                        // remove the segment, then loop.
                                        residual += static_cast<int32_t>(tail.stepCount);
                                        move.segmentCount--;
                                }
                        }
                        move.totalSteps = sumSegments();
                        return;
                }

                // Strategy B: triangular / stop ramp — adjust the last segment.
                // Loop in case the adjustment would zero out the segment and we
                // need to keep walking backward.
                int32_t residual = delta;
                while (residual != 0 && move.segmentCount > 0) {
                        auto &tail = move.segments[move.segmentCount - 1u];
                        const int64_t newTail = static_cast<int64_t>(tail.stepCount) + residual;
                        if (newTail > 0) {
                                tail.stepCount = static_cast<uint32_t>(newTail);
                                residual = 0;
                        } else {
                                residual += static_cast<int32_t>(tail.stepCount);
                                move.segmentCount--;
                        }
                }
                move.totalSteps = sumSegments();
        }

} // namespace

PlannedMove MotionPlanner::planBy(Distance deltaSteps, const TrajectoryLimits &limits,
                                  uint32_t resolutionHz, uint32_t minHalfPeriodTicks) const
{
        PlannedMove move;
        if (deltaSteps == 0 || resolutionHz == 0)
                return move;
        if (limits.maxVelocitySps <= 0 || limits.accelSpsPerSec == 0 ||
            limits.decelSpsPerSec == 0) {
                return move;
        }

        move.direction = (deltaSteps > 0) ? Direction::Forward : Direction::Backward;
        const uint32_t requestedSteps =
            static_cast<uint32_t>(deltaSteps > 0 ? deltaSteps : -deltaSteps);

        const uint32_t accel = limits.accelSpsPerSec;
        const uint32_t decel = limits.decelSpsPerSec;
        const uint32_t maxV = static_cast<uint32_t>(limits.maxVelocitySps);
        const uint32_t cap = limits.maxStepRateSps ? limits.maxStepRateSps : maxV;
        const uint32_t vCapReq = (maxV < cap) ? maxV : cap;

        const uint32_t minPulseUs = (limits.minPulseHighUs > limits.minPulseLowUs) ?
                                        limits.minPulseHighUs :
                                        limits.minPulseLowUs;
        const uint32_t minHalfTicks =
            computeMinHalfPeriodTicks(minPulseUs, resolutionHz, minHalfPeriodTicks);

        // Cap peak velocity so it can't ask for a half-period below the
        // combined floor (timer minTicks ∪ minPulse).
        const uint32_t vCap = capByHalfPeriodFloor(vCapReq, minHalfTicks, resolutionHz);

        // ---- Very-short-move fallback (single constant-velocity segment) ----
        if (requestedSteps < 4u) {
                const uint32_t hp =
                    clampHalfPeriod(spsToHalfPeriodTicks(vCap, resolutionHz), minHalfTicks);
                appendSegment(move, requestedSteps, hp);
                move.totalSteps = requestedSteps;
                return move;
        }

        // ---- Choose trapezoidal vs triangular ----
        const uint64_t vCapSq = static_cast<uint64_t>(vCap) * vCap;
        const uint32_t accelStepsA = static_cast<uint32_t>(vCapSq / (2u * accel));
        const uint32_t decelStepsA = static_cast<uint32_t>(vCapSq / (2u * decel));

        uint32_t peakSps;
        uint32_t aStepsA; // analytical accel distance
        uint32_t dStepsA; // analytical decel distance
        bool trapezoidal = false;

        if (accelStepsA + decelStepsA <= requestedSteps) {
                trapezoidal = true;
                peakSps = vCap;
                aStepsA = accelStepsA;
                dStepsA = decelStepsA;
        } else {
                const uint64_t numerator = 2ull * accel * decel * requestedSteps;
                const uint64_t denom = static_cast<uint64_t>(accel) + decel;
                const uint64_t peakSq = numerator / denom;
                peakSps = isqrt64(peakSq);
                if (peakSps == 0)
                        peakSps = 1;
                peakSps = capByHalfPeriodFloor(peakSps, minHalfTicks, resolutionHz);
                aStepsA = static_cast<uint32_t>((static_cast<uint64_t>(peakSps) * peakSps) /
                                                (2u * accel));
                if (aStepsA > requestedSteps)
                        aStepsA = requestedSteps;
                dStepsA = requestedSteps - aStepsA;
        }

        const uint8_t nAccel =
            static_cast<uint8_t>((aStepsA < MAX_RAMP_SEGMENTS) ? aStepsA : MAX_RAMP_SEGMENTS);
        const uint8_t nDecel =
            static_cast<uint8_t>((dStepsA < MAX_RAMP_SEGMENTS) ? dStepsA : MAX_RAMP_SEGMENTS);

        // ---- Emit accel ramp ----
        appendRamp(move, peakSps, accel, nAccel, RampDir::Accel, minHalfTicks, resolutionHz);

        // ---- Reserve a cruise placeholder (trapezoidal only) ----
        // We insert a placeholder NOW so the segment order is
        // accel ... | cruise | ... decel, and we can patch the cruise's
        // stepCount after we know the emitted decel total.
        int8_t cruiseIdx = -1;
        if (trapezoidal) {
                const uint32_t cruiseHp =
                    clampHalfPeriod(spsToHalfPeriodTicks(peakSps, resolutionHz), minHalfTicks);
                // Use stepCount = 1 as a placeholder so appendSegment accepts
                // it; we'll overwrite the count in rebalanceForExactStepCount.
                if (appendSegment(move, 1u, cruiseHp)) {
                        cruiseIdx = static_cast<int8_t>(move.segmentCount - 1u);
                }
        }

        // ---- Emit decel ramp ----
        appendRamp(move, peakSps, decel, nDecel, RampDir::Decel, minHalfTicks, resolutionHz);

        // ---- Make totalSteps == requestedSteps EXACTLY ----
        // The cruise was created with stepCount=1; the placeholder will be
        // adjusted (or removed) by rebalanceForExactStepCount.
        if (cruiseIdx >= 0) {
                // Pre-subtract the placeholder "1" before rebalancing so the
                // delta math matches the true emitted sum.
                move.segments[cruiseIdx].stepCount = 0;
        }
        rebalanceForExactStepCount(move, requestedSteps, cruiseIdx);

        return move;
}

PlannedMove MotionPlanner::planTo(Position currentPos, Position targetPos,
                                  const TrajectoryLimits &limits, uint32_t resolutionHz,
                                  uint32_t minHalfPeriodTicks) const
{
        return planBy(static_cast<Distance>(targetPos - currentPos), limits, resolutionHz,
                      minHalfPeriodTicks);
}

PlannedMove MotionPlanner::planJog(Direction direction, uint32_t maxSteps,
                                   const TrajectoryLimits &limits, uint32_t resolutionHz,
                                   uint32_t minHalfPeriodTicks) const
{
        const Distance delta = (direction == Direction::Forward) ? static_cast<Distance>(maxSteps) :
                                                                   -static_cast<Distance>(maxSteps);
        return planBy(delta, limits, resolutionHz, minHalfPeriodTicks);
}

PlannedMove MotionPlanner::planStop(Direction direction, Velocity currentSps,
                                    const TrajectoryLimits &limits, uint32_t resolutionHz,
                                    uint32_t minHalfPeriodTicks) const
{
        PlannedMove move;
        if (currentSps <= 0 || resolutionHz == 0 || limits.decelSpsPerSec == 0) {
                return move;
        }

        move.direction = direction;

        const uint32_t startV = static_cast<uint32_t>(currentSps);
        const uint32_t decel = limits.decelSpsPerSec;

        const uint64_t startSq = static_cast<uint64_t>(startV) * startV;
        const uint32_t stopSteps = static_cast<uint32_t>(startSq / (2u * decel));
        if (stopSteps == 0)
                return move;

        const uint32_t minPulseUs = (limits.minPulseHighUs > limits.minPulseLowUs) ?
                                        limits.minPulseHighUs :
                                        limits.minPulseLowUs;
        const uint32_t minHalfTicks =
            computeMinHalfPeriodTicks(minPulseUs, resolutionHz, minHalfPeriodTicks);

        const uint8_t n =
            static_cast<uint8_t>((stopSteps < MAX_RAMP_SEGMENTS) ? stopSteps : MAX_RAMP_SEGMENTS);

        appendRamp(move, startV, decel, n, RampDir::Decel, minHalfTicks, resolutionHz);

        // Same exact-distance guarantee as planBy: the stop distance is
        // `stopSteps`, computed analytically. The ramp's emitted sum can
        // drift; rebalance into the last segment.
        rebalanceForExactStepCount(move, stopSteps, /*cruiseIdx=*/-1);

        return move;
}

} // namespace ungula::motor
