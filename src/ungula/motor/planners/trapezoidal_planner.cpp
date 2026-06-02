// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/planners/trapezoidal_planner.h"

#include <cstdint>

namespace ungula::motor
{

namespace
{

        // Integer square root via Newton's method. Floating-point sqrt is
        // available everywhere we ship today, but the planner keeps the
        // segment values free of float to stay deterministic across
        // compiler versions and make hand-verification of expected
        // values practical.
        uint32_t isqrt64(uint64_t x)
        {
                if (x == 0) {
                        return 0;
                }
                uint64_t r = x;
                uint64_t next = (r + x / r) / 2u;
                while (next < r) {
                        r = next;
                        next = (r + x / r) / 2u;
                }
                return static_cast<uint32_t>(r);
        }

        // Convert a step rate to a timer half-period in ticks. One step =
        // two timer alarms (rising edge + falling edge).
        //
        // **Ceiling** division is critical. Actual output SPS is
        // `resolutionHz / (2 * hp)`. Floor-divide rounds hp down →
        // output rate lands HIGHER than requested by up to a full
        // rung. At 1 MHz, a request for 167 kSPS lands on hp=2 →
        // 250 kSPS (≈ 50 % over). Ceiling guarantees
        // `actual_sps ≤ requested_sps` — the planner never makes the
        // motor go faster than the host asked.
        uint32_t spsToHalfPeriodTicks(uint32_t sps, uint32_t resolutionHz)
        {
                if (sps == 0) {
                        return UINT32_MAX;
                }
                const uint64_t num = static_cast<uint64_t>(resolutionHz) + (2ull * sps - 1ull);
                const uint64_t hp = num / (2ull * sps);
                return (hp > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(hp);
        }

        // Combine the driver pulse-width floor and the HAL timer minTicks
        // floor. Returns whichever is larger so both constraints are
        // honoured.
        uint32_t computeMinHalfPeriodTicks(uint32_t minPulseUs, uint32_t resolutionHz,
                                           uint32_t timerMinTicks)
        {
                const uint64_t fromPulse =
                    (static_cast<uint64_t>(minPulseUs) * resolutionHz) / 1'000'000u;
                const uint32_t pulseTicks =
                    (fromPulse > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(fromPulse);
                return (pulseTicks > timerMinTicks) ? pulseTicks : timerMinTicks;
        }

        uint32_t clampHalfPeriod(uint32_t hp, uint32_t floorTicks)
        {
                return hp < floorTicks ? floorTicks : hp;
        }

        bool appendSegment(PlannedMove &move, uint32_t stepCount, uint32_t halfPeriodTicks)
        {
                if (stepCount == 0) {
                        return true;
                }
                if (move.segmentCount >= MAX_PLANNED_SEGMENTS) {
                        return false;
                }
                move.segments[move.segmentCount].stepCount = stepCount;
                move.segments[move.segmentCount].halfPeriodTicks = halfPeriodTicks;
                ++move.segmentCount;
                return true;
        }

        enum class RampDir : uint8_t { Accel, Decel };

        // Emit a tiered ramp into `move`. Returns the actual number of
        // steps emitted (which can differ from the analytical distance
        // by ±a few because each tier's step count is integer-rounded).
        uint32_t appendRamp(PlannedMove &move, uint32_t peakSps, uint32_t rateSpsPerSec,
                            uint8_t nSubSegments, RampDir dir, uint32_t minHalfPeriodTicks,
                            uint32_t resolutionHz)
        {
                if (peakSps == 0 || nSubSegments == 0 || rateSpsPerSec == 0) {
                        return 0;
                }

                uint32_t emitted = 0;
                for (uint8_t i = 1; i <= nSubSegments; ++i) {
                        const uint32_t i_lo = (dir == RampDir::Accel) ? (i - 1u) :
                                                                        (nSubSegments - i + 1u);
                        const uint32_t i_hi = (dir == RampDir::Accel) ? i : (nSubSegments - i + 0u);
                        const uint32_t v_lo = (peakSps * i_lo) / nSubSegments;
                        const uint32_t v_hi = (peakSps * i_hi) / nSubSegments;
                        const uint32_t v_avg = (v_lo + v_hi + 1u) / 2u;
                        if (v_avg == 0) {
                                continue;
                        }
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

        uint32_t capByHalfPeriodFloor(uint32_t requestedSps, uint32_t floorTicks,
                                      uint32_t resolutionHz)
        {
                if (floorTicks == 0) {
                        return requestedSps;
                }
                const uint64_t maxSps = static_cast<uint64_t>(resolutionHz) / (2ull * floorTicks);
                const uint32_t cap32 = (maxSps > UINT32_MAX) ? UINT32_MAX :
                                                               static_cast<uint32_t>(maxSps);
                return (requestedSps > cap32) ? cap32 : requestedSps;
        }

        // Make the move's emitted sum match the request exactly. See
        // header doc on `TrapezoidalPlanner` for the strategy.
        void rebalanceForExactStepCount(PlannedMove &move, uint32_t requestedSteps,
                                        int8_t cruiseIdx)
        {
                auto sumSegments = [&move]() -> uint32_t {
                        uint32_t s = 0;
                        for (uint8_t i = 0; i < move.segmentCount; ++i) {
                                s += move.segments[i].stepCount;
                        }
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
                        // Strip a stale-zero cruise placeholder if present.
                        if (cruiseIdx >= 0 && cruiseIdx < static_cast<int8_t>(move.segmentCount) &&
                            move.segments[cruiseIdx].stepCount == 0) {
                                for (uint8_t i = static_cast<uint8_t>(cruiseIdx);
                                     i + 1u < move.segmentCount; ++i) {
                                        move.segments[i] = move.segments[i + 1u];
                                }
                                --move.segmentCount;
                        }
                        move.totalSteps = requestedSteps;
                        return;
                }

                // Trapezoidal: absorb delta into the cruise segment.
                if (cruiseIdx >= 0 && cruiseIdx < static_cast<int8_t>(move.segmentCount)) {
                        auto &cruise = move.segments[cruiseIdx];
                        const int64_t newCruise = static_cast<int64_t>(cruise.stepCount) + delta;
                        if (newCruise >= 0) {
                                cruise.stepCount = static_cast<uint32_t>(newCruise);
                                if (cruise.stepCount == 0) {
                                        for (uint8_t i = static_cast<uint8_t>(cruiseIdx);
                                             i + 1u < move.segmentCount; ++i) {
                                                move.segments[i] = move.segments[i + 1u];
                                        }
                                        --move.segmentCount;
                                }
                                move.totalSteps = requestedSteps;
                                return;
                        }
                        // Cruise can't absorb full negative delta — zero it then
                        // walk the decel tail.
                        const int32_t consumed = static_cast<int32_t>(cruise.stepCount);
                        cruise.stepCount = 0;
                        for (uint8_t i = static_cast<uint8_t>(cruiseIdx);
                             i + 1u < move.segmentCount; ++i) {
                                move.segments[i] = move.segments[i + 1u];
                        }
                        --move.segmentCount;
                        int32_t residual = delta + consumed;
                        while (residual != 0 && move.segmentCount > 0) {
                                auto &tail = move.segments[move.segmentCount - 1u];
                                const int64_t newTail =
                                    static_cast<int64_t>(tail.stepCount) + residual;
                                if (newTail > 0) {
                                        tail.stepCount = static_cast<uint32_t>(newTail);
                                        residual = 0;
                                } else {
                                        residual += static_cast<int32_t>(tail.stepCount);
                                        --move.segmentCount;
                                }
                        }
                        move.totalSteps = sumSegments();
                        return;
                }

                // Triangular / stop ramp: adjust the tail.
                int32_t residual = delta;
                while (residual != 0 && move.segmentCount > 0) {
                        auto &tail = move.segments[move.segmentCount - 1u];
                        const int64_t newTail = static_cast<int64_t>(tail.stepCount) + residual;
                        if (newTail > 0) {
                                tail.stepCount = static_cast<uint32_t>(newTail);
                                residual = 0;
                        } else {
                                residual += static_cast<int32_t>(tail.stepCount);
                                --move.segmentCount;
                        }
                }
                move.totalSteps = sumSegments();
        }

        // The shared core for planMove/planJog. Distance is signed; sign
        // selects direction.
        PlannedMove planRelative(int32_t deltaSteps, const PlannerLimits &limits,
                                 uint32_t resolutionHz, uint32_t minTimerTicks)
        {
                PlannedMove move;
                if (deltaSteps == 0 || resolutionHz == 0) {
                        return move;
                }
                if (limits.maxVelocitySps == 0) {
                        return move;
                }

                move.direction = (deltaSteps > 0) ? Direction::Forward : Direction::Backward;
                const uint32_t requestedSteps =
                    static_cast<uint32_t>(deltaSteps > 0 ? deltaSteps : -deltaSteps);

                const uint32_t accel = limits.accelSpsPerSec;
                const uint32_t decel = limits.decelSpsPerSec;
                const uint32_t maxV = limits.maxVelocitySps;
                const uint32_t cap =
                    (limits.hardStepRateCeilingSps != 0u) ? limits.hardStepRateCeilingSps : maxV;
                const uint32_t vCapReq = (maxV < cap) ? maxV : cap;

                const uint32_t minPulseUs = (limits.minPulseHighUs > limits.minPulseLowUs) ?
                                                limits.minPulseHighUs :
                                                limits.minPulseLowUs;
                const uint32_t minHalfTicks =
                    computeMinHalfPeriodTicks(minPulseUs, resolutionHz, minTimerTicks);
                const uint32_t vCap = capByHalfPeriodFloor(vCapReq, minHalfTicks, resolutionHz);

                // Very-short-move fallback.
                if (requestedSteps < 4u) {
                        const uint32_t hp =
                            clampHalfPeriod(spsToHalfPeriodTicks(vCap, resolutionHz), minHalfTicks);
                        appendSegment(move, requestedSteps, hp);
                        move.totalSteps = requestedSteps;
                        move.cruiseSps = vCap;
                        return move;
                }

                // No-ramp path. Either accel or decel at zero means the
                // host explicitly does not want a ramp - emit one
                // cruise segment covering every requested step. The
                // motor (or downstream drive) must tolerate the instant
                // rate transition; that's safe at low SPS (a couple of
                // RPM) and what the host is asking for. At high SPS
                // expect lost steps. We use OR rather than AND so a
                // host that zeroes only one direction still gets the
                // no-ramp behaviour: ramping up but not down would
                // need a hybrid plan the planner doesn't model.
                if (accel == 0u || decel == 0u) {
                        const uint32_t hp =
                            clampHalfPeriod(spsToHalfPeriodTicks(vCap, resolutionHz), minHalfTicks);
                        appendSegment(move, requestedSteps, hp);
                        move.totalSteps = requestedSteps;
                        move.cruiseSps = vCap;
                        return move;
                }

                // Trapezoidal vs triangular?
                const uint64_t vCapSq = static_cast<uint64_t>(vCap) * vCap;
                const uint32_t accelStepsA = static_cast<uint32_t>(vCapSq / (2u * accel));
                const uint32_t decelStepsA = static_cast<uint32_t>(vCapSq / (2u * decel));

                uint32_t peakSps;
                uint32_t aStepsA;
                uint32_t dStepsA;
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
                        if (peakSps == 0) {
                                peakSps = 1;
                        }
                        peakSps = capByHalfPeriodFloor(peakSps, minHalfTicks, resolutionHz);
                        aStepsA = static_cast<uint32_t>((static_cast<uint64_t>(peakSps) * peakSps) /
                                                        (2u * accel));
                        if (aStepsA > requestedSteps) {
                                aStepsA = requestedSteps;
                        }
                        dStepsA = requestedSteps - aStepsA;
                }

                const uint8_t nAccel =
                    static_cast<uint8_t>((aStepsA < TrapezoidalPlanner::MAX_RAMP_SUB_SEGMENTS) ?
                                             aStepsA :
                                             TrapezoidalPlanner::MAX_RAMP_SUB_SEGMENTS);
                const uint8_t nDecel =
                    static_cast<uint8_t>((dStepsA < TrapezoidalPlanner::MAX_RAMP_SUB_SEGMENTS) ?
                                             dStepsA :
                                             TrapezoidalPlanner::MAX_RAMP_SUB_SEGMENTS);

                appendRamp(move, peakSps, accel, nAccel, RampDir::Accel, minHalfTicks,
                           resolutionHz);

                int8_t cruiseIdx = -1;
                if (trapezoidal) {
                        const uint32_t cruiseHp = clampHalfPeriod(
                            spsToHalfPeriodTicks(peakSps, resolutionHz), minHalfTicks);
                        if (appendSegment(move, 1u, cruiseHp)) {
                                cruiseIdx = static_cast<int8_t>(move.segmentCount - 1u);
                        }
                }

                appendRamp(move, peakSps, decel, nDecel, RampDir::Decel, minHalfTicks,
                           resolutionHz);

                if (cruiseIdx >= 0) {
                        move.segments[cruiseIdx].stepCount = 0;
                }
                rebalanceForExactStepCount(move, requestedSteps, cruiseIdx);
                move.cruiseSps = peakSps;
                return move;
        }

} // namespace

PlannedMove TrapezoidalPlanner::planMove(Position fromSteps, Position toSteps,
                                         const PlannerLimits &limits, uint32_t timerResolutionHz,
                                         uint32_t minTimerTicks)
{
        const int32_t delta = static_cast<int32_t>(toSteps - fromSteps);
        return planRelative(delta, limits, timerResolutionHz, minTimerTicks);
}

PlannedMove TrapezoidalPlanner::planJog(Direction dir, uint32_t safetyCapSteps,
                                        const PlannerLimits &limits, uint32_t timerResolutionHz,
                                        uint32_t minTimerTicks)
{
        const int32_t delta = (dir == Direction::Forward) ? static_cast<int32_t>(safetyCapSteps) :
                                                            -static_cast<int32_t>(safetyCapSteps);
        return planRelative(delta, limits, timerResolutionHz, minTimerTicks);
}

uint32_t TrapezoidalPlanner::actualSpsFor(uint32_t requestedSps, uint32_t resolutionHz)
{
        if (requestedSps == 0u || resolutionHz == 0u) {
                return 0u;
        }
        const uint64_t num = static_cast<uint64_t>(resolutionHz) + (2ull * requestedSps - 1ull);
        const uint64_t hp = num / (2ull * requestedSps);
        if (hp == 0u) {
                return resolutionHz / 2u; // 1-tick floor: maximum representable rate
        }
        return static_cast<uint32_t>(static_cast<uint64_t>(resolutionHz) / (2ull * hp));
}

PlannedMove TrapezoidalPlanner::planStop(Direction dir, uint32_t currentSps,
                                         const PlannerLimits &limits, uint32_t timerResolutionHz,
                                         uint32_t minTimerTicks) const
{
        PlannedMove move;
        if (currentSps == 0 || timerResolutionHz == 0 || limits.decelSpsPerSec == 0) {
                return move;
        }

        move.direction = dir;

        const uint32_t startV = currentSps;
        const uint32_t decel = limits.decelSpsPerSec;
        const uint64_t startSq = static_cast<uint64_t>(startV) * startV;
        const uint32_t stopSteps = static_cast<uint32_t>(startSq / (2u * decel));
        if (stopSteps == 0) {
                return move;
        }

        const uint32_t minPulseUs = (limits.minPulseHighUs > limits.minPulseLowUs) ?
                                        limits.minPulseHighUs :
                                        limits.minPulseLowUs;
        const uint32_t minHalfTicks =
            computeMinHalfPeriodTicks(minPulseUs, timerResolutionHz, minTimerTicks);

        const uint8_t n = static_cast<uint8_t>(
            (stopSteps < MAX_RAMP_SUB_SEGMENTS) ? stopSteps : MAX_RAMP_SUB_SEGMENTS);

        appendRamp(move, startV, decel, n, RampDir::Decel, minHalfTicks, timerResolutionHz);
        rebalanceForExactStepCount(move, stopSteps, /*cruiseIdx=*/-1);
        move.cruiseSps = startV;
        return move;
}

} // namespace ungula::motor
