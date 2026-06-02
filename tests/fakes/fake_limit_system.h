// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/motor/i_limit_system.h"

namespace ungula::motor::tests
{

/// Host-side fake limit system that lets tests drive activation states
/// directly via setters. No ISR plumbing, no GPIO — purely a deterministic
/// container of limit states the test can manipulate.
class FakeLimitSystem final : public ILimitSystem {
    public:
        Status begin(const LimitWiring * /*wirings*/, uint8_t /*count*/,
                     IStepSignalGenerator * /*engineForIsr*/) override
        {
                return Status::Ok();
        }
        void end() override {}
        void service(int64_t /*nowMs*/) override {}

        bool isActive(LimitKind kind) const override
        {
                switch (kind) {
                case LimitKind::TravelLimit:    return travelFwd || travelBwd;
                case LimitKind::EmergencyLimit: return emergencyActive;
                case LimitKind::HomeSensor:     return homeActive;
                case LimitKind::StallSensor:    return stallActive;
                }
                return false;
        }
        bool isActive(LimitKind kind, Direction dir) const override
        {
                if (kind == LimitKind::TravelLimit) {
                        return (dir == Direction::Forward) ? travelFwd : travelBwd;
                }
                return isActive(kind);
        }
        bool consumeEmergencyActivation() override
        {
                const bool was = emergencyActive;
                emergencyActive = false;
                return was;
        }
        bool consumeStallActivation() override
        {
                const bool was = stallActive;
                stallActive = false;
                return was;
        }
        void notifyMotionStart(int64_t nowMs) override
        {
                motionStartedAtMs = nowMs;
                motionEnded = false;
        }
        void notifyMotionEnd() override
        {
                motionEnded = true;
        }
        uint32_t totalStallHits() const override { return stallHitsTotal; }
        void     resetStallHitsTotal() override { stallHitsTotal = 0; }

        // ---- Test knobs ------------------------------------------------
        bool travelFwd = false;
        bool travelBwd = false;
        bool emergencyActive = false;
        bool stallActive = false;
        bool homeActive = false;
        uint32_t stallHitsTotal = 0;
        int64_t  motionStartedAtMs = 0;
        bool     motionEnded = false;
};

} // namespace ungula::motor::tests
