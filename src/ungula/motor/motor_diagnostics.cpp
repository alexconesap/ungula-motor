// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/motor/motor_diagnostics.h"

#include <cstdio>

namespace ungula::motor
{

size_t toJson(const MotorDiagnostics &d, char *outBuf, size_t outLen)
{
        if (outBuf == nullptr || outLen == 0) {
                return 0;
        }

        // We compose into the caller's buffer with snprintf, then keep
        // an offset for chained writes. snprintf returns the number of
        // chars it WOULD have written (excluding NUL) — we use that to
        // detect truncation early.
        size_t offset = 0;
        auto append = [&](int written) -> bool {
                if (written < 0) {
                        return false;
                }
                const size_t w = static_cast<size_t>(written);
                if (w >= outLen - offset) {
                        // Would overflow; abandon.
                        return false;
                }
                offset += w;
                return true;
        };

        if (!append(snprintf(
                outBuf + offset, outLen - offset,
                "{\"axisId\":%u,\"state\":\"%s\",\"position\":%ld,"
                "\"currentSps\":%lu,\"targetSps\":%lu,\"stepsToTarget\":%ld,"
                "\"stopReason\":\"%s\",\"fault\":\"%s\","
                "\"totalStepsIssued\":%lu,\"homed\":%s,"
                "\"identity\":{\"vendor\":\"%s\",\"model\":\"%s\","
                "\"fw\":\"0x%02x.%u\"}",
                static_cast<unsigned>(d.axisId.value), motorStateToString(d.state),
                static_cast<long>(d.commandedPosition), static_cast<unsigned long>(d.currentSps),
                static_cast<unsigned long>(d.targetSps), static_cast<long>(d.stepsToTarget),
                stopReasonToString(d.lastStopReason), faultToString(d.lastFault),
                static_cast<unsigned long>(d.totalStepsIssued), d.homed ? "true" : "false",
                d.identity.vendor ? d.identity.vendor : "Unknown",
                d.identity.model ? d.identity.model : "Unknown",
                static_cast<unsigned>(d.identity.firmwareMajor),
                static_cast<unsigned>(d.identity.firmwareMinor)))) {
                outBuf[0] = '\0';
                return 0;
        }

        if (d.stall_valid) {
                if (!append(snprintf(outBuf + offset, outLen - offset,
                                     ",\"stall\":{\"sensitivity_pct\":%u,"
                                     "\"reading_pct\":%u,\"hits\":%lu}",
                                     static_cast<unsigned>(d.stallSensitivityPct),
                                     static_cast<unsigned>(d.stallReadingPct),
                                     static_cast<unsigned long>(d.stallHitsSinceClear)))) {
                        outBuf[0] = '\0';
                        return 0;
                }
        }

        if (d.adaptive_current_valid) {
                if (!append(snprintf(outBuf + offset, outLen - offset,
                                     ",\"adaptiveCurrent\":{\"active_ma\":%u}",
                                     static_cast<unsigned>(d.adaptiveCurrentMa)))) {
                        outBuf[0] = '\0';
                        return 0;
                }
        }

        if (d.driverRawDiagnostics != nullptr) {
                if (!append(snprintf(outBuf + offset, outLen - offset, ",\"driverRaw\":\"%s\"",
                                     d.driverRawDiagnostics))) {
                        outBuf[0] = '\0';
                        return 0;
                }
        }

        if (!append(snprintf(outBuf + offset, outLen - offset, "}"))) {
                outBuf[0] = '\0';
                return 0;
        }

        return offset;
}

} // namespace ungula::motor
