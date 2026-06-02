// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::motor::timing
{

/// Default minimum STEP-pulse HIGH width, in microseconds. Every
/// STEP/DIR drive the lib currently targets (TMC silicon, DM542 /
/// TB6600 generation, RATTMOTOR S2SVD15) accepts 1 us as its
/// datasheet minimum. The planner caps the maximum velocity at
/// `1 / (HIGH + LOW)`, so with HIGH = LOW = 1 us the ceiling lands
/// at 500 kSPS - enough for 3000 RPM at 10 000 PPR or 9000 RPM at
/// 3200 PPR. Hosts driving older optoisolated boards (1990s drives
/// at 2.5 us) bump these per-config via the driver-specific
/// `<Driver>Config::minPulseHighUs` field.
constexpr uint32_t kDefaultMinPulseHighUs = 1u;

/// Default minimum STEP-pulse LOW width, in microseconds. Same
/// constraints as `kDefaultMinPulseHighUs`; lib treats the larger of
/// HIGH and LOW as the effective floor for the velocity cap.
constexpr uint32_t kDefaultMinPulseLowUs = 1u;

/// Default DIR-to-STEP setup window, in microseconds. The drive sees
/// DIR change, then the next STEP rising edge `dirSetupUs` later.
/// Most drives spec 1-2 us; 5 us leaves cable + isolator delay
/// margin without measurably slowing motion.
constexpr uint32_t kDefaultDirSetupUs = 5u;

} // namespace ungula::motor::timing
