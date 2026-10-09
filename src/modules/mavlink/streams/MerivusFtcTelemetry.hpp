/****************************************************************************
 *
 *   Copyright (c) 2026 MERIVUS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name MERIVUS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 ****************************************************************************/

#ifndef MERIVUS_FTC_TELEMETRY_HPP
#define MERIVUS_FTC_TELEMETRY_HPP

#include <cmath>
#include <cstdint>

namespace merivus_ftc_telemetry
{
static constexpr uint8_t ProtocolVersion = 2;
static constexpr uint8_t PercentageUnavailable = UINT8_MAX;
static constexpr uint8_t MaxMotors = 12;

inline uint8_t encode_percentage(float value, bool valid = true)
{
	if (!valid || !std::isfinite(value)) {
		return PercentageUnavailable;
	}

	const float constrained = value < 0.f ? 0.f : (value > 1.f ? 1.f : value);
	return static_cast<uint8_t>(lroundf(constrained * 200.f));
}
}

#endif // MERIVUS_FTC_TELEMETRY_HPP
