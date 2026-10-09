/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include <math.h>
#include <stdint.h>

class FtcRecoveryArbiter
{
public:
	enum Reason : uint32_t { DISABLED = 1, FLIGHT_GATE = 2, STALE = 4, INVALID = 8, MODE_CHANGED = 16 };
	struct Input {
		uint64_t now{0}, candidate_timestamp{0}, normal_timestamp{0};
		bool enabled{false}, flight_allowed{false}, candidate_valid{false}, mode_matches{false};
		bool hard_exit{false}, reentry{false};
		float requested_weight{1.f};
		float normal[4] {}, candidate[4] {};
	};
	void update(float dt, const Input &input)
	{
		reason = 0;
		if (!input.enabled) { reason |= DISABLED; }
		if (!input.flight_allowed) { reason |= FLIGHT_GATE; }
		if (!fresh(input.now, input.candidate_timestamp) || !fresh(input.now, input.normal_timestamp)) { reason |= STALE; }
		if (!input.candidate_valid) { reason |= INVALID; }
		if (!input.mode_matches) { reason |= MODE_CHANGED; }
		bool normal_valid = true;
		for (unsigned i = 0; i < 4; ++i) {
			normal_valid &= isfinite(input.normal[i]);
			if (!isfinite(input.candidate[i]) || !isfinite(input.normal[i])) { reason |= INVALID; }
			if (i < 3 && fabsf(input.candidate[i]) > 6.f) { reason |= INVALID; }
		}
		if (!isfinite(input.requested_weight) || input.requested_weight < 0.f || input.requested_weight > 1.f
		    || input.candidate[3] > 0.f || input.candidate[3] < -1.f) { reason |= INVALID; }
		const float target = reason == 0 ? clamp(input.requested_weight, 0.f, 1.f) : 0.f;
		dt = clamp(dt, 0.f, 0.02f);
		// No interpolation is defined against an invalid normal setpoint. Release FTC
		// ownership; handling the original invalid input remains with the PX4 controller.
		if (input.hard_exit || !normal_valid || !fresh(input.now, input.normal_timestamp)) { weight = 0.f; }
		else { weight += clamp(target - weight, -2.f * dt, dt); }
		for (unsigned i = 0; i < 4; ++i) {
			if (reason == 0) { _held[i] = input.candidate[i]; }
			output[i] = weight > 0.f ? input.normal[i] * (1.f - weight) + _held[i] * weight : input.normal[i];
		}
		active = weight > 0.f;
	}
	float output[4] {};
	float weight{0.f};
	bool active{false};
	uint32_t reason{DISABLED};
private:
	static bool fresh(uint64_t now, uint64_t timestamp) { return timestamp && now >= timestamp && now - timestamp < 200000; }
	static float clamp(float value, float lo, float hi) { return fminf(fmaxf(value, lo), hi); }
	float _held[4] {};
};
