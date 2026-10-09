/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include <math.h>
#include <stdint.h>

class FtcAllocationPolicy
{
public:
	static constexpr unsigned Motors = 12;
	enum State : uint8_t { NOMINAL, WAITING, BLENDING, ACTIVE, FALLBACK };
	enum Reason : uint32_t { DISABLED = 1, STALE_INPUT = 2, INVALID_MODEL = 4, UNCERTAINTY = 8,
		AUTHORITY = 16, UNSUPPORTED = 32, RESET = 64, NO_PERSISTENT_FAULT = 128, DISARMED = 256, LANDED = 512 };
	struct Input {
		uint64_t now{0}, model_timestamp{0}, authority_timestamp{0};
		uint32_t reset_count{0};
		unsigned count{0};
		bool enabled{false}, armed{false}, supported{false}, model_valid{false}, authority_valid{false};
		bool landed{true};
		float estimate_age{INFINITY}, attitude_authority{0.f}, thrust_authority{0.f}, yaw_authority{0.f};
		float lambda[Motors] {}, uncertainty[Motors] {};
	};
	FtcAllocationPolicy() { for (float &value : _lambda) { value = 1.f; } }
	void resetGeometry()
	{
		for (float &value : _lambda) { value = 1.f; }
		_fault_elapsed = 0.f; _engaged = _yaw_relaxed = active = false; yaw_weight = 1.f;
	}
	void update(float dt, const Input &input)
	{
		reason = 0;
		if (!input.enabled) { reason |= DISABLED; }
		if (!input.armed) { reason |= DISARMED; }
		if (input.landed) { reason |= LANDED; }
		if (!input.supported || input.count < 4 || input.count > Motors) { reason |= UNSUPPORTED; }
		if (!fresh(input.now, input.model_timestamp, 200000) || !fresh(input.now, input.authority_timestamp, 200000)
		    || !isfinite(input.estimate_age) || input.estimate_age < 0.f || input.estimate_age > 2.f) { reason |= STALE_INPUT; }
		if (!input.model_valid) { reason |= INVALID_MODEL; }
		if (!input.authority_valid || !isfinite(input.attitude_authority) || !isfinite(input.thrust_authority)
		    || !isfinite(input.yaw_authority) || input.attitude_authority < 0.35f || input.thrust_authority < 0.25f) { reason |= AUTHORITY; }
		if (_reset_count != input.reset_count) { reason |= RESET; _reset_count = input.reset_count; _fault_elapsed = 0.f; }
		bool fault = false;
		for (unsigned i = 0; i < input.count && i < Motors; ++i) {
			if (!isfinite(input.lambda[i]) || input.lambda[i] < 0.1f || input.lambda[i] > 1.f
		    || !isfinite(input.uncertainty[i]) || input.uncertainty[i] < 0.f || input.uncertainty[i] > 0.12f) { reason |= UNCERTAINTY; }
			fault |= input.lambda[i] < (_engaged ? 0.98f : 0.95f);
		}
		dt = fminf(fmaxf(dt, 0.f), 0.1f);
		if (reason & (DISARMED | LANDED | UNSUPPORTED)) { resetGeometry(); }
		_fault_elapsed = reason == 0 && fault ? _fault_elapsed + dt : 0.f;
		if (_fault_elapsed < 1.f) { reason |= NO_PERSISTENT_FAULT; }
		_engaged = reason == 0;
		bool changed = false, nominal = true;
		for (unsigned i = 0; i < Motors; ++i) {
			const float target = _engaged && i < input.count ? input.lambda[i] : 1.f;
			const float step = (_engaged ? 0.2f : 1.f) * dt;
			const float delta = fminf(fmaxf(target - _lambda[i], -step), step);
			_lambda[i] += delta;
			changed |= fabsf(target - _lambda[i]) > 1e-5f;
			nominal &= fabsf(_lambda[i] - 1.f) < 1e-5f;
		}
		if (!_engaged && nominal) { for (float &v : _lambda) { v = 1.f; } }
		_yaw_relaxed = _engaged && input.yaw_authority < (_yaw_relaxed ? 0.3f : 0.2f);
		yaw_weight += fminf(fmaxf((_yaw_relaxed ? 0.f : 1.f) - yaw_weight, -dt), dt);
		active = !nominal || yaw_weight < 0.99999f;
		state = _engaged ? (changed ? BLENDING : ACTIVE) : (active ? FALLBACK : (input.enabled ? WAITING : NOMINAL));
	}
	float lambda(unsigned index) const { return index < Motors ? _lambda[index] : 1.f; }
	static bool fresh(uint64_t now, uint64_t timestamp, uint64_t timeout)
	{ return timestamp && now >= timestamp && now - timestamp < timeout; }
	uint8_t state{NOMINAL};
	uint32_t reason{DISABLED};
	bool active{false};
	float yaw_weight{1.f};
private:
	float _lambda[Motors] {};
	float _fault_elapsed{0.f};
	uint32_t _reset_count{0};
	bool _engaged{false}, _yaw_relaxed{false};
};
