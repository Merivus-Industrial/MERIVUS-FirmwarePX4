/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include <math.h>
#include <stdint.h>

class FtcRecoveryController
{
public:
	enum State : uint8_t { DISABLED, MONITORING, DISTURBANCE_DETECTED, RATE_DAMPING,
		THRUST_VECTOR_RECOVERY, ATTITUDE_RECOVERY, ALTITUDE_STABILIZATION, CONTROL_REENTRY,
		EMERGENCY_LAND, ABORTED, FAILED, VERTICAL_SPEED_RECOVERY };
	enum FallbackReason : uint32_t { VERTICAL_STATE_RESET = 1u << 4 };
	static bool entryAltitudeAllowed(uint8_t previous_state, bool position_fresh, bool dist_bottom_valid,
					 float dist_bottom, bool z_valid, float z, float minimum_altitude)
	{
		const bool entry = previous_state == MONITORING || previous_state == DISABLED
			|| previous_state == DISTURBANCE_DETECTED;
		const float altitude = dist_bottom_valid ? dist_bottom : (z_valid ? -z : 0.f);
		const bool altitude_valid = (dist_bottom_valid || z_valid) && isfinite(altitude);
		return !entry || (position_fresh && altitude_valid && isfinite(minimum_altitude)
				 && altitude >= minimum_altitude);
	}
	static bool requiresImmediateExit(uint32_t fallback_reason)
	{
		return (fallback_reason & VERTICAL_STATE_RESET) != 0;
	}
	struct Input {
		uint64_t now{0};
		uint64_t position_timestamp{0};
		uint8_t z_reset_counter{0}, vz_reset_counter{0};
		bool enabled{false}, eligible{false}, armed{false}, landed{true}, fresh{false};
		bool controllable{false}, vertical_valid{false}, position_valid{false}, failsafe{false};
		bool mode_allowed{false};
		uint8_t mode{0};
		uint32_t trigger{0};
		float q[4] {1.f, 0.f, 0.f, 0.f};
		float rates[3] {};
		float normal_rates[3] {};
		float normal_thrust{-0.5f};
		float z{0.f}, vz{0.f}, hover_thrust{0.5f};
		float max_rate{2.f}, damping{0.8f};
		float arbitration_weight{0.f};
	};
	struct Output {
		uint8_t state{DISABLED}, original_mode{0};
		bool candidate_valid{false}, reentry_ready{false};
		float rate[3] {}, thrust{-0.5f}, q[4] {1.f, 0.f, 0.f, 0.f};
		float vertical_speed{0.f}, altitude_reference{0.f}, reentry_weight{1.f}, elapsed{0.f};
		uint32_t fallback_reason{0};
	};
	void update(float dt, const Input &in)
	{
		dt = fminf(fmaxf(dt, 0.001f), 0.1f);
		_output.candidate_valid = _output.reentry_ready = false;
		if (_output.state == DISABLED || _output.state == MONITORING) { _output.fallback_reason = 0; }
		const bool was_recovering = isRecovering();
		bool vertical_state_reset = false;
		if (in.position_timestamp) {
			// Establish the current frame before entry. During recovery any counter
			// change (including wrap or missed resets) invalidates the held target.
			vertical_state_reset = _vertical_reset_initialized && (in.position_timestamp < _position_timestamp
				|| in.z_reset_counter != _z_reset_counter || in.vz_reset_counter != _vz_reset_counter);
			_position_timestamp = in.position_timestamp;
			_z_reset_counter = in.z_reset_counter;
			_vz_reset_counter = in.vz_reset_counter;
			_vertical_reset_initialized = true;
		}
		float norm = 0.f;
		for (float v : in.q) { norm += v*v; }
		bool finite = isfinite(norm) && norm > 0.9f && norm < 1.1f && isfinite(in.normal_thrust);
		finite &= (!in.vertical_valid || isfinite(in.vz)) && (!in.position_valid || isfinite(in.z));
		finite &= isfinite(in.hover_thrust) && in.hover_thrust > 0.f && in.hover_thrust < 1.f;
		finite &= isfinite(in.max_rate) && in.max_rate > 0.f && isfinite(in.damping) && in.damping >= 0.f;
		for (unsigned a = 0; a < 3; ++a) { finite &= isfinite(in.rates[a]) && isfinite(in.normal_rates[a]); }
		if (!in.enabled) { transition(DISABLED, in.now); }
		else if (!in.armed && in.landed && in.fresh) { transition(MONITORING, in.now); _trigger_latched = false; }
		else if (_output.state == DISABLED && in.eligible) { transition(MONITORING, in.now); }
		if (!in.trigger) { _trigger_latched = false; }
		// Do not consume a trigger until the complete wrapper eligibility gate,
		// including minimum entry altitude, is satisfied in this same cycle.
		if (_output.state == MONITORING && in.eligible && finite && in.armed && !in.landed
		    && in.trigger && !_trigger_latched) {
			_output.original_mode = in.mode;
			_recovery_started = in.now;
			_trigger_latched = true;
			_altitude = in.z;
			_vertical_integral = 0.f;
			transition(DISTURBANCE_DETECTED, in.now);
		}
		const float tilt_cos = 1.f - 2.f * (in.q[1]*in.q[1] + in.q[2]*in.q[2]);
		const bool recovering = isRecovering();
		if (recovering) {
			if (was_recovering && vertical_state_reset) {
				// Do not apply delta_z/delta_vz: a latest sample may omit intermediate
				// resets. Abort rather than manufacture motion in an unknown frame.
				_output.fallback_reason = VERTICAL_STATE_RESET;
				_vertical_integral = 0.f;
				transition(ABORTED, in.now);
			} else if (!in.fresh || !finite || in.failsafe || !in.mode_allowed || in.mode != _output.original_mode || !in.armed || in.landed) {
				_output.fallback_reason = 1;
				transition(ABORTED, in.now);
			} else if (!in.controllable) {
				_output.fallback_reason = 2;
				transition(FAILED, in.now);
			} else if (in.now - _recovery_started > 20000000 && _output.state != EMERGENCY_LAND) {
				_output.fallback_reason = 4;
				transition(in.vertical_valid && tilt_cos > 0.9f ? EMERGENCY_LAND : FAILED, in.now);
			}
		}
		const float yaw_norm = sqrtf(in.q[0]*in.q[0] + in.q[3]*in.q[3]);
		_output.q[0] = yaw_norm > 0.01f ? in.q[0] / yaw_norm : 1.f;
		_output.q[1] = _output.q[2] = 0.f;
		_output.q[3] = yaw_norm > 0.01f ? in.q[3] / yaw_norm : 0.f;
		const float rp = sqrtf(in.rates[0]*in.rates[0] + in.rates[1]*in.rates[1]);
		const bool rate_safe = rp < 0.4f && fabsf(in.rates[2]) < 1.f;
		const bool attitude_safe = tilt_cos > 0.97f;
		const bool vertical_safe = in.vertical_valid && fabsf(in.vz) < 0.3f;
		bool stable = false;
		switch (_output.state) {
		case DISTURBANCE_DETECTED:
			if (in.eligible) { transition(RATE_DAMPING, in.now); }
			else if (in.now - _entered > 1000000) { transition(ABORTED, in.now); }
			break;
		case RATE_DAMPING: stable = rate_safe; break;
		case THRUST_VECTOR_RECOVERY: stable = rate_safe && tilt_cos > 0.9f; break;
		case ATTITUDE_RECOVERY: stable = rate_safe && attitude_safe; break;
		case VERTICAL_SPEED_RECOVERY: stable = rate_safe && attitude_safe && vertical_safe; break;
		case ALTITUDE_STABILIZATION:
			stable = rate_safe && attitude_safe && vertical_safe && fabsf(in.z - _altitude) < 0.5f; break;
		case CONTROL_REENTRY:
			stable = rate_safe && attitude_safe && vertical_safe && in.eligible
				&& fabsf(in.normal_thrust - _output.thrust) < 0.2f;
			for (unsigned a = 0; a < 3; ++a) { stable &= fabsf(in.normal_rates[a] - _output.rate[a]) < 0.5f; }
			_output.reentry_ready = stable;
			break;
		default: break;
		}
		_stable = stable ? _stable + dt : 0.f;
		if (_stable > (_output.state == ALTITUDE_STABILIZATION ? 2.f : 0.5f)) {
			switch (_output.state) {
			case RATE_DAMPING: transition(THRUST_VECTOR_RECOVERY, in.now); break;
			case THRUST_VECTOR_RECOVERY: transition(ATTITUDE_RECOVERY, in.now); break;
			case ATTITUDE_RECOVERY: transition(in.vertical_valid ? VERTICAL_SPEED_RECOVERY : FAILED, in.now); break;
			case VERTICAL_SPEED_RECOVERY:
				_altitude = in.z;
				transition(in.position_valid ? ALTITUDE_STABILIZATION : EMERGENCY_LAND, in.now);
				break;
			case ALTITUDE_STABILIZATION: transition(CONTROL_REENTRY, in.now); break;
			case CONTROL_REENTRY:
				if (in.arbitration_weight < 0.001f && in.now - _entered > 3000000) { transition(MONITORING, in.now); }
				break;
			default: break;
			}
		}
		if (_output.state == EMERGENCY_LAND && (!rate_safe || tilt_cos < 0.85f)) {
			_output.fallback_reason = 8; transition(FAILED, in.now);
		}
		if ((_output.state == ALTITUDE_STABILIZATION || _output.state == VERTICAL_SPEED_RECOVERY || _output.state == CONTROL_REENTRY)
		    && (!rate_safe || tilt_cos < 0.85f)) { transition(RATE_DAMPING, in.now); }
		_output.elapsed = _entered ? (in.now - _entered) * 1e-6f : 0.f;
		_output.altitude_reference = _altitude;
		_output.reentry_weight = _output.state == CONTROL_REENTRY && _output.reentry_ready
			? fmaxf(0.f, 1.f - fmaxf(0.f, _stable - 0.5f) / 2.f) : 1.f;
		_output.candidate_valid = in.eligible && finite && (_output.state == RATE_DAMPING
			|| _output.state == THRUST_VECTOR_RECOVERY || _output.state == ATTITUDE_RECOVERY
			|| _output.state == VERTICAL_SPEED_RECOVERY || _output.state == ALTITUDE_STABILIZATION
			|| _output.state == CONTROL_REENTRY || _output.state == EMERGENCY_LAND);
		if (!_output.candidate_valid) { return; }
		float target[3] {};
		for (unsigned a = 0; a < 3; ++a) { target[a] = -in.damping * in.rates[a] * (a == 2 ? 0.3f : 1.f); }
		if (_output.state != RATE_DAMPING) {
			// q_error = conjugate(q) * q_level; bounded quaternion error avoids Euler singularities.
			float error[3] {
				-in.q[1]*_output.q[0] - in.q[2]*_output.q[3],
				-in.q[2]*_output.q[0] + in.q[1]*_output.q[3],
				in.q[0]*_output.q[3] - in.q[3]*_output.q[0]
			};
			const float w = in.q[0]*_output.q[0] + in.q[3]*_output.q[3];
			for (unsigned a = 0; a < 2; ++a) { target[a] = 4.f * (w >= 0.f ? error[a] : -error[a]); }
			target[2] = -0.2f * in.rates[2];
		}
		for (unsigned a = 0; a < 3; ++a) {
			const float limit = a == 2 ? fminf(in.max_rate, 0.7f) : in.max_rate;
			target[a] = clamp(target[a], -limit, limit);
			_output.rate[a] += clamp(target[a] - _output.rate[a], -4.f*dt, 4.f*dt);
		}
		float thrust = clamp(-in.normal_thrust, 0.1f, 0.9f);
		const bool vertical_phase = _output.state == VERTICAL_SPEED_RECOVERY || _output.state == ALTITUDE_STABILIZATION
			|| _output.state == CONTROL_REENTRY || _output.state == EMERGENCY_LAND;
		if (vertical_phase) {
			if (!in.vertical_valid) { transition(FAILED, in.now); _output.candidate_valid = false; return; }
			_output.vertical_speed = _output.state == EMERGENCY_LAND ? 0.7f
				: (_output.state == VERTICAL_SPEED_RECOVERY ? 0.f : clamp((_altitude - in.z) * 0.8f, -1.f, 1.f));
			const float error = _output.vertical_speed - in.vz;
			const float acceleration = clamp(2.f * error + _vertical_integral, -3.f, 3.f);
			const float raw = in.hover_thrust * (1.f - acceleration / 9.81f) / fmaxf(tilt_cos, 0.3f);
			thrust = clamp(raw, 0.1f, 0.9f);
			if (raw > 0.1f && raw < 0.9f) { _vertical_integral = clamp(_vertical_integral + 0.5f * error * dt, -2.f, 2.f); }
		}
		_output.thrust += clamp(-thrust - _output.thrust, -0.5f*dt, 0.5f*dt);
	}
	const Output &output() const { return _output; }
private:
	bool isRecovering() const
	{
		return _output.state != DISABLED && _output.state != MONITORING
			&& _output.state != ABORTED && _output.state != FAILED;
	}
	static float clamp(float value, float lo, float hi) { return fminf(fmaxf(value, lo), hi); }
	void transition(uint8_t state, uint64_t now)
	{
		if (_output.state != state) { _output.state = state; _entered = now; _stable = 0.f; }
	}
	Output _output{};
	uint64_t _entered{0}, _recovery_started{0};
	uint64_t _position_timestamp{0};
	uint8_t _z_reset_counter{0}, _vz_reset_counter{0};
	float _stable{0.f}, _altitude{0.f}, _vertical_integral{0.f};
	bool _trigger_latched{false};
	bool _vertical_reset_initialized{false};
};
