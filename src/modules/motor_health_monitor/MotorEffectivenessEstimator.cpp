/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#include "MotorEffectivenessEstimator.hpp"
#include <math.h>
#include <string.h>

namespace
{
float bound(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }
constexpr float ResponseNoise = 0.05f; // rad/s^2 floor: noiseless simulation must not imply zero uncertainty
constexpr float ProcessVariance = 0.0004f; // lambda^2/s random walk, also applied without observations
constexpr float ConfidenceScale = 0.15f; // uncertainty score scale; not a calibrated probability
}

void MotorEffectivenessEstimator::reset()
{
	const uint32_t resets = _output.reset_count + 1;
	_output = {};
	_output.reset_count = resets;
	_output.estimate_age = INFINITY;
	memset(_covariance, 0, sizeof(_covariance));
	memset(_information, 0, sizeof(_information));
	memset(_axis_energy, 0, sizeof(_axis_energy));
	memset(_axis_cross, 0, sizeof(_axis_cross));
	memset(_axis_gain, 0, sizeof(_axis_gain));
	for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
		_covariance[i][i] = 1.f;
		_output.effectiveness[i] = 1.f;
		_output.uncertainty[i] = 1.f;
	}
	_baseline_elapsed = _information_min = 0.f;
	_last_update = _last_sample = 0;
	_samples = 0;
	_initialized = false;
}

void MotorEffectivenessEstimator::updateCondition(uint8_t count)
{
	auto &a = _eigen_workspace;
	memcpy(a, _information, sizeof(a));
	// Bounded symmetric Jacobi sweep, run every ten monitor samples.
	for (unsigned sweep = 0; sweep < 8; ++sweep) {
		for (uint8_t p = 0; p < count; ++p) {
			for (uint8_t q = p + 1; q < count; ++q) {
				if (fabsf(a[p][q]) < 1e-9f) { continue; }
				const float angle = 0.5f * atan2f(2.f * a[p][q], a[q][q] - a[p][p]);
				const float c = cosf(angle), s = sinf(angle);
				const float pp = a[p][p], qq = a[q][q], pq = a[p][q];
				for (uint8_t k = 0; k < count; ++k) {
					if (k != p && k != q) {
						const float kp = a[k][p], kq = a[k][q];
						a[k][p] = a[p][k] = c * kp - s * kq;
						a[k][q] = a[q][k] = s * kp + c * kq;
					}
				}
				a[p][p] = c*c*pp - 2.f*s*c*pq + s*s*qq;
				a[q][q] = s*s*pp + 2.f*s*c*pq + c*c*qq;
				a[p][q] = a[q][p] = 0.f;
			}
		}
	}
	float largest = 0.f;
	_information_min = INFINITY;
	for (uint8_t i = 0; i < count; ++i) {
		largest = fmaxf(largest, a[i][i]);
		_information_min = fminf(_information_min, a[i][i]);
	}
	_output.condition_number = _information_min > 1e-8f ? largest / _information_min : INFINITY;
}

void MotorEffectivenessEstimator::update(float dt, const Input &input, const Configuration &cfg)
{
	const uint8_t n = input.motor_count;
	bool finite = n >= 2 && n <= MAX_MOTORS && input.timestamp > 0;
	for (uint8_t i = 0; i < n && i < MAX_MOTORS; ++i) {
		finite = finite && isfinite(input.control[i]);
		for (uint8_t a = 0; a < AXES; ++a) { finite = finite && isfinite(input.geometry[a][i]); }
	}
	for (uint8_t a = 0; a < AXES; ++a) { finite = finite && isfinite(input.angular_acceleration[a]); }
	if (!finite || (_last_sample && input.timestamp < _last_sample)) {
		reset(); _output.state = INVALID; return;
	}
	if (input.timestamp == _last_sample) { return; }
	bool geometry_changed = _output.motor_count && n != _output.motor_count;
	for (uint8_t a = 0; a < AXES; ++a) {
		for (uint8_t i = 0; i < n; ++i) {
			geometry_changed |= _initialized && fabsf(input.geometry[a][i] - _geometry[a][i]) > 1e-5f;
		}
	}
	if (geometry_changed) { reset(); }
	_last_sample = input.timestamp;
	_output.motor_count = n;
	dt = bound(dt, 0.001f, 0.1f);
	_output.current_observable = _output.update_allowed = _output.estimate_valid = false;
	for (uint8_t i = 0; i < n; ++i) { _covariance[i][i] = fminf(_covariance[i][i] + ProcessVariance * dt, 4.f); }
	const float alpha = dt / (fmaxf(cfg.lpf_time_constant, 0.01f) + dt);
	const float mean_alpha = dt / (2.f + dt);
	float du[MAX_MOTORS] {}, response[AXES] {}, phi[AXES][MAX_MOTORS] {};
	float energy = 0.f;
	if (!_initialized && input.valid && input.aligned) {
		memcpy(_filtered_control, input.control, sizeof(_filtered_control));
		memcpy(_control_mean, input.control, sizeof(_control_mean));
		memcpy(_filtered_response, input.angular_acceleration, sizeof(_filtered_response));
		memcpy(_response_mean, input.angular_acceleration, sizeof(_response_mean));
		memcpy(_geometry, input.geometry, sizeof(_geometry));
		_initialized = true;
	}
	if (_initialized && input.valid && input.aligned) {
		for (uint8_t i = 0; i < n; ++i) {
			_filtered_control[i] += alpha * (input.control[i] - _filtered_control[i]);
			du[i] = _filtered_control[i] - _control_mean[i];
			_control_mean[i] += mean_alpha * du[i];
			energy += du[i] * du[i];
		}
		for (uint8_t a = 0; a < AXES; ++a) {
			_filtered_response[a] += alpha * (input.angular_acceleration[a] - _filtered_response[a]);
			response[a] = _filtered_response[a] - _response_mean[a];
			_response_mean[a] += mean_alpha * response[a];
			for (uint8_t i = 0; i < n; ++i) { phi[a][i] = input.geometry[a][i] * du[i]; }
		}
	}
	_output.excitation = sqrtf(energy / n) / fmaxf(cfg.excitation_threshold, 0.001f);
	const bool sample_allowed = input.valid && input.aligned && !input.saturated && !input.authority_limited;
	const bool excited = sample_allowed && _output.excitation > 1.f;
	const float decay = expf(-dt / 20.f);
	for (uint8_t i = 0; i < n; ++i) {
		for (uint8_t j = 0; j < n; ++j) {
			_information[i][j] *= decay;
			if (excited) {
				for (uint8_t a = 0; a < AXES; ++a) { _information[i][j] += dt * phi[a][i] * phi[a][j]; }
			}
		}
	}
	if (++_samples % 10 == 0) { updateCondition(n); }
	const bool conditioned = _information_min > 1e-6f && _output.condition_number < 1000.f;
	_output.current_observable = excited && conditioned;
	if (!_output.baseline_learned && excited) {
		bool gains_valid = true;
		for (uint8_t a = 0; a < AXES; ++a) {
			float nominal = 0.f;
			for (uint8_t i = 0; i < n; ++i) { nominal += phi[a][i]; }
			_axis_energy[a] += dt * nominal * nominal;
			_axis_cross[a] += dt * nominal * response[a];
			_axis_gain[a] = _axis_cross[a] / fmaxf(_axis_energy[a], 1e-8f);
			gains_valid &= _axis_energy[a] > 0.001f && _axis_gain[a] > 0.01f && _axis_gain[a] < 10000.f;
		}
		_baseline_elapsed += dt;
		if (_baseline_elapsed >= cfg.baseline_time && conditioned && gains_valid) { _output.baseline_learned = true; }
	}
	_output.update_allowed = _output.baseline_learned && _output.current_observable;
	float prediction_error_sq = 0.f, response_sq = 0.f;
	float next_effectiveness[MAX_MOTORS];
	memcpy(next_effectiveness, _output.effectiveness, sizeof(next_effectiveness));
	if (_output.update_allowed) {
		const float forgetting = powf(bound(cfg.forgetting_factor, 0.9f, 1.f), dt / 0.02f);
		for (uint8_t i = 0; i < n; ++i) {
			for (uint8_t j = 0; j < n; ++j) { _covariance[i][j] /= forgetting; }
		}
	}
	for (uint8_t a = 0; a < AXES; ++a) {
		float reg[MAX_MOTORS] {}, prediction = 0.f;
		for (uint8_t i = 0; i < n; ++i) {
			reg[i] = phi[a][i] * _axis_gain[a];
			prediction += reg[i] * _output.effectiveness[i];
		}
		_output.predicted_response[a] = prediction;
		_output.measured_response[a] = response[a];
		const float error = response[a] - prediction;
		prediction_error_sq += error * error;
		response_sq += response[a] * response[a];
		if (_output.update_allowed) {
			float pr[MAX_MOTORS] {}, denominator = ResponseNoise * ResponseNoise / dt;
			float sequential_prediction = 0.f;
			for (uint8_t i = 0; i < n; ++i) {
				sequential_prediction += reg[i] * next_effectiveness[i];
				for (uint8_t j = 0; j < n; ++j) { pr[i] += _covariance[i][j] * reg[j]; }
				denominator += reg[i] * pr[i];
			}
			if (!(denominator > 0.f) || !isfinite(denominator)) { reset(); _output.state = INVALID; return; }
			for (uint8_t i = 0; i < n; ++i) {
				next_effectiveness[i] += pr[i] / denominator * (response[a] - sequential_prediction);
				for (uint8_t j = 0; j < n; ++j) { _covariance[i][j] -= pr[i] * pr[j] / denominator; }
			}
		}
	}
	if (input.valid && input.aligned && _output.baseline_learned) {
		_output.prediction_residual = sqrtf(prediction_error_sq);
		const float normalized = sqrtf(prediction_error_sq) / fmaxf(sqrtf(response_sq), 1.f);
		_output.model_residual += mean_alpha * (normalized - _output.model_residual);
	}
	if (_output.update_allowed) {
		_last_update = input.timestamp;
		++_output.update_count;
		for (uint8_t i = 0; i < n; ++i) {
			const float step = cfg.effectiveness_rate_limit * dt;
			_output.effectiveness[i] = bound(_output.effectiveness[i]
				+ bound(next_effectiveness[i] - _output.effectiveness[i], -step, step), cfg.minimum_effectiveness, 1.f);
		}
	}
	_output.estimate_age = _last_update ? (input.timestamp - _last_update) * 1e-6f : INFINITY;
	float confidence_min = 1.f;
	for (uint8_t i = 0; i < n; ++i) {
		_output.uncertainty[i] = sqrtf(fmaxf(_covariance[i][i], 0.f)) * fmaxf(1.f, _output.model_residual / ResponseNoise);
		const float relative = _output.uncertainty[i] / ConfidenceScale;
		_output.confidence[i] = _output.baseline_learned ? expf(-0.5f * relative * relative) : 0.f;
		_output.residual[i] = 1.f - _output.effectiveness[i];
		confidence_min = fminf(confidence_min, _output.confidence[i]);
	}
	// Only an accepted observation can certify a new estimate. Quiet residuals cannot certify an old, invalid update.
	const bool certified = _output.update_allowed || (_last_update && _last_update <= _output.last_valid_timestamp);
	_output.estimate_valid = input.valid && certified && _output.baseline_learned && _output.estimate_age <= cfg.stale_time
		&& confidence_min >= cfg.confidence_minimum && _output.model_residual <= cfg.residual_limit;
	_output.model_quality = _output.estimate_valid ? confidence_min / (1.f + _output.model_residual) : 0.f;
	if (_output.estimate_valid && _output.update_allowed) { _output.last_valid_timestamp = input.timestamp; }
	_output.state = !input.valid ? INVALID : (!_initialized ? UNINITIALIZED
		: (!_output.baseline_learned ? CALIBRATING
		   : (_output.estimate_age > cfg.stale_time ? STALE
		      : (!_output.current_observable ? TEMPORARILY_UNOBSERVABLE
			 : (_output.estimate_valid ? VALID : OBSERVABLE)))));
}
