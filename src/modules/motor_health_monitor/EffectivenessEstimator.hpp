/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include <stdint.h>

class EffectivenessEstimator
{
public:
	static constexpr uint8_t MAX_MOTORS = 12;
	static constexpr uint8_t AXES = 3;
	enum State : uint8_t { UNINITIALIZED, CALIBRATING, BASELINE_LEARNED, OBSERVABLE,
		TEMPORARILY_UNOBSERVABLE, VALID, STALE, INVALID };
	struct Configuration {
		float lpf_time_constant{0.2f};
		float excitation_threshold{0.025f};
		float baseline_time{5.f};
		float effectiveness_rate_limit{0.3f};
		float forgetting_factor{0.995f};
		float minimum_effectiveness{0.1f};
		float stale_time{10.f};
		float confidence_minimum{0.6f};
		float residual_limit{1.f};
	};
	struct Input {
		uint64_t timestamp{0};
		uint8_t motor_count{0};
		float control[MAX_MOTORS] {};
		float angular_acceleration[AXES] {};
		float geometry[AXES][MAX_MOTORS] {};
		bool valid{false};
		bool aligned{false};
		bool saturated{false};
		bool authority_limited{false};
		float headroom{0.f};
		float control_residual{0.f};
	};
	struct Output {
		float effectiveness[MAX_MOTORS] {};
		float confidence[MAX_MOTORS] {};
		float uncertainty[MAX_MOTORS] {};
		float residual[MAX_MOTORS] {};
		float predicted_response[AXES] {};
		float measured_response[AXES] {};
		float model_residual{0.f};
		float prediction_residual{0.f};
		float excitation{0.f};
		float model_quality{0.f};
		float condition_number{0.f};
		float estimate_age{0.f};
		uint64_t last_valid_timestamp{0};
		uint32_t update_count{0};
		uint32_t reset_count{0};
		uint8_t motor_count{0};
		uint8_t state{UNINITIALIZED};
		bool baseline_learned{false};
		bool current_observable{false};
		bool estimate_valid{false};
		bool update_allowed{false};
	};
	virtual ~EffectivenessEstimator() = default;
	virtual void reset() = 0;
	virtual void update(float dt, const Input &input, const Configuration &configuration) = 0;
	virtual const Output &output() const = 0;
};
