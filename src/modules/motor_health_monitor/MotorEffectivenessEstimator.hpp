/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include "EffectivenessEstimator.hpp"

class MotorEffectivenessEstimator : public EffectivenessEstimator
{
public:
	MotorEffectivenessEstimator() { reset(); }
	void reset() override;
	void update(float dt, const Input &input, const Configuration &configuration) override;
	const Output &output() const override { return _output; }
private:
	void updateCondition(uint8_t count);
	float _covariance[MAX_MOTORS][MAX_MOTORS] {};
	float _information[MAX_MOTORS][MAX_MOTORS] {};
	float _eigen_workspace[MAX_MOTORS][MAX_MOTORS] {};
	float _filtered_control[MAX_MOTORS] {};
	float _control_mean[MAX_MOTORS] {};
	float _filtered_response[AXES] {};
	float _response_mean[AXES] {};
	float _axis_energy[AXES] {};
	float _axis_cross[AXES] {};
	float _axis_gain[AXES] {};
	float _geometry[AXES][MAX_MOTORS] {};
	float _baseline_elapsed{0.f};
	float _information_min{0.f};
	uint64_t _last_update{0};
	uint64_t _last_sample{0};
	uint32_t _samples{0};
	bool _initialized{false};
	Output _output{};
};
