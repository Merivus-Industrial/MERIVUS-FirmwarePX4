/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include "FtcRecoveryArbiter.hpp"
#include "FtcRecoveryController.hpp"
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ftc_recovery_status.h>
#include <uORB/topics/ftc_arbitration_status.h>
#include <uORB/topics/ftc_control_authority.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <matrix/matrix/math.hpp>

class FtcRateInput : public ModuleParams
{
public:
	explicit FtcRateInput(ModuleParams *parent) : ModuleParams(parent) {}
	void select(uint64_t now, float dt, uint64_t normal_timestamp, const vehicle_status_s &vehicle,
		    const vehicle_control_mode_s &mode, bool landed, matrix::Vector3f &rates, matrix::Vector3f &thrust)
	{
		if (normal_timestamp) { _normal_timestamp = normal_timestamp; }
		ftc_recovery_status_s candidate{};
		ftc_control_authority_s authority{};
		_candidate_sub.copy(&candidate);
		_authority_sub.copy(&authority);
		FtcRecoveryArbiter::Input input{};
		input.now = now;
		input.normal_timestamp = _normal_timestamp;
		input.candidate_timestamp = candidate.timestamp;
		input.enabled = _param_ftc_mon_en.get() && _param_ftc_rec_en.get() && _param_ftc_rec_act.get();
		input.hard_exit = !mode.flag_armed || landed || vehicle.failsafe || mode.flag_control_termination_enabled
			|| vehicle.is_vtol || !mode.flag_control_rates_enabled;
		input.flight_allowed = !input.hard_exit && vehicle.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			&& vehicle.timestamp && now >= vehicle.timestamp && now - vehicle.timestamp < 1000000
			&& authority.timestamp && now >= authority.timestamp && now - authority.timestamp < 200000
			&& authority.valid && authority.matrix_valid && authority.minimum_attitude_authority >= 0.35f
			&& authority.thrust_authority >= 0.25f;
		input.mode_matches = candidate.original_nav_state == vehicle.nav_state;
		input.hard_exit |= _arbiter.active && !input.mode_matches;
		// A candidate computed before an estimator vertical reset must not remain
		// in the arbiter's held setpoint during its normal gradual release.
		input.hard_exit |= FtcRecoveryController::requiresImmediateExit(candidate.fallback_reason);
		input.candidate_valid = candidate.candidate_valid && candidate.eligible && candidate.intervention_enabled;
		input.reentry = candidate.state == ftc_recovery_status_s::CONTROL_REENTRY;
		input.requested_weight = input.reentry ? candidate.reentry_weight : 1.f;
		for (unsigned i = 0; i < 3; ++i) { input.normal[i] = rates(i); input.candidate[i] = candidate.body_rate_setpoint[i]; }
		input.normal[3] = thrust(2);
		input.candidate[3] = candidate.thrust_body[2];
		_arbiter.update(dt, input);
		for (unsigned i = 0; i < 3; ++i) { rates(i) = _arbiter.output[i]; }
		thrust(2) = _arbiter.output[3];
		if (now - _last_publish >= 20000) {
			ftc_arbitration_status_s status{};
			status.timestamp = now;
			status.candidate_timestamp = candidate.timestamp;
			status.active = _arbiter.active;
			status.weight = _arbiter.weight;
			status.reentry = input.reentry && _arbiter.active;
			status.fallback_reason = _arbiter.reason;
			rates.copyTo(status.selected_rates);
			thrust.copyTo(status.selected_thrust);
			_status_pub.publish(status);
			_last_publish = now;
		}
	}
private:
	FtcRecoveryArbiter _arbiter{};
	uint64_t _normal_timestamp{0}, _last_publish{0};
	uORB::Subscription _candidate_sub{ORB_ID(ftc_recovery_status)};
	uORB::Subscription _authority_sub{ORB_ID(ftc_control_authority)};
	uORB::Publication<ftc_arbitration_status_s> _status_pub{ORB_ID(ftc_arbitration_status)};
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTC_MON_EN>) _param_ftc_mon_en,
		(ParamBool<px4::params::FTC_REC_EN>) _param_ftc_rec_en,
		(ParamBool<px4::params::FTC_REC_ACT>) _param_ftc_rec_act
	)
};
