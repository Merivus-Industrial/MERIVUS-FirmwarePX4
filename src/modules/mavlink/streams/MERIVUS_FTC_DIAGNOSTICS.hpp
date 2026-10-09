/****************************************************************************
 * Copyright (c) 2026 MERIVUS. All rights reserved.
 ****************************************************************************/

#ifndef MERIVUS_FTC_DIAGNOSTICS_HPP
#define MERIVUS_FTC_DIAGNOSTICS_HPP

#include "MerivusFtcTelemetry.hpp"

#include <uORB/topics/ftc_allocation_shadow.h>
#include <uORB/topics/ftc_extreme_state.h>
#include <uORB/topics/ftc_model_status.h>
#include <uORB/topics/ftc_simulation_status.h>
#include <uORB/topics/ftc_system_status.h>
#include <uORB/topics/motor_health_status.h>

class MavlinkStreamMerivusFtcDiagnostics : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMerivusFtcDiagnostics(mavlink); }
	static constexpr const char *get_name_static() { return "MERIVUS_FTC_DIAGNOSTICS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MERIVUS_FTC_DIAGNOSTICS; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _motor_sub.advertised() ? MAVLINK_MSG_ID_MERIVUS_FTC_DIAGNOSTICS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMerivusFtcDiagnostics(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _motor_sub{ORB_ID(motor_health_status)};
	uORB::Subscription _model_sub{ORB_ID(ftc_model_status)};
	uORB::Subscription _shadow_sub{ORB_ID(ftc_allocation_shadow)};
	uORB::Subscription _extreme_sub{ORB_ID(ftc_extreme_state)};
	uORB::Subscription _simulation_sub{ORB_ID(ftc_simulation_status)};
	uORB::Subscription _system_sub{ORB_ID(ftc_system_status)};
	motor_health_status_s _motor{};
	ftc_model_status_s _model{};
	ftc_allocation_shadow_s _shadow{};
	ftc_extreme_state_s _extreme{};
	ftc_simulation_status_s _simulation{};
	ftc_system_status_s _system{};

	bool send() override
	{
		bool updated = _motor_sub.update(&_motor);
		updated |= _model_sub.update(&_model);
		updated |= _shadow_sub.update(&_shadow);
		updated |= _extreme_sub.update(&_extreme);
		updated |= _simulation_sub.update(&_simulation);
		updated |= _system_sub.update(&_system);

		if (!updated) {
			return false;
		}

		mavlink_merivus_ftc_diagnostics_t msg{};
		msg.time_usec = _motor.timestamp > 0 ? _motor.timestamp : _system.timestamp;
		msg.model_residual = _motor.model_residual;
		msg.excitation = _model.timestamp > 0 ? _model.excitation : _motor.excitation;
		msg.maneuver_intensity = _motor.maneuver_intensity;
		msg.external_disturbance_score = _motor.external_disturbance_score;
		msg.vibration_score = _motor.vibration_score;
		msg.allocation_residual_norm = _shadow.residual_norm;
		msg.attitude_error = _extreme.attitude_error;
		msg.rate_error = _extreme.rate_error;
		msg.jerk = _extreme.jerk;
		msg.acceleration_magnitude = _extreme.acceleration_magnitude;
		msg.angular_rate = _extreme.angular_rate;
		msg.angular_acceleration = _extreme.angular_acceleration;
		msg.system_reason_mask = _system.reason_mask;
		msg.protocol_version = merivus_ftc_telemetry::ProtocolVersion;
		msg.flags = (_model.valid ? MERIVUS_FTC_DIAGNOSTIC_FLAGS_MODEL_VALID : 0)
			| (_shadow.valid ? MERIVUS_FTC_DIAGNOSTIC_FLAGS_SHADOW_VALID : 0)
			| (_extreme.valid ? MERIVUS_FTC_DIAGNOSTIC_FLAGS_EXTREME_VALID : 0)
			| (_simulation.enabled ? MERIVUS_FTC_DIAGNOSTIC_FLAGS_SIMULATION_ENABLED : 0)
			| (_simulation.intermittent ? MERIVUS_FTC_DIAGNOSTIC_FLAGS_SIMULATION_INTERMITTENT : 0);
		msg.simulation_motor_index = _simulation.motor_index;
		msg.simulation_target_effectiveness_pct = merivus_ftc_telemetry::encode_percentage(
				_simulation.target_effectiveness, _simulation.enabled);
		msg.simulation_applied_effectiveness_pct = merivus_ftc_telemetry::encode_percentage(
				_simulation.applied_effectiveness, _simulation.enabled);

		msg.condition_number = _model.condition_number;
		msg.rigid_body_activity = _model.rigid_body_activity;
		msg.model_prediction_residual = _model.model_prediction_residual;
		msg.update_count = _model.update_count;
		msg.reset_count = _model.reset_count;
		msg.mass = _model.mass;
		memcpy(msg.inertia, _model.inertia, sizeof(msg.inertia));
		memcpy(msg.cg_offset, _model.cg_offset, sizeof(msg.cg_offset));
		msg.mass_state = _model.mass_state;
		msg.inertia_state = _model.inertia_state;
		msg.cg_state = _model.cg_state;
		msg.estimator_flags = (_model.update_allowed ? 1 : 0) | (_model.saturated ? 2 : 0)
			| (_model.authority_limited ? 4 : 0) | (_model.timing_aligned ? 8 : 0);
		mavlink_msg_merivus_ftc_diagnostics_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MERIVUS_FTC_DIAGNOSTICS_HPP
