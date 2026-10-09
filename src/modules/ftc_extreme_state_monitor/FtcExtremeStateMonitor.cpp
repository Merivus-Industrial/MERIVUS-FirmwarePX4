/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#include "FtcExtremeStateMonitor.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

#include <math.h>
#include <string.h>

namespace
{
constexpr uint32_t REASON_ATTITUDE_ERROR = 1u << 0;
constexpr uint32_t REASON_RATE_ERROR = 1u << 1;
constexpr uint32_t REASON_SATURATION = 1u << 2;
constexpr uint32_t REASON_AUTHORITY = 1u << 3;
constexpr uint32_t REASON_PROPULSION = 1u << 4;
constexpr uint32_t REASON_VERTICAL_MOTION = 1u << 5;
}

FtcExtremeStateMonitor::FtcExtremeStateMonitor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

FtcExtremeStateMonitor::~FtcExtremeStateMonitor()
{
	ScheduleClear();
}

int FtcExtremeStateMonitor::task_spawn(int argc, char *argv[])
{
	FtcExtremeStateMonitor *instance = new FtcExtremeStateMonitor();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->start()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool FtcExtremeStateMonitor::start()
{
	ScheduleOnInterval(20_ms);
	return true;
}

float FtcExtremeStateMonitor::quaternionError(const float q[4], const float q_d[4]) const
{
	const float dot = fabsf(q[0] * q_d[0] + q[1] * q_d[1] + q[2] * q_d[2] + q[3] * q_d[3]);
	return 2.f * acosf(math::constrain(dot, 0.f, 1.f));
}

void FtcExtremeStateMonitor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();
	vehicle_acceleration_s acceleration{};
	vehicle_angular_velocity_s angular_velocity{};
	vehicle_attitude_s attitude{};
	vehicle_attitude_setpoint_s attitude_sp{};
	vehicle_rates_setpoint_s rates_sp{};
	vehicle_local_position_s local_position{};
	vehicle_land_detected_s land{};
	vehicle_status_s vehicle_status{};
	actuator_motors_s motors{};
	control_allocator_status_s allocator_status{};
	ftc_control_authority_s authority{};
	motor_health_status_s health{};
	const bool acceleration_updated = _acceleration_sub.copy(&acceleration);
	const bool angular_velocity_updated = _angular_velocity_sub.copy(&angular_velocity);
	_attitude_sub.copy(&attitude);
	_attitude_sp_sub.copy(&attitude_sp);
	_rates_sp_sub.copy(&rates_sp);
	_local_position_sub.copy(&local_position);
	_land_sub.copy(&land);
	_vehicle_status_sub.copy(&vehicle_status);
	_motors_sub.copy(&motors);
	_allocator_status_sub.copy(&allocator_status);
	_authority_sub.copy(&authority);
	_health_sub.copy(&health);

	ftc_extreme_state_s status{};
	status.timestamp = now;
	status.event_timestamp = _last_impact;
	status.loc_state = ftc_extreme_state_s::LOC_NORMAL;
	status.impact_type = ftc_extreme_state_s::IMPACT_NONE;
	status.valid = acceleration_updated && angular_velocity_updated && acceleration.timestamp != 0
		       && angular_velocity.timestamp != 0 && now >= acceleration.timestamp && now - acceleration.timestamp < 200_ms
		       && now >= angular_velocity.timestamp && now - angular_velocity.timestamp < 200_ms;

	if (!_param_ftc_impact_en.get() && !_param_ftc_loc_en.get()) {
		_last_status = status;
		_status_pub.publish(status);
		return;
	}

	const float dt = _previous_sample == 0 ? 0.02f : math::constrain((now - _previous_sample) * 1e-6f, 0.002f, 0.1f);
	_previous_sample = now;

	if (!_derivatives_initialized) {
		memcpy(_previous_acceleration, acceleration.xyz, sizeof(_previous_acceleration));
		_previous_velocity[0] = local_position.vx;
		_previous_velocity[1] = local_position.vy;
		_previous_velocity[2] = local_position.vz;
		memcpy(_previous_q, attitude.q, sizeof(_previous_q));
		_derivatives_initialized = true;
	}

	float acceleration_norm_sq = 0.f;
	float jerk_norm_sq = 0.f;
	float angular_rate_norm_sq = 0.f;
	float angular_acceleration_norm_sq = 0.f;
	float rate_error_norm_sq = 0.f;
	const float rate_sp[3] {rates_sp.roll, rates_sp.pitch, rates_sp.yaw};

	for (uint8_t axis = 0; axis < 3; ++axis) {
		acceleration_norm_sq += acceleration.xyz[axis] * acceleration.xyz[axis];
		const float jerk_axis = (acceleration.xyz[axis] - _previous_acceleration[axis]) / dt;
		jerk_norm_sq += jerk_axis * jerk_axis;
		angular_rate_norm_sq += angular_velocity.xyz[axis] * angular_velocity.xyz[axis];
		angular_acceleration_norm_sq += angular_velocity.xyz_derivative[axis] * angular_velocity.xyz_derivative[axis];
		const float rate_error_axis = rate_sp[axis] - angular_velocity.xyz[axis];
		rate_error_norm_sq += rate_error_axis * rate_error_axis;
		_previous_acceleration[axis] = acceleration.xyz[axis];
	}

	status.acceleration_magnitude = sqrtf(acceleration_norm_sq);
	status.jerk = sqrtf(jerk_norm_sq);
	status.angular_rate = sqrtf(angular_rate_norm_sq);
	status.angular_acceleration = sqrtf(angular_acceleration_norm_sq);
	status.rate_error = rates_sp.timestamp != 0 ? sqrtf(rate_error_norm_sq) : 0.f;
	status.attitude_error = attitude_sp.timestamp != 0 ? quaternionError(attitude.q, attitude_sp.q_d) : 0.f;
	const float attitude_jump = quaternionError(attitude.q, _previous_q);
	memcpy(_previous_q, attitude.q, sizeof(_previous_q));
	float velocity_jump = 0.f;

	if (local_position.v_xy_valid && local_position.v_z_valid) {
		const float velocity[3] {local_position.vx, local_position.vy, local_position.vz};

		for (uint8_t axis = 0; axis < 3; ++axis) {
			const float delta = velocity[axis] - _previous_velocity[axis];
			velocity_jump += delta * delta;
			_previous_velocity[axis] = velocity[axis];
		}

		velocity_jump = sqrtf(velocity_jump);
	}

	if (_param_ftc_impact_en.get() && status.valid) {
		const float acceleration_score = status.acceleration_magnitude / fmaxf(_param_ftc_impact_acc.get(), 1.f);
		const float jerk_score = status.jerk / fmaxf(_param_ftc_impact_jrk.get(), 1.f);
		const float rotation_score = status.angular_acceleration / 100.f;
		const float attitude_score = attitude_jump / 0.5f;
		const float velocity_score = velocity_jump / 3.f;
		status.impact_score = math::constrain(0.35f * acceleration_score + 0.30f * jerk_score
				      + 0.15f * rotation_score + 0.10f * attitude_score + 0.10f * velocity_score, 0.f, 1.f);
		status.impact_confidence = math::constrain(fmaxf(acceleration_score, jerk_score) * 0.7f
					   + fmaxf(attitude_score, velocity_score) * 0.3f, 0.f, 1.f);
		status.impact_detected = status.impact_score >= 0.7f && status.impact_confidence >= 0.6f;

		if (status.impact_detected && now - _last_impact > 500_ms) {
			_last_impact = now;
			status.event_timestamp = now;
		}

		status.hard_landing = status.impact_detected && (land.ground_contact || land.maybe_landed)
				      && local_position.v_z_valid && local_position.vz > 1.5f;
		status.impact_type = status.hard_landing ? ftc_extreme_state_s::IMPACT_HARD_LANDING
				     : (status.impact_detected ? ftc_extreme_state_s::IMPACT_EXTERNAL : ftc_extreme_state_s::IMPACT_NONE);
		status.impact_severity = status.impact_detected ? status.impact_score : 0.f;

		if (status.acceleration_magnitude > 1.f) {
			for (uint8_t axis = 0; axis < 3; ++axis) {
				status.impact_axis[axis] = acceleration.xyz[axis] / status.acceleration_magnitude;
			}
		}
	}

	if (_param_ftc_loc_en.get() && status.valid) {
		float score = 0.f;

		if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED || land.landed) {
			_last_status = status;
			_status_pub.publish(status);
			return;
		}

		if (status.attitude_error > 0.6f) {
			status.loss_of_control_reason_mask |= REASON_ATTITUDE_ERROR;
			score += 0.25f;
		}

		if (status.rate_error > 2.f || status.angular_rate > 5.f) {
			status.loss_of_control_reason_mask |= REASON_RATE_ERROR;
			score += 0.25f;
		}

		if (!allocator_status.torque_setpoint_achieved || !allocator_status.thrust_setpoint_achieved) {
			status.loss_of_control_reason_mask |= REASON_SATURATION;
			score += 0.15f;
		}

		for (uint8_t i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			if (PX4_ISFINITE(motors.control[i]) && motors.control[i] > 0.98f) {
				status.loss_of_control_reason_mask |= REASON_SATURATION;
				score = fmaxf(score, 0.15f);
				break;
			}
		}

		if (authority.valid && authority.state >= ftc_control_authority_s::ATTITUDE_DEGRADED) {
			status.loss_of_control_reason_mask |= REASON_AUTHORITY;
			score += 0.20f;
		}

		if (health.failed_mask != 0 || health.degraded_mask != 0) {
			status.loss_of_control_reason_mask |= REASON_PROPULSION;
			score += health.failed_mask != 0 ? 0.25f : 0.10f;
		}

		if (local_position.v_z_valid && local_position.vz > 3.f) {
			status.loss_of_control_reason_mask |= REASON_VERTICAL_MOTION;
			score += 0.15f;
		}

		status.loss_of_control_score = math::constrain(score, 0.f, 1.f);
		const float threshold = _param_ftc_loc_thr.get();

		if (status.loss_of_control_score >= 0.95f || (authority.valid && authority.state == ftc_control_authority_s::UNCONTROLLABLE)) {
			status.loc_state = ftc_extreme_state_s::LOC_UNRECOVERABLE;

		} else if (status.loss_of_control_score >= threshold) {
			status.loc_state = ftc_extreme_state_s::LOC_LOSS_OF_CONTROL;

		} else if (status.loss_of_control_score >= threshold * 0.7f) {
			status.loc_state = ftc_extreme_state_s::LOC_RECOVERY_RECOMMENDED;

		} else if (status.loss_of_control_score >= threshold * 0.35f || status.impact_detected) {
			status.loc_state = ftc_extreme_state_s::LOC_DISTURBED;
		}
	}

	_last_status = status;
	_status_pub.publish(status);
}

int FtcExtremeStateMonitor::print_status()
{
	PX4_INFO("impact: %s score %.2f, LOC state: %u score %.2f", _last_status.impact_detected ? "yes" : "no",
		 (double)_last_status.impact_score, (unsigned)_last_status.loc_state,
		 (double)_last_status.loss_of_control_score);
	return 0;
}

int FtcExtremeStateMonitor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FtcExtremeStateMonitor::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Independent impact, hard-landing and loss-of-control monitor.");
	PRINT_MODULE_USAGE_NAME("ftc_extreme_state_monitor", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ftc_extreme_state_monitor_main(int argc, char *argv[])
{
	return FtcExtremeStateMonitor::main(argc, argv);
}
