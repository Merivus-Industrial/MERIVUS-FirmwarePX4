/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#include "FtcRecovery.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

#include <math.h>

namespace
{
constexpr uint32_t TRIGGER_IMPACT = 1u << 0;
constexpr uint32_t TRIGGER_LOSS_OF_CONTROL = 1u << 1;
constexpr uint32_t TRIGGER_MOTOR_FAULT = 1u << 2;
constexpr uint32_t TRIGGER_AUTHORITY = 1u << 3;
constexpr uint32_t INHIBIT_NOT_ARMED = 1u << 0;
constexpr uint32_t INHIBIT_LANDED = 1u << 1;
constexpr uint32_t INHIBIT_LOW_ALTITUDE = 1u << 2;
constexpr uint32_t INHIBIT_STALE_STATE = 1u << 3;
constexpr uint32_t INHIBIT_VEHICLE_TYPE = 1u << 5;
constexpr uint32_t INHIBIT_AUTHORITY = 1u << 6;
}

FtcRecovery::FtcRecovery() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

FtcRecovery::~FtcRecovery()
{
	ScheduleClear();
}

int FtcRecovery::task_spawn(int argc, char *argv[])
{
	FtcRecovery *instance = new FtcRecovery();

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

bool FtcRecovery::start()
{
	ScheduleOnInterval(20_ms);
	return true;
}

void FtcRecovery::Run()
{
	if (should_exit()) { ScheduleClear(); exit_and_cleanup(); return; }
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}
	const hrt_abstime now = hrt_absolute_time();
	ftc_extreme_state_s extreme{};
	ftc_control_authority_s authority{};
	motor_health_status_s health{};
	vehicle_attitude_s attitude{};
	vehicle_angular_velocity_s velocity{};
	vehicle_rates_setpoint_s normal{};
	vehicle_local_position_s position{};
	vehicle_land_detected_s land{};
	vehicle_status_s vehicle{};
	ftc_arbitration_status_s arbitration{};
	_extreme_sub.copy(&extreme); _authority_sub.copy(&authority); _health_sub.copy(&health);
	_attitude_sub.copy(&attitude); _angular_velocity_sub.copy(&velocity); _rates_sp_sub.copy(&normal);
	_local_position_sub.copy(&position); _land_sub.copy(&land); _vehicle_status_sub.copy(&vehicle);
	_arbitration_sub.copy(&arbitration);
	auto fresh = [now](uint64_t timestamp, uint64_t timeout) { return timestamp && now >= timestamp && now - timestamp < timeout; };
	FtcRecoveryController::Input input{};
	input.now = now;
	input.enabled = _param_ftc_mon_en.get() && _param_ftc_rec_en.get();
	input.armed = vehicle.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	input.landed = land.landed;
	input.fresh = fresh(vehicle.timestamp, 1_s) && fresh(land.timestamp, 1_s)
		&& fresh(attitude.timestamp, 200_ms) && fresh(velocity.timestamp, 200_ms) && fresh(normal.timestamp, 200_ms);
	input.failsafe = vehicle.failsafe;
	input.mode = vehicle.nav_state;
	input.mode_allowed = !vehicle.is_vtol && (vehicle.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
		|| vehicle.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL
		|| vehicle.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
		|| vehicle.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	input.controllable = fresh(authority.timestamp, 200_ms) && authority.valid && authority.matrix_valid
		&& authority.minimum_attitude_authority >= 0.35f && authority.thrust_authority >= 0.25f;
	input.vertical_valid = fresh(position.timestamp, 200_ms) && position.z_valid && position.v_z_valid
		&& PX4_ISFINITE(position.z) && PX4_ISFINITE(position.vz);
	input.position_valid = input.vertical_valid && position.xy_valid;
	input.z = position.z; input.vz = position.vz;
	input.position_timestamp = position.timestamp;
	input.z_reset_counter = position.z_reset_counter;
	input.vz_reset_counter = position.vz_reset_counter;
	input.hover_thrust = _param_mpc_thr_hover.get();
	memcpy(input.q, attitude.q, sizeof(input.q));
	memcpy(input.rates, velocity.xyz, sizeof(input.rates));
	input.normal_rates[0] = normal.roll; input.normal_rates[1] = normal.pitch; input.normal_rates[2] = normal.yaw;
	input.normal_thrust = normal.thrust_body[2];
	input.damping = _param_ftc_rec_kd.get(); input.max_rate = _param_ftc_rec_rate.get();
	input.arbitration_weight = fresh(arbitration.timestamp, 200_ms) ? arbitration.weight : 0.f;
	if (fresh(extreme.timestamp, 200_ms) && extreme.valid) {
		if (extreme.impact_detected) { input.trigger |= TRIGGER_IMPACT; }
		if (extreme.loc_state >= ftc_extreme_state_s::LOC_RECOVERY_RECOMMENDED) { input.trigger |= TRIGGER_LOSS_OF_CONTROL; }
	}
	if (fresh(health.timestamp, 200_ms) && health.failed_mask) { input.trigger |= TRIGGER_MOTOR_FAULT; }
	if (fresh(authority.timestamp, 200_ms) && authority.valid && authority.state >= ftc_control_authority_s::ATTITUDE_DEGRADED) {
		input.trigger |= TRIGGER_AUTHORITY;
	}
	ftc_recovery_status_s status{};
	status.timestamp = now;
	status.trigger_mask = input.trigger;
	if (!input.armed) { status.inhibit_reason_mask |= INHIBIT_NOT_ARMED; }
	if (input.landed) { status.inhibit_reason_mask |= INHIBIT_LANDED; }
	if (!input.fresh) { status.inhibit_reason_mask |= INHIBIT_STALE_STATE; }
	if (!input.controllable) { status.inhibit_reason_mask |= INHIBIT_AUTHORITY; }
	if (!input.mode_allowed || vehicle.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) { status.inhibit_reason_mask |= INHIBIT_VEHICLE_TYPE; }
	const auto previous_state = _controller.output().state;
	if (!FtcRecoveryController::entryAltitudeAllowed(previous_state, fresh(position.timestamp, 200_ms),
			position.dist_bottom_valid, position.dist_bottom, position.z_valid, position.z, _param_ftc_rec_alt.get())) {
		status.inhibit_reason_mask |= INHIBIT_LOW_ALTITUDE;
	}
	input.eligible = status.inhibit_reason_mask == 0 && !input.failsafe;
	const float dt = _last_run ? math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.1f) : 0.02f;
	_last_run = now;
	_controller.update(dt, input);
	const auto &candidate = _controller.output();
	status.state = candidate.state;
	status.original_nav_state = candidate.original_mode;
	status.eligible = input.eligible;
	status.candidate_valid = candidate.candidate_valid;
	status.intervention_enabled = input.enabled && _param_ftc_rec_act.get();
	status.active = fresh(arbitration.timestamp, 200_ms) && arbitration.active;
	status.state_elapsed = candidate.elapsed;
	status.progress = candidate.state == FtcRecoveryController::CONTROL_REENTRY ? 1.f - candidate.reentry_weight : 0.f;
	status.reentry_weight = candidate.reentry_weight;
	status.reentry_ready = candidate.reentry_ready;
	status.vertical_speed_setpoint = candidate.vertical_speed;
	status.altitude_reference = candidate.altitude_reference;
	status.fallback_reason = candidate.fallback_reason;
	memcpy(status.body_rate_setpoint, candidate.rate, sizeof(status.body_rate_setpoint));
	memcpy(status.attitude_setpoint_q, candidate.q, sizeof(status.attitude_setpoint_q));
	status.thrust_body[2] = candidate.thrust;
	status.yaw_sacrifice_requested = candidate.candidate_valid;
	_last_status = status;
	_status_pub.publish(status);
}

int FtcRecovery::print_status()
{
	PX4_INFO("state: %u (%.2f s), eligible: %s, trigger: 0x%08lx, inhibit: 0x%08lx",
		 (unsigned)_last_status.state, (double)_last_status.state_elapsed, _last_status.eligible ? "yes" : "no",
		 (unsigned long)_last_status.trigger_mask,
		 (unsigned long)_last_status.inhibit_reason_mask);
	PX4_INFO("candidate %u active %u reentry %.2f fallback %lu", (unsigned)_last_status.candidate_valid,
		(unsigned)_last_status.active, (double)_last_status.reentry_weight, (unsigned long)_last_status.fallback_reason);
	return 0;
}

int FtcRecovery::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FtcRecovery::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Gated recovery candidates with exclusive rate-controller input arbitration. Active defaults off.");
	PRINT_MODULE_USAGE_NAME("ftc_recovery", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ftc_recovery_main(int argc, char *argv[])
{
	return FtcRecovery::main(argc, argv);
}
