/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#include "FtcControlMonitor.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

#include <math.h>

FtcControlMonitor::FtcControlMonitor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

FtcControlMonitor::~FtcControlMonitor()
{
	ScheduleClear();
}

int FtcControlMonitor::task_spawn(int argc, char *argv[])
{
	FtcControlMonitor *instance = new FtcControlMonitor();

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

bool FtcControlMonitor::start()
{
	ScheduleOnInterval(20_ms);
	return true;
}

bool FtcControlMonitor::updateMatrix()
{
	ftc_effectiveness_matrix_s matrix{};

	if (!_matrix_sub.update(&matrix)) {
		return _matrix_valid;
	}

	_matrix = matrix;
	_normalization_initialized = false;
	_matrix_valid = matrix.valid && matrix.matrix_index == 0 && matrix.num_axes == ControlAllocation::NUM_AXES
			&& matrix.num_actuators > 0 && matrix.num_actuators <= ControlAllocation::NUM_ACTUATORS;

	if (!_matrix_valid) {
		return false;
	}

	for (uint8_t axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
		for (uint8_t actuator = 0; actuator < ControlAllocation::NUM_ACTUATORS; ++actuator) {
			_nominal_matrix(axis, actuator) = actuator < matrix.num_actuators
						  ? matrix.effectiveness[axis * ftc_effectiveness_matrix_s::NUM_ACTUATORS + actuator] : 0.f;
		}
	}

	return true;
}

float FtcControlMonitor::axisAuthority(uint8_t axis) const
{
	float nominal = 0.f;
	float dynamic = 0.f;

	for (uint8_t actuator = 0; actuator < _matrix.num_actuators; ++actuator) {
		const float range = fmaxf(_matrix.maximum[actuator] - _matrix.minimum[actuator], 0.f);
		nominal += fabsf(_nominal_matrix(axis, actuator)) * range;
		dynamic += fabsf(_dynamic_matrix(axis, actuator)) * range;
	}

	return nominal > 1e-5f ? math::constrain(dynamic / nominal, 0.f, 1.f) : 0.f;
}

void FtcControlMonitor::publishInvalid(hrt_abstime now)
{
	if (now - _last_authority.timestamp < 1_s) {
		return;
	}

	_last_authority = {};
	_last_authority.timestamp = now;
	_last_authority.state = ftc_control_authority_s::UNCONTROLLABLE;
	_authority_pub.publish(_last_authority);
}

void FtcControlMonitor::calculateShadow(hrt_abstime now)
{
	using ActuatorVector = ControlAllocation::ActuatorVector;
	using ControlVector = matrix::Vector<float, ControlAllocation::NUM_AXES>;
	ActuatorVector minimum{};
	ActuatorVector maximum{};
	ActuatorVector trim{};
	ActuatorVector linearization_point{};
	ActuatorVector current{};

	_dynamic_matrix = _nominal_matrix;

	for (uint8_t actuator = 0; actuator < _matrix.num_actuators; ++actuator) {
		minimum(actuator) = _matrix.minimum[actuator];
		maximum(actuator) = _matrix.maximum[actuator];
		trim(actuator) = _matrix.trim[actuator];
		linearization_point(actuator) = _matrix.linearization_point[actuator];
		current(actuator) = actuator < actuator_motors_s::NUM_CONTROLS && PX4_ISFINITE(_motors.control[actuator])
				    ? _motors.control[actuator] : trim(actuator);
		const float effectiveness = actuator < _matrix.num_motors && actuator < _health.motor_count
					    && PX4_ISFINITE(_health.effectiveness[actuator])
					    ? math::constrain(_health.effectiveness[actuator], 0.05f, 1.f) : 1.f;

		for (uint8_t axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
			_dynamic_matrix(axis, actuator) *= effectiveness;
		}
	}

	_allocator.setActuatorMin(minimum);
	_allocator.setActuatorMax(maximum);
	_allocator.setNormalizeRPY(_matrix.normalize_rpy);

	if (!_normalization_initialized) {
		ControlVector zero_control{};
		_allocator.setEffectivenessMatrix(_nominal_matrix, trim, linearization_point, _matrix.num_actuators, true);
		_allocator.setControlSetpoint(zero_control);
		_allocator.allocate();
		_normalization_initialized = true;
	}

	_allocator.setEffectivenessMatrix(_dynamic_matrix, trim, linearization_point, _matrix.num_actuators, false);
	_allocator.setActuatorSetpoint(current);
	ControlVector control_sp{};

	for (uint8_t axis = 0; axis < 3; ++axis) {
		control_sp(axis) = _torque.xyz[axis];
		control_sp(axis + 3) = _thrust.xyz[axis];
	}

	_allocator.setControlSetpoint(control_sp);
	_allocator.allocate();
	const ActuatorVector &candidate = _allocator.getActuatorSetpoint();
	const ControlVector residual = control_sp - _allocator.getAllocatedControl();
	ftc_allocation_shadow_s shadow{};
	shadow.timestamp = now;
	shadow.timestamp_sample = _motors.timestamp_sample;
	shadow.motor_count = _matrix.num_motors;
	shadow.valid = _health.model_valid;
	float residual_norm_sq = 0.f;

	for (uint8_t axis = 0; axis < 3; ++axis) {
		shadow.residual_torque[axis] = residual(axis);
		shadow.residual_thrust[axis] = residual(axis + 3);
		residual_norm_sq += residual(axis) * residual(axis) + residual(axis + 3) * residual(axis + 3);
	}

	shadow.residual_norm = sqrtf(residual_norm_sq);

	for (uint8_t i = 0; i < ftc_allocation_shadow_s::NUM_MOTORS; ++i) {
		shadow.nominal[i] = i < shadow.motor_count ? current(i) : NAN;
		shadow.candidate[i] = i < shadow.motor_count ? candidate(i) : NAN;
		shadow.effectiveness[i] = i < _health.motor_count ? _health.effectiveness[i] : NAN;

		if (i < shadow.motor_count && (candidate(i) <= minimum(i) + 0.01f || candidate(i) >= maximum(i) - 0.01f)) {
			shadow.saturated_mask |= 1u << i;
		}
	}

	ftc_control_authority_s authority{};
	authority.timestamp = now;
	authority.valid = _health.model_valid;
	authority.matrix_valid = _matrix_valid;
	authority.motor_count = _matrix.num_motors;
	authority.saturated_mask = shadow.saturated_mask;
	authority.roll_authority = axisAuthority(ControlAllocation::ROLL);
	authority.pitch_authority = axisAuthority(ControlAllocation::PITCH);
	authority.yaw_authority = axisAuthority(ControlAllocation::YAW);
	authority.thrust_authority = axisAuthority(ControlAllocation::THRUST_Z);
	authority.minimum_attitude_authority = fminf(authority.roll_authority, authority.pitch_authority);
	float headroom_sum = 0.f;

	for (uint8_t i = 0; i < _matrix.num_motors; ++i) {
		const float range = fmaxf(maximum(i) - minimum(i), 0.01f);
		headroom_sum += fminf(maximum(i) - current(i), current(i) - minimum(i)) / range * 2.f;
	}

	authority.actuator_headroom = _matrix.num_motors > 0
				      ? math::constrain(headroom_sum / _matrix.num_motors, 0.f, 1.f) : 0.f;
	authority.state = ftc_control_authority_s::FULL_CONTROL;

	if (authority.thrust_authority < _param_ftc_ca_thr_min.get()) {
		authority.state = ftc_control_authority_s::THRUST_INSUFFICIENT;

	} else if (authority.minimum_attitude_authority < 0.1f) {
		authority.state = ftc_control_authority_s::UNCONTROLLABLE;

	} else if (authority.minimum_attitude_authority < _param_ftc_ca_att_min.get() * 0.5f) {
		authority.state = ftc_control_authority_s::ATTITUDE_DEGRADED;

	} else if (authority.minimum_attitude_authority < _param_ftc_ca_att_min.get()) {
		authority.state = ftc_control_authority_s::RECOVERY_ONLY;

	} else if (authority.yaw_authority < _param_ftc_ca_yaw_min.get()) {
		authority.state = ftc_control_authority_s::YAW_UNCONTROLLABLE;
		shadow.yaw_sacrificed = true;

	} else if (authority.saturated_mask != 0 || authority.minimum_attitude_authority < 0.8f) {
		authority.state = ftc_control_authority_s::DEGRADED_CONTROL;
	}

	for (uint8_t axis = 0; axis < 6; ++axis) {
		float positive = 0.f, negative = 0.f;
		for (uint8_t i = 0; i < _matrix.num_motors; ++i) {
			const float up = fmaxf(maximum(i) - current(i), 0.f);
			const float down = fmaxf(current(i) - minimum(i), 0.f);
			const float b = _dynamic_matrix(axis, i);
			positive += b >= 0.f ? b * up : -b * down;
			negative += b >= 0.f ? b * down : -b * up;
		}
		if (axis < 3) { authority.positive_authority[axis] = positive; authority.negative_authority[axis] = negative; }
		if (axis == 5) { authority.thrust_up = negative; authority.thrust_down = positive; }
	}
	authority.reachable_residual = shadow.residual_norm;
	_last_authority = authority;
	_shadow_pub.publish(shadow);
	_authority_pub.publish(authority);
}

void FtcControlMonitor::Run()
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
	updateMatrix();

	if ((!_param_ftc_ca_shadow.get() && !_param_ftc_ca_en.get()) || !_matrix_valid) {
		publishInvalid(now);
		return;
	}

	_health_sub.copy(&_health);
	_motors_sub.copy(&_motors);
	_torque_sub.copy(&_torque);
	_thrust_sub.copy(&_thrust);
	const bool inputs_fresh = _health.timestamp != 0 && now - _health.timestamp < 500_ms
				  && _motors.timestamp != 0 && now - _motors.timestamp < 200_ms
				  && _torque.timestamp != 0 && now - _torque.timestamp < 200_ms
				  && _thrust.timestamp != 0 && now - _thrust.timestamp < 200_ms;

	if (inputs_fresh) {
		calculateShadow(now);

	} else {
		publishInvalid(now);
	}
}

int FtcControlMonitor::print_status()
{
	ftc_allocation_status_s allocation{};
	_allocation_sub.copy(&allocation);
	PX4_INFO("actual allocation: active %u state %u fallback 0x%lx age %.2f s", allocation.active,
		(unsigned)allocation.state, (unsigned long)allocation.fallback_reason,
		allocation.timestamp ? (double)((hrt_absolute_time() - allocation.timestamp) * 1e-6f) : -1.);
	PX4_INFO("shadow: %s, takeover interface: %s, matrix: %s, authority state: %u",
		 _param_ftc_ca_shadow.get() ? "enabled" : "disabled", _param_ftc_ca_en.get() ? "enabled/gated" : "disabled",
		 _matrix_valid ? "valid" : "invalid", (unsigned)_last_authority.state);
	PX4_INFO("authority R/P/Y/T %.2f/%.2f/%.2f/%.2f, headroom %.2f, saturation 0x%04x",
		 (double)_last_authority.roll_authority, (double)_last_authority.pitch_authority,
		 (double)_last_authority.yaw_authority, (double)_last_authority.thrust_authority,
		 (double)_last_authority.actuator_headroom, (unsigned)_last_authority.saturated_mask);
	return 0;
}

int FtcControlMonitor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FtcControlMonitor::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Dynamic effectiveness-matrix allocation shadow and control authority monitor. No actuator output.");
	PRINT_MODULE_USAGE_NAME("ftc_control_monitor", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ftc_control_monitor_main(int argc, char *argv[])
{
	return FtcControlMonitor::main(argc, argv);
}
