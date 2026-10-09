/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <ControlAllocationSequentialDesaturation.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/ftc_allocation_shadow.h>
#include <uORB/topics/ftc_allocation_status.h>
#include <uORB/topics/ftc_control_authority.h>
#include <uORB/topics/ftc_effectiveness_matrix.h>
#include <uORB/topics/motor_health_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class FtcControlMonitor : public ModuleBase<FtcControlMonitor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FtcControlMonitor();
	~FtcControlMonitor() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool start();

private:
	void Run() override;
	bool updateMatrix();
	void publishInvalid(hrt_abstime now);
	void calculateShadow(hrt_abstime now);
	float axisAuthority(uint8_t axis) const;

	uORB::Subscription _allocation_sub{ORB_ID(ftc_allocation_status)};
	uORB::Subscription _matrix_sub{ORB_ID(ftc_effectiveness_matrix)};
	uORB::Subscription _health_sub{ORB_ID(motor_health_status)};
	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<ftc_allocation_shadow_s> _shadow_pub{ORB_ID(ftc_allocation_shadow)};
	uORB::Publication<ftc_control_authority_s> _authority_pub{ORB_ID(ftc_control_authority)};

	ControlAllocationSequentialDesaturation _allocator{};
	ftc_effectiveness_matrix_s _matrix{};
	motor_health_status_s _health{};
	actuator_motors_s _motors{};
	vehicle_torque_setpoint_s _torque{};
	vehicle_thrust_setpoint_s _thrust{};
	matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> _nominal_matrix{};
	matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> _dynamic_matrix{};
	ftc_control_authority_s _last_authority{};
	bool _matrix_valid{false};
	bool _normalization_initialized{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTC_CA_SHADOW>) _param_ftc_ca_shadow,
		(ParamBool<px4::params::FTC_CA_EN>) _param_ftc_ca_en,
		(ParamFloat<px4::params::FTC_CA_ATT_MIN>) _param_ftc_ca_att_min,
		(ParamFloat<px4::params::FTC_CA_YAW_MIN>) _param_ftc_ca_yaw_min,
		(ParamFloat<px4::params::FTC_CA_THR_MIN>) _param_ftc_ca_thr_min
	)
};
