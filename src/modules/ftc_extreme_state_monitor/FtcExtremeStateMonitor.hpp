/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/ftc_control_authority.h>
#include <uORB/topics/ftc_extreme_state.h>
#include <uORB/topics/motor_health_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class FtcExtremeStateMonitor : public ModuleBase<FtcExtremeStateMonitor>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FtcExtremeStateMonitor();
	~FtcExtremeStateMonitor() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool start();

private:
	void Run() override;
	float quaternionError(const float q[4], const float q_d[4]) const;

	uORB::Subscription _acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _attitude_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _authority_sub{ORB_ID(ftc_control_authority)};
	uORB::Subscription _health_sub{ORB_ID(motor_health_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<ftc_extreme_state_s> _status_pub{ORB_ID(ftc_extreme_state)};

	float _previous_acceleration[3] {};
	float _previous_velocity[3] {};
	float _previous_q[4] {1.f, 0.f, 0.f, 0.f};
	hrt_abstime _previous_sample{0};
	hrt_abstime _last_impact{0};
	bool _derivatives_initialized{false};
	ftc_extreme_state_s _last_status{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTC_IMPACT_EN>) _param_ftc_impact_en,
		(ParamFloat<px4::params::FTC_IMPACT_ACC>) _param_ftc_impact_acc,
		(ParamFloat<px4::params::FTC_IMPACT_JRK>) _param_ftc_impact_jrk,
		(ParamBool<px4::params::FTC_LOC_EN>) _param_ftc_loc_en,
		(ParamFloat<px4::params::FTC_LOC_THR>) _param_ftc_loc_thr
	)
};
