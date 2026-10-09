/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#pragma once
#include "FtcRecoveryController.hpp"
#include <uORB/topics/ftc_arbitration_status.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/ftc_control_authority.h>
#include <uORB/topics/ftc_extreme_state.h>
#include <uORB/topics/ftc_recovery_status.h>
#include <uORB/topics/motor_health_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class FtcRecovery : public ModuleBase<FtcRecovery>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FtcRecovery();
	~FtcRecovery() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool start();

private:
	void Run() override;
	FtcRecoveryController _controller{};
	hrt_abstime _last_run{0};
	uORB::Subscription _arbitration_sub{ORB_ID(ftc_arbitration_status)};

	uORB::Subscription _extreme_sub{ORB_ID(ftc_extreme_state)};
	uORB::Subscription _authority_sub{ORB_ID(ftc_control_authority)};
	uORB::Subscription _health_sub{ORB_ID(motor_health_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<ftc_recovery_status_s> _status_pub{ORB_ID(ftc_recovery_status)};

	ftc_recovery_status_s _last_status{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTC_MON_EN>) _param_ftc_mon_en,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamBool<px4::params::FTC_REC_EN>) _param_ftc_rec_en,
		(ParamBool<px4::params::FTC_REC_ACT>) _param_ftc_rec_act,
		(ParamFloat<px4::params::FTC_REC_RATE>) _param_ftc_rec_rate,
		(ParamFloat<px4::params::FTC_REC_KD>) _param_ftc_rec_kd,
		(ParamFloat<px4::params::FTC_REC_ALT>) _param_ftc_rec_alt
	)
};
