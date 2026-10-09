/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#pragma once
#include <uORB/topics/ftc_allocation_status.h>
#include <uORB/topics/ftc_allocation_shadow.h>
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
#include <uORB/topics/ftc_model_status.h>
#include <uORB/topics/ftc_recovery_status.h>
#include <uORB/topics/ftc_system_status.h>
#include <uORB/topics/motor_health_status.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class FtcSupervisor : public ModuleBase<FtcSupervisor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FtcSupervisor();
	~FtcSupervisor() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool start();

private:
	void Run() override;

	uORB::Subscription _shadow_sub{ORB_ID(ftc_allocation_shadow)};
	uORB::Subscription _allocation_sub{ORB_ID(ftc_allocation_status)};
	uORB::Subscription _arbitration_sub{ORB_ID(ftc_arbitration_status)};
	uORB::Subscription _health_sub{ORB_ID(motor_health_status)};
	uORB::Subscription _model_sub{ORB_ID(ftc_model_status)};
	uORB::Subscription _authority_sub{ORB_ID(ftc_control_authority)};
	uORB::Subscription _extreme_sub{ORB_ID(ftc_extreme_state)};
	uORB::Subscription _recovery_sub{ORB_ID(ftc_recovery_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<ftc_system_status_s> _status_pub{ORB_ID(ftc_system_status)};
	ftc_system_status_s _last_status{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTC_MON_EN>) _param_ftc_mon_en,
		(ParamBool<px4::params::FTC_REC_ACT>) _param_ftc_rec_act
	)
};
