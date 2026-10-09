/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#include "FtcSupervisor.hpp"

#include "FtcSystemPolicy.hpp"
#include <px4_platform_common/log.h>

FtcSupervisor::FtcSupervisor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

FtcSupervisor::~FtcSupervisor()
{
	ScheduleClear();
}

int FtcSupervisor::task_spawn(int argc, char *argv[])
{
	FtcSupervisor *instance = new FtcSupervisor();

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

bool FtcSupervisor::start()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void FtcSupervisor::Run()
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

	motor_health_status_s health{};
	ftc_model_status_s model{};
	ftc_control_authority_s authority{};
	ftc_extreme_state_s extreme{};
	ftc_recovery_status_s recovery{};
	_health_sub.copy(&health);
	_model_sub.copy(&model);
	_authority_sub.copy(&authority);
	_extreme_sub.copy(&extreme);
	_recovery_sub.copy(&recovery);
	ftc_allocation_status_s allocation{};
	ftc_arbitration_status_s arbitration{};
	_allocation_sub.copy(&allocation);
	_arbitration_sub.copy(&arbitration);
	ftc_allocation_shadow_s shadow{};
	_shadow_sub.copy(&shadow);
	const auto status = FtcSystemPolicy::evaluate(hrt_absolute_time(), _param_ftc_mon_en.get(),
		health, model, authority, extreme, recovery, allocation, arbitration, shadow);
	_last_status = status;
	_status_pub.publish(status);
}

int FtcSupervisor::print_status()
{
	PX4_INFO("state: %u, confidence %.2f, degraded 0x%04x, failed 0x%04x, intervention: %s",
		 (unsigned)_last_status.state, (double)_last_status.system_confidence,
		 (unsigned)_last_status.degraded_motor_mask, (unsigned)_last_status.failed_motor_mask,
		 _last_status.intervention_enabled ? "active" : "inactive");
	PX4_INFO("reason mask: 0x%08lx, model: %s, authority: %s, recovery eligible: %s",
		 (unsigned long)_last_status.reason_mask, _last_status.model_valid ? "valid" : "invalid",
		 _last_status.control_authority_valid ? "valid" : "invalid",
		 _last_status.recovery_eligible ? "yes" : "no");
	return 0;
}

int FtcSupervisor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FtcSupervisor::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Read-only FTC subsystem state and confidence supervisor.");
	PRINT_MODULE_USAGE_NAME("ftc_supervisor", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ftc_supervisor_main(int argc, char *argv[])
{
	return FtcSupervisor::main(argc, argv);
}
