/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "MotorEffectivenessEstimator.hpp"
#include "CommandAlignment.hpp"
#include "RigidBodyObserver.hpp"
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/ftc_effectiveness_matrix.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/ftc_model_status.h>
#include <uORB/topics/motor_health_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class MotorHealthMonitor : public ModuleBase<MotorHealthMonitor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MotorHealthMonitor();
	~MotorHealthMonitor() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool start();

private:
	void Run() override;
	void resetEstimator();
	void publishDisabled(hrt_abstime now);
	uint8_t motorCount(const actuator_motors_s &actuator_motors) const;
	void classifyFaults(hrt_abstime now, const actuator_motors_s &actuator_motors,
			    motor_health_status_s &status, float dt);
	void publishModelStatus(hrt_abstime now, const vehicle_angular_velocity_s &angular_velocity,
				const motor_health_status_s &health_status);

	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<motor_health_status_s> _motor_health_status_pub{ORB_ID(motor_health_status)};
	uORB::Publication<ftc_model_status_s> _model_status_pub{ORB_ID(ftc_model_status)};

	MotorEffectivenessEstimator _estimator{};
	CommandAlignment _alignment{};
	RigidBodyObserver _rigid_body{};
	MotorEffectivenessEstimator::Input _estimator_input{};
	ftc_effectiveness_matrix_s _matrix{};
	uORB::Subscription _matrix_sub{ORB_ID(ftc_effectiveness_matrix)};
	uORB::Subscription _allocator_sub{ORB_ID(control_allocator_status)};
	motor_health_status_s _last_status{};
	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	esc_status_s _esc_status{};
	hrt_abstime _degraded_start[MotorEffectivenessEstimator::MAX_MOTORS] {};
	hrt_abstime _failed_start[MotorEffectivenessEstimator::MAX_MOTORS] {};
	float _fault_probability_lpf[MotorEffectivenessEstimator::MAX_MOTORS] {};
	float _effectiveness_change_lpf[MotorEffectivenessEstimator::MAX_MOTORS] {};
	float _previous_effectiveness[MotorEffectivenessEstimator::MAX_MOTORS] {};
	float _filtered_acceleration_magnitude{0.f};
	bool _acceleration_filter_initialized{false};
	hrt_abstime _last_run{0}, _last_response_sample{0};
	hrt_abstime _last_disabled_publish{0};
	bool _was_armed{false};
	bool _was_landed{true};
	bool _was_enabled{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTC_MON_EN>) _param_ftc_mon_en,
		(ParamFloat<px4::params::FTC_EST_DELAY>) _param_ftc_est_delay,
		(ParamFloat<px4::params::FTC_EST_AGE>) _param_ftc_est_age,
		(ParamFloat<px4::params::FTC_THR_MAX>) _param_ftc_thr_max,
		(ParamFloat<px4::params::FTC_MIN_THR>) _param_ftc_min_thr,
		(ParamFloat<px4::params::FTC_LPF_TC>) _param_ftc_lpf_tc,
		(ParamFloat<px4::params::FTC_EXC_MIN>) _param_ftc_exc_min,
		(ParamFloat<px4::params::FTC_BASE_T>) _param_ftc_base_t,
		(ParamFloat<px4::params::FTC_EST_RATE>) _param_ftc_est_rate,
		(ParamFloat<px4::params::FTC_EST_FORG>) _param_ftc_est_forg,
		(ParamFloat<px4::params::FTC_EST_LMIN>) _param_ftc_est_lmin,
		(ParamFloat<px4::params::FTC_EST_IXX>) _param_ftc_est_ixx,
		(ParamFloat<px4::params::FTC_EST_IYY>) _param_ftc_est_iyy,
		(ParamFloat<px4::params::FTC_EST_IZZ>) _param_ftc_est_izz,
		(ParamFloat<px4::params::FTC_RES_THR>) _param_ftc_res_thr,
		(ParamFloat<px4::params::FTC_HLTH_MIN>) _param_ftc_hlth_min,
		(ParamFloat<px4::params::FTC_FAIL_MIN>) _param_ftc_fail_min,
		(ParamFloat<px4::params::FTC_CONF_MIN>) _param_ftc_conf_min,
		(ParamFloat<px4::params::FTC_FAIL_T>) _param_ftc_fail_t,
		(ParamFloat<px4::params::FTC_FAULT_P>) _param_ftc_fault_p,
		(ParamFloat<px4::params::FTC_FAULT_VIB>) _param_ftc_fault_vib,
		(ParamFloat<px4::params::FTC_FAULT_EXT>) _param_ftc_fault_ext
	)
};
