/****************************************************************************
 * Copyright (c) 2026 MERIVUS. All rights reserved.
 ****************************************************************************/

#ifndef MERIVUS_FTC_MOTOR_STATUS_HPP
#define MERIVUS_FTC_MOTOR_STATUS_HPP

#include "MerivusFtcTelemetry.hpp"

#include <uORB/topics/ftc_model_status.h>
#include <uORB/topics/ftc_system_status.h>
#include <uORB/topics/motor_health_status.h>

class MavlinkStreamMerivusFtcMotorStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMerivusFtcMotorStatus(mavlink); }
	static constexpr const char *get_name_static() { return "MERIVUS_FTC_MOTOR_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MERIVUS_FTC_MOTOR_STATUS; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _motor_sub.advertised() ? MAVLINK_MSG_ID_MERIVUS_FTC_MOTOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMerivusFtcMotorStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _motor_sub{ORB_ID(motor_health_status)};
	uORB::Subscription _model_sub{ORB_ID(ftc_model_status)};
	uORB::Subscription _system_sub{ORB_ID(ftc_system_status)};
	ftc_model_status_s _model{};
	ftc_system_status_s _system{};

	bool send() override
	{
		motor_health_status_s motor{};

		if (!_motor_sub.update(&motor)) {
			return false;
		}

		_model_sub.copy(&_model);
		_system_sub.copy(&_system);
		mavlink_merivus_ftc_motor_status_t msg{};
		msg.time_usec = motor.timestamp;
		msg.degraded_mask = motor.degraded_mask;
		msg.failed_mask = motor.failed_mask;
		msg.protocol_version = merivus_ftc_telemetry::ProtocolVersion;
		msg.system_state = _system.state;
		msg.monitor_state = motor.state;
		msg.motor_count = motor.motor_count < merivus_ftc_telemetry::MaxMotors ? motor.motor_count : merivus_ftc_telemetry::MaxMotors;
		msg.flags = (_system.monitor_enabled ? MERIVUS_FTC_MOTOR_FLAGS_MONITOR_ENABLED : 0)
			| (motor.model_valid ? MERIVUS_FTC_MOTOR_FLAGS_MODEL_VALID : 0)
			| (motor.esc_data_available ? MERIVUS_FTC_MOTOR_FLAGS_ESC_DATA_AVAILABLE : 0)
			| (motor.imu_only ? MERIVUS_FTC_MOTOR_FLAGS_IMU_ONLY : 0);
		msg.model_quality_pct = merivus_ftc_telemetry::encode_percentage(_model.model_quality, _model.valid);

		for (uint8_t i = 0; i < merivus_ftc_telemetry::MaxMotors; ++i) {
			const bool available = i < msg.motor_count && motor.state == motor_health_status_s::STATE_VALID;
			msg.health_pct[i] = merivus_ftc_telemetry::encode_percentage(motor.health[i], available);
			msg.effectiveness_pct[i] = merivus_ftc_telemetry::encode_percentage(motor.effectiveness[i], i < msg.motor_count && _model.baseline_learned);
			msg.fault_probability_pct[i] = merivus_ftc_telemetry::encode_percentage(motor.fault_probability[i], available);
			msg.confidence_pct[i] = merivus_ftc_telemetry::encode_percentage(motor.confidence[i], i < msg.motor_count);
			msg.fault_type[i] = motor.fault_type[i];
			msg.diagnosis_state[i] = motor.diagnosis_state[i];
			msg.estimate_uncertainty[i] = _model.estimate_uncertainty[i];
		}

		msg.last_valid_timestamp = _model.last_valid_timestamp;
		msg.estimate_age = _model.estimate_age;
		msg.estimator_state = _model.state;
		msg.baseline_learned = _model.baseline_learned;
		msg.current_observable = _model.current_observable;
		mavlink_msg_merivus_ftc_motor_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MERIVUS_FTC_MOTOR_STATUS_HPP
