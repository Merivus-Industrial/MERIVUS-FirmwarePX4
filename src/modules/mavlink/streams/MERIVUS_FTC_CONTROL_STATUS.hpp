/****************************************************************************
 * Copyright (c) 2026 MERIVUS. All rights reserved.
 ****************************************************************************/

#ifndef MERIVUS_FTC_CONTROL_STATUS_HPP
#define MERIVUS_FTC_CONTROL_STATUS_HPP

#include "MerivusFtcTelemetry.hpp"
#include <uORB/topics/ftc_arbitration_status.h>

#include <uORB/topics/ftc_allocation_shadow.h>
#include <uORB/topics/ftc_control_authority.h>
#include <uORB/topics/ftc_effectiveness_matrix.h>
#include <uORB/topics/ftc_recovery_status.h>
#include <uORB/topics/ftc_system_status.h>

class MavlinkStreamMerivusFtcControlStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMerivusFtcControlStatus(mavlink); }
	static constexpr const char *get_name_static() { return "MERIVUS_FTC_CONTROL_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MERIVUS_FTC_CONTROL_STATUS; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		const bool available = _authority_sub.advertised() || _system_sub.advertised();
		return available ? MAVLINK_MSG_ID_MERIVUS_FTC_CONTROL_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMerivusFtcControlStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _authority_sub{ORB_ID(ftc_control_authority)};
	uORB::Subscription _matrix_sub{ORB_ID(ftc_effectiveness_matrix)};
	uORB::Subscription _shadow_sub{ORB_ID(ftc_allocation_shadow)};
	uORB::Subscription _recovery_sub{ORB_ID(ftc_recovery_status)};
	uORB::Subscription _arbitration_sub{ORB_ID(ftc_arbitration_status)};
	uORB::Subscription _system_sub{ORB_ID(ftc_system_status)};
	ftc_control_authority_s _authority{};
	ftc_effectiveness_matrix_s _matrix{};
	ftc_allocation_shadow_s _shadow{};
	ftc_recovery_status_s _recovery{};
	ftc_system_status_s _system{};

	bool send() override
	{
		bool updated = _authority_sub.update(&_authority);
		updated |= _matrix_sub.update(&_matrix);
		updated |= _shadow_sub.update(&_shadow);
		updated |= _recovery_sub.update(&_recovery);
		updated |= _system_sub.update(&_system);

		if (!updated) {
			return false;
		}

		mavlink_merivus_ftc_control_status_t msg{};
		msg.time_usec = _system.timestamp > 0 ? _system.timestamp : _authority.timestamp;
		msg.saturated_mask = _authority.saturated_mask | _shadow.saturated_mask;
		msg.protocol_version = merivus_ftc_telemetry::ProtocolVersion;
		msg.system_state = _system.state;
		msg.authority_state = _authority.state;
		msg.recovery_state = _recovery.state;
		msg.control_mode = _system.mode;
		msg.flags = (_system.monitor_enabled ? MERIVUS_FTC_CONTROL_FLAGS_MONITOR_ENABLED : 0)
			| (_authority.valid ? MERIVUS_FTC_CONTROL_FLAGS_AUTHORITY_VALID : 0)
			| (_matrix.valid ? MERIVUS_FTC_CONTROL_FLAGS_MATRIX_VALID : 0)
			| (_shadow.valid ? MERIVUS_FTC_CONTROL_FLAGS_SHADOW_VALID : 0)
			| (_shadow.yaw_sacrificed ? MERIVUS_FTC_CONTROL_FLAGS_YAW_SACRIFICED : 0)
			| (_recovery.eligible ? MERIVUS_FTC_CONTROL_FLAGS_RECOVERY_ELIGIBLE : 0)
			| (_recovery.candidate_valid ? MERIVUS_FTC_CONTROL_FLAGS_RECOVERY_CANDIDATE_VALID : 0);
		msg.roll_authority_pct = merivus_ftc_telemetry::encode_percentage(_authority.roll_authority, _authority.valid);
		msg.pitch_authority_pct = merivus_ftc_telemetry::encode_percentage(_authority.pitch_authority, _authority.valid);
		msg.yaw_authority_pct = merivus_ftc_telemetry::encode_percentage(_authority.yaw_authority, _authority.valid);
		msg.thrust_authority_pct = merivus_ftc_telemetry::encode_percentage(_authority.thrust_authority, _authority.valid);
		msg.minimum_attitude_authority_pct = merivus_ftc_telemetry::encode_percentage(_authority.minimum_attitude_authority, _authority.valid);
		msg.actuator_headroom_pct = merivus_ftc_telemetry::encode_percentage(_authority.actuator_headroom, _authority.valid);
		msg.system_confidence_pct = merivus_ftc_telemetry::encode_percentage(_system.system_confidence, _system.timestamp > 0);
		msg.recovery_progress_pct = merivus_ftc_telemetry::encode_percentage(_recovery.progress, _recovery.candidate_valid);

		ftc_arbitration_status_s arbitration{};
		_arbitration_sub.copy(&arbitration);
		memcpy(msg.positive_authority, _authority.positive_authority, sizeof(msg.positive_authority));
		memcpy(msg.negative_authority, _authority.negative_authority, sizeof(msg.negative_authority));
		msg.thrust_up = _authority.thrust_up;
		msg.thrust_down = _authority.thrust_down;
		msg.reachable_residual = _authority.reachable_residual;
		msg.allocation_fallback = _system.allocation_fallback;
		msg.recovery_fallback = _system.recovery_fallback;
		msg.allocation_active = _system.allocation_active;
		msg.recovery_active = _system.recovery_active;
		msg.arbitration_weight = arbitration.weight;
		msg.reentry_weight = _recovery.reentry_weight;
		if (_system.intervention_enabled) { msg.flags |= MERIVUS_FTC_CONTROL_FLAGS_ACTIVE_COMMAND_PATH; }
		mavlink_msg_merivus_ftc_control_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MERIVUS_FTC_CONTROL_STATUS_HPP
