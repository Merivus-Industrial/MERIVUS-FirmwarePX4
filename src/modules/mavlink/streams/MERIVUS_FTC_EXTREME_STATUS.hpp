/****************************************************************************
 * Copyright (c) 2026 MERIVUS. All rights reserved.
 ****************************************************************************/

#ifndef MERIVUS_FTC_EXTREME_STATUS_HPP
#define MERIVUS_FTC_EXTREME_STATUS_HPP

#include "MerivusFtcTelemetry.hpp"

#include <uORB/topics/ftc_extreme_state.h>
#include <uORB/topics/ftc_recovery_status.h>

class MavlinkStreamMerivusFtcExtremeStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMerivusFtcExtremeStatus(mavlink); }
	static constexpr const char *get_name_static() { return "MERIVUS_FTC_EXTREME_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MERIVUS_FTC_EXTREME_STATUS; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _extreme_sub.advertised() ? MAVLINK_MSG_ID_MERIVUS_FTC_EXTREME_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMerivusFtcExtremeStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _extreme_sub{ORB_ID(ftc_extreme_state)};
	uORB::Subscription _recovery_sub{ORB_ID(ftc_recovery_status)};
	ftc_extreme_state_s _extreme{};
	ftc_recovery_status_s _recovery{};

	bool send() override
	{
		bool updated = _extreme_sub.update(&_extreme);
		updated |= _recovery_sub.update(&_recovery);

		if (!updated) {
			return false;
		}

		mavlink_merivus_ftc_extreme_status_t msg{};
		msg.time_usec = _extreme.timestamp > 0 ? _extreme.timestamp : _recovery.timestamp;
		msg.event_time_usec = _extreme.event_timestamp;
		msg.loss_of_control_reason_mask = _extreme.loss_of_control_reason_mask;
		msg.recovery_trigger_mask = _recovery.trigger_mask;
		msg.recovery_inhibit_mask = _recovery.inhibit_reason_mask;
		msg.protocol_version = merivus_ftc_telemetry::ProtocolVersion;
		msg.impact_type = _extreme.impact_type;
		msg.loc_state = _extreme.loc_state;
		msg.recovery_state = _recovery.state;
		msg.flags = (_extreme.valid ? MERIVUS_FTC_EXTREME_FLAGS_VALID : 0)
			| (_extreme.impact_detected ? MERIVUS_FTC_EXTREME_FLAGS_IMPACT_DETECTED : 0)
			| (_extreme.hard_landing ? MERIVUS_FTC_EXTREME_FLAGS_HARD_LANDING : 0)
			| (_recovery.eligible ? MERIVUS_FTC_EXTREME_FLAGS_RECOVERY_ELIGIBLE : 0)
			| (_recovery.intervention_enabled ? MERIVUS_FTC_EXTREME_FLAGS_INTERVENTION_ENABLED : 0)
			| (_recovery.candidate_valid ? MERIVUS_FTC_EXTREME_FLAGS_RECOVERY_CANDIDATE_VALID : 0)
			| (_recovery.active ? MERIVUS_FTC_EXTREME_FLAGS_RECOVERY_STATE_ACTIVE : 0);
		msg.impact_score_pct = merivus_ftc_telemetry::encode_percentage(_extreme.impact_score, _extreme.valid);
		msg.impact_confidence_pct = merivus_ftc_telemetry::encode_percentage(_extreme.impact_confidence, _extreme.valid);
		msg.impact_severity_pct = merivus_ftc_telemetry::encode_percentage(_extreme.impact_severity, _extreme.valid);
		msg.loss_of_control_score_pct = merivus_ftc_telemetry::encode_percentage(_extreme.loss_of_control_score, _extreme.valid);
		msg.recovery_progress_pct = merivus_ftc_telemetry::encode_percentage(_recovery.progress, _recovery.candidate_valid);

		mavlink_msg_merivus_ftc_extreme_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MERIVUS_FTC_EXTREME_STATUS_HPP
