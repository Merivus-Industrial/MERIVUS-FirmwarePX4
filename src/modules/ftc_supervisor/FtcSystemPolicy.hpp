#pragma once
#include <math.h>
#include <uORB/topics/ftc_allocation_shadow.h>
#include <uORB/topics/ftc_allocation_status.h>
#include <uORB/topics/ftc_arbitration_status.h>
#include <uORB/topics/ftc_control_authority.h>
#include <uORB/topics/ftc_extreme_state.h>
#include <uORB/topics/ftc_model_status.h>
#include <uORB/topics/ftc_recovery_status.h>
#include <uORB/topics/ftc_system_status.h>
#include <uORB/topics/motor_health_status.h>

class FtcSystemPolicy
{
public:
    static ftc_system_status_s evaluate(uint64_t now, bool enabled,
        const motor_health_status_s &health, const ftc_model_status_s &model,
        const ftc_control_authority_s &authority, const ftc_extreme_state_s &extreme,
        const ftc_recovery_status_s &recovery, const ftc_allocation_status_s &allocation,
        const ftc_arbitration_status_s &arbitration, const ftc_allocation_shadow_s &shadow)
    {
        using S = ftc_system_status_s;
        const auto fresh = [now](uint64_t t) { return t && now >= t && now - t < 300000; };
        S out{};
        out.timestamp = now;
        out.monitor_enabled = enabled;
        out.model_valid = fresh(model.timestamp) && model.valid;
        out.control_authority_valid = fresh(authority.timestamp) && authority.valid;
        out.recovery_eligible = fresh(recovery.timestamp) && recovery.eligible;
        out.allocation_active = fresh(allocation.timestamp) && allocation.active;
        out.recovery_active = fresh(arbitration.timestamp) && arbitration.active;
        out.intervention_enabled = out.allocation_active || out.recovery_active;
        out.allocation_fallback = allocation.fallback_reason;
        out.recovery_fallback = arbitration.fallback_reason;
        out.degraded_motor_mask = fresh(health.timestamp) ? health.degraded_mask : 0;
        out.failed_motor_mask = fresh(health.timestamp) ? health.failed_mask : 0;
        out.fault_confirmed = out.degraded_motor_mask || out.failed_motor_mask;
        out.state = enabled ? S::INITIALIZING : S::DISABLED;
        out.mode = enabled ? 1 : 0;
        if (enabled) {
            if (!out.model_valid) {
                out.reason_mask |= 1u << 0;
                out.state = !fresh(model.timestamp) ? S::INITIALIZING
                    : (!model.baseline_learned ? S::CALIBRATING : S::UNOBSERVABLE);
            } else if (!out.control_authority_valid) {
                out.state = S::READY;
                out.reason_mask |= 1u << 3;
            } else {
                out.state = model.current_observable ? S::NORMAL : S::UNOBSERVABLE;
                if (fresh(shadow.timestamp) && shadow.valid) {
                    out.mode = 2;
                    if (model.current_observable) { out.state = S::SHADOW; }
                }
            }
            if (!fresh(health.timestamp) || !fresh(extreme.timestamp) || !fresh(recovery.timestamp)) {
                out.reason_mask |= 1u << 6;
                out.state = S::INITIALIZING;
            }
            if (out.degraded_motor_mask) { out.reason_mask |= 1u << 1; out.state = S::DEGRADED; }
            if (out.control_authority_valid && authority.state != ftc_control_authority_s::FULL_CONTROL) {
                out.reason_mask |= 1u << 3; out.state = S::DEGRADED;
            }
            if (fresh(extreme.timestamp) && extreme.valid
                && (extreme.impact_detected || extreme.loc_state >= ftc_extreme_state_s::LOC_RECOVERY_RECOMMENDED)) {
                out.reason_mask |= 1u << 4; out.state = S::DEGRADED;
            }
            // A confirmed failure remains visible when authority is also degraded.
            if (out.failed_motor_mask) { out.reason_mask |= 1u << 2; out.state = S::FAULT_CONFIRMED; }
            if (fresh(recovery.timestamp) && recovery.candidate_valid) {
                out.mode = 3; out.state = S::RECOVERY_CANDIDATE;
            }
        }
        // During a disable/fallback blend, report actual control ownership until it reaches zero.
        if (out.allocation_active) { out.state = S::ACTIVE_ALLOCATION; out.mode = 4; }
        if (out.recovery_active) {
            out.mode = 4;
            out.state = fresh(recovery.timestamp) && recovery.state == ftc_recovery_status_s::EMERGENCY_LAND
                ? S::EMERGENCY_LAND : S::RECOVERY_ACTIVE;
        }
        if (enabled && fresh(recovery.timestamp)
            && (recovery.state == ftc_recovery_status_s::ABORTED || recovery.state == ftc_recovery_status_s::FAILED)) {
            out.reason_mask |= 1u << 5; out.state = S::FAILED;
        }
        const float mc = out.model_valid ? model.model_quality : 0.f;
        const float ac = out.control_authority_valid ? authority.minimum_attitude_authority : 0.f;
        const float ec = fresh(extreme.timestamp) && extreme.valid ? 1.f - extreme.loss_of_control_score : 0.f;
        out.system_confidence = fminf(fmaxf((mc + ac + ec) / 3.f, 0.f), 1.f);
        return out;
    }
};
