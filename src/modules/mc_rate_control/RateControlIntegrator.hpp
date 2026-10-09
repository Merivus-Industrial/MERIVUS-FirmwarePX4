/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/rate_control/rate_control.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

/**
 * Prepare the actual rate-controller integral before every gyro cycle,
 * including cycles where rate control is disabled. The return value inhibits
 * integration in RateControl::update; it does not suppress actuator outputs.
 */
inline bool prepareRateControlIntegrator(RateControl &rate_control, const uint64_t control_time,
		const actuator_armed_s &actuator_armed, const vehicle_control_mode_s &control_mode,
		const vehicle_status_s &vehicle_status, const bool maybe_landed, const bool landed)
{
	// Commander publishes actuator_armed at 2 Hz and immediately on changes.
	// Unknown/stale output authority must not let the integral grow. Keep the
	// pre-lockdown trim rather than losing steady-state CG compensation.
	constexpr uint64_t actuator_armed_timeout_us = 1000000;
	const bool actuator_armed_fresh = actuator_armed.timestamp != 0
					 && actuator_armed.timestamp <= control_time
					 && control_time - actuator_armed.timestamp <= actuator_armed_timeout_us;
	const bool reset_integral = !control_mode.flag_armed
				    || vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				    || (actuator_armed_fresh && !actuator_armed.armed);

	// Also reset while rate control is disabled, so disarming cannot leave a
	// previous flight's integral waiting for the next rate-control activation.
	if (reset_integral) {
		rate_control.resetIntegral();
	}

	// HIL uses lockdown to inhibit physical outputs, not simulated control.
	// Actual output suppression remains the mixer's responsibility.
	return reset_integral || !actuator_armed_fresh
	       || actuator_armed.manual_lockdown || actuator_armed.force_failsafe
	       || actuator_armed.in_esc_calibration_mode
	       || (actuator_armed.lockdown && vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON)
	       || maybe_landed || landed;
}
