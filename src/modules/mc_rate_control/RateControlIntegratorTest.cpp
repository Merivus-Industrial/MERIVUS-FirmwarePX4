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

#include <gtest/gtest.h>

#include "RateControlIntegrator.hpp"

using matrix::Vector3f;

class RateControlIntegratorTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		configure(controller);
		actuator_armed.timestamp = now;
		actuator_armed.armed = true;
		control_mode.flag_armed = true;
		control_mode.flag_control_rates_enabled = true;
		vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		vehicle_status.hil_state = vehicle_status_s::HIL_STATE_OFF;
	}

	static void configure(RateControl &rate_control)
	{
		rate_control.setGains(Vector3f{0.2f, 0.2f, 0.2f}, Vector3f{0.3f, 0.3f, 0.3f}, Vector3f{0.1f, 0.1f, 0.1f});
		rate_control.setIntegratorLimit(Vector3f{0.3f, 0.3f, 0.3f});
		rate_control.setFeedForwardGain(Vector3f{0.05f, 0.05f, 0.05f});
	}

	bool prepare()
	{
		// This is the production decision and reset operation used by Run(),
		// not a test-local reconstruction of its authorization conditions.
		return prepareRateControlIntegrator(controller, now, actuator_armed, control_mode,
				vehicle_status, maybe_landed, landed);
	}

	Vector3f step()
	{
		return controller.update(rate, setpoint, angular_accel, dt, prepare());
	}

	static Vector3f integral(RateControl &rate_control)
	{
		rate_ctrl_status_s status{};
		rate_control.getRateControlStatus(status);
		return Vector3f{status.rollspeed_integ, status.pitchspeed_integ, status.yawspeed_integ};
	}

	Vector3f integral() { return integral(controller); }

	static void expectVectorNear(const Vector3f &actual, const Vector3f &expected)
	{
		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_NEAR(actual(axis), expected(axis), 1e-6f) << "axis " << axis;
		}
	}

	void buildTrim()
	{
		for (int i = 0; i < 20; ++i) {
			step();
		}

		ASSERT_GT(integral()(0), 0.f);
		ASSERT_LT(integral()(1), 0.f);
		ASSERT_GT(integral()(2), 0.f);
	}

	RateControl controller;
	uint64_t now{2000000};
	actuator_armed_s actuator_armed{};
	vehicle_control_mode_s control_mode{};
	vehicle_status_s vehicle_status{};
	bool maybe_landed{false};
	bool landed{false};
	const float dt{0.01f};
	Vector3f rate{};
	Vector3f setpoint{0.3f, -0.2f, 0.1f};
	Vector3f angular_accel{};
};

TEST_F(RateControlIntegratorTest, NormalControlMatchesOriginalRateControl)
{
	RateControl original;
	configure(original);
	angular_accel = Vector3f{0.02f, -0.03f, 0.01f};

	for (int i = 0; i < 100; ++i) {
		EXPECT_FALSE(prepare());
		expectVectorNear(step(), original.update(rate, setpoint, angular_accel, dt, false));
		expectVectorNear(integral(), integral(original));
	}

	EXPECT_GT(integral()(0), 0.f);
}

TEST_F(RateControlIntegratorTest, AirborneManualKillFreezesTrimAndReleaseResumes)
{
	buildTrim();
	const Vector3f before_kill = integral();
	actuator_armed.manual_lockdown = true;
	setpoint = Vector3f{1.f, -0.7f, 0.5f};

	// 3.29 seconds of synthetic controller cycles with ongoing rate error.
	for (int i = 0; i < 329; ++i) {
		now += 10000;
		actuator_armed.timestamp = now;
		EXPECT_TRUE(prepare());
		const Vector3f torque = step();
		EXPECT_TRUE(torque.isAllFinite());
		EXPECT_GT(torque.norm(), 0.f); // The guard must not replace mixer output suppression.
		expectVectorNear(integral(), before_kill);
	}

	actuator_armed.manual_lockdown = false;
	setpoint.zero();
	EXPECT_FALSE(prepare());
	expectVectorNear(step(), before_kill);
	expectVectorNear(integral(), before_kill);
	setpoint = Vector3f{0.3f, -0.2f, 0.1f};
	step();
	EXPECT_GT(integral()(0), before_kill(0));
}

TEST_F(RateControlIntegratorTest, ForceFailsafeFreezesTrim)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.force_failsafe = true;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, PhysicalLockdownFreezesTrim)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.lockdown = true;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, HilLockdownDoesNotFreezeSimulatedControl)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.lockdown = true;
	vehicle_status.hil_state = vehicle_status_s::HIL_STATE_ON;
	EXPECT_FALSE(prepare());
	step();
	EXPECT_GT(integral()(0), before(0));
}

TEST_F(RateControlIntegratorTest, HilDoesNotBypassManualKill)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.lockdown = true;
	actuator_armed.manual_lockdown = true;
	vehicle_status.hil_state = vehicle_status_s::HIL_STATE_ON;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, EscCalibrationFreezesTrim)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.in_esc_calibration_mode = true;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, MissingAuthorizationFreezesWithoutInventingDisarm)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed = {};
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, FutureAuthorizationFreezesWithoutInventingDisarm)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.timestamp = now + 1;
	actuator_armed.armed = false;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, StaleAuthorizationFreezesWithoutInventingDisarm)
{
	buildTrim();
	const Vector3f before = integral();
	actuator_armed.timestamp = now - 1000001;
	actuator_armed.armed = false;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
}

TEST_F(RateControlIntegratorTest, ExactlyOneSecondIsFreshAndNextMicrosecondIsStale)
{
	actuator_armed.timestamp = now - 1000000;
	EXPECT_FALSE(prepare());
	step();
	const Vector3f at_boundary = integral();
	ASSERT_GT(at_boundary(0), 0.f);
	++now;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), at_boundary);
}

TEST_F(RateControlIntegratorTest, NewAuthorizationResumesAfterTimeout)
{
	buildTrim();
	const Vector3f before = integral();
	now += 1000001;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
	actuator_armed.timestamp = now;
	EXPECT_FALSE(prepare());
	step();
	EXPECT_GT(integral()(0), before(0));
}

TEST_F(RateControlIntegratorTest, ControlModeDisarmResetsAndInhibits)
{
	buildTrim();
	control_mode.flag_armed = false;
	EXPECT_TRUE(prepare());
	expectVectorNear(integral(), Vector3f{});
	step();
	expectVectorNear(integral(), Vector3f{});
}

TEST_F(RateControlIntegratorTest, FreshActuatorDisarmResetsDespiteArmedControlMode)
{
	buildTrim();
	actuator_armed.armed = false;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), Vector3f{});
}

TEST_F(RateControlIntegratorTest, DisarmResetsWhileRateControlIsDisabled)
{
	buildTrim();
	control_mode.flag_control_rates_enabled = false;
	control_mode.flag_armed = false;
	actuator_armed = {};
	// Run() invokes prepare outside its rate-control-enabled branch.
	EXPECT_TRUE(prepare());
	expectVectorNear(integral(), Vector3f{});
	control_mode.flag_armed = true;
	control_mode.flag_control_rates_enabled = true;
	actuator_armed.timestamp = now;
	actuator_armed.armed = true;
	setpoint.zero();
	expectVectorNear(step(), Vector3f{});
}

TEST_F(RateControlIntegratorTest, NonRotaryWingResetsAndInhibits)
{
	buildTrim();
	vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), Vector3f{});
}

TEST_F(RateControlIntegratorTest, LandedAndMaybeLandedFreezeWithoutErasingTrim)
{
	buildTrim();
	const Vector3f before = integral();
	landed = true;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
	landed = false;
	maybe_landed = true;
	EXPECT_TRUE(prepare());
	step();
	expectVectorNear(integral(), before);
	maybe_landed = false;
	EXPECT_FALSE(prepare());
	step();
	EXPECT_GT(integral()(0), before(0));
}

TEST_F(RateControlIntegratorTest, OriginalSaturationPreventionAndUnwindRemainEffective)
{
	buildTrim();
	const Vector3f before = integral();
	controller.setPositiveSaturationFlag(0, true);
	controller.setNegativeSaturationFlag(1, true);
	controller.setPositiveSaturationFlag(2, true);
	EXPECT_FALSE(prepare());
	step();
	expectVectorNear(integral(), before);
	setpoint = -setpoint;
	step();
	EXPECT_LT(integral()(0), before(0));
	EXPECT_GT(integral()(1), before(1));
	EXPECT_LT(integral()(2), before(2));
}

TEST_F(RateControlIntegratorTest, OriginalIntegralLimitsRemainEffective)
{
	// Allow every axis to reach its limit despite RateControl's error-dependent I gain.
	for (int i = 0; i < 2000; ++i) {
		step();
	}

	expectVectorNear(integral(), Vector3f{0.3f, -0.3f, 0.3f});
}
