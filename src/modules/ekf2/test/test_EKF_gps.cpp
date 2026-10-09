/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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

/**
 * Test the gps fusion
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include <cmath>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfGpsTest : public ::testing::Test
{
public:

	EkfGpsTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EkfGpsTest, gpsTimeout)
{
	// GIVEN:EKF that fuses GPS

	// WHEN: setting the PDOP to high
	_sensor_simulator._gps.setNumberOfSatellites(3);

	// THEN: EKF should stop fusing GPS
	_sensor_simulator.runSeconds(20);

	// TODO: this is not happening as expected
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
}

TEST_F(EkfGpsTest, resetToGpsVelocity)
{
	ResetLoggingChecker reset_logging_checker(_ekf);
	// GIVEN:EKF that fuses GPS
	// and has gps checks already passed

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	reset_logging_checker.capturePreResetState();

	// AND: simulate constant velocity gps samples for short time
	_sensor_simulator.startGps();
	const Vector3f simulated_velocity(0.5f, 1.0f, -0.3f);
	_sensor_simulator._gps.setVelocity(simulated_velocity);
	const uint64_t dt_us = 1e5;
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f(simulated_velocity) * dt_us * 1e-6);
	_sensor_simulator._gps.stepHeightByMeters(simulated_velocity(2) * dt_us * 1e-6f);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runMicroseconds(dt_us);

	// THEN: a reset to GPS velocity should be done
	const Vector3f estimated_velocity = _ekf->getVelocity();
	EXPECT_NEAR(estimated_velocity(0), simulated_velocity(0), 1e-3f);
	EXPECT_NEAR(estimated_velocity(1), simulated_velocity(1), 1e-3f);
	EXPECT_NEAR(estimated_velocity(2), simulated_velocity(2), 1e-3f);

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(1e-2f));
}

TEST_F(EkfGpsTest, resetToGpsPosition)
{
	// GIVEN:EKF that fuses GPS
	// and has gps checks already passed
	const Vector3f previous_position = _ekf->getPosition();

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// AND: simulate jump in position
	_sensor_simulator.startGps();
	const Vector3f simulated_position_change(2.0f, -1.0f, 0.f);
	_sensor_simulator._gps.stepHorizontalPositionByMeters(
		Vector2f(simulated_position_change));
	_sensor_simulator.runMicroseconds(1e5);

	// THEN: a reset to the new GPS position should be done
	const Vector3f estimated_position = _ekf->getPosition();
	EXPECT_TRUE(isEqual(estimated_position,
			    previous_position + simulated_position_change, 1e-2f));
}

TEST_F(EkfGpsTest, gpsHgtToBaroFallback)
{
	// GIVEN: EKF that fuses GPS and flow, and in GPS height mode
	_sensor_simulator._flow.setData(_sensor_simulator._flow.dataAtRest());
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();

	_ekf_wrapper.enableGpsHeightFusion();

	_sensor_simulator.runSeconds(1);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// THEN: the height source should automatically change to baro
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}

TEST_F(EkfGpsTest, altitudeDrift)
{
	// GIVEN: a drifting GNSS altitude
	const float dt = 0.2f;
	const float height_rate = 0.15f;
	const float duration = 80.f;

	// WHEN: running on ground
	for (int i = 0; i < (duration / dt); i++) {
		_sensor_simulator._gps.stepHeightByMeters(height_rate * dt);
		_sensor_simulator.runSeconds(dt);
	}

	float baro_innov;
	_ekf->getBaroHgtInnov(baro_innov);
	BiasEstimator::status status = _ekf->getBaroBiasEstimatorStatus();

	printf("baro innov = %f\n", (double)baro_innov);
	printf("bias: %f, innov bias = %f\n", (double)status.bias, (double)status.innov);

	// THEN: the baro and local position should follow it
	EXPECT_LT(fabsf(baro_innov), 0.1f);
}

// Exercise the actual GNSS input buffer, quality checks, fusion controller and
// reset paths. No mock implementation of the two-dimensional fusion is used.
class EkfGpsVelocityDimensionsTest : public ::testing::Test
{
public:
	std::shared_ptr<Ekf> _ekf{std::make_shared<Ekf>()};
	SensorSimulator _sensor_simulator{_ekf};
	EkfWrapper _ekf_wrapper{_ekf};

	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1f);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
		_sensor_simulator.runSeconds(2.f);

		// Isolate velocity dimensions from GNSS-height-source changes. Barometer
		// aiding remains enabled throughout these tests.
		_ekf->getParamHandle()->gnss_ctrl = GnssCtrl::HPOS | GnssCtrl::VEL;
		_ekf->getParamHandle()->gps_vel_noise = 0.3f;
		_ekf->getParamHandle()->req_sacc = 0.5f;
	}

	gpsMessage horizontalGps(float accuracy = 0.2f)
	{
		gpsMessage gps = _sensor_simulator._gps.getDefaultGpsData();
		gps.vel_ne_valid = true;
		gps.vel_ned_valid = false;
		gps.sacc = accuracy;
		return gps;
	}

	void startGps(const gpsMessage &gps)
	{
		_sensor_simulator._gps.setData(gps);
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11.f);
	}

	bool gsfHasStarted()
	{
		float yaw{};
		float variance{};
		float models[5]{};
		float innov_n[5]{};
		float innov_e[5]{};
		float weights[5]{};
		return _ekf->getDataEKFGSF(&yaw, &variance, models, innov_n, innov_e, weights);
	}

	void expectMalformedAccuracyRejected(float accuracy)
	{
		startGps(horizontalGps(accuracy));
		EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
		EXPECT_TRUE(_ekf->gps_check_fail_status_flags().sacc);
		EXPECT_EQ(_ekf->get_gps_sample_delayed().time_us, 0u);
		EXPECT_TRUE(_ekf->getVelocity().isAllFinite());
		EXPECT_FALSE(gsfHasStarted());
	}
};

TEST_F(EkfGpsVelocityDimensionsTest, noGnssSampleKeepsLegacyDiagnosticsUntilActualHorizontalOnlyData)
{
	ASSERT_EQ(_ekf->get_gps_sample_delayed().time_us, 0u);
	float horizontal_velocity[2]{};
	float vertical_velocity{};
	float horizontal_position[2]{};
	float vertical_position{};
	float horizontal_velocity_ratio{};
	float horizontal_position_ratio{};

	_ekf->getGpsVelPosInnov(horizontal_velocity, vertical_velocity, horizontal_position, vertical_position);
	EXPECT_FLOAT_EQ(vertical_velocity, 0.f);
	_ekf->getGpsVelPosInnovVar(horizontal_velocity, vertical_velocity, horizontal_position, vertical_position);
	EXPECT_FLOAT_EQ(vertical_velocity, 0.f);
	_ekf->getGpsVelPosInnovRatio(horizontal_velocity_ratio, vertical_velocity, horizontal_position_ratio,
				   vertical_position);
	EXPECT_FLOAT_EQ(vertical_velocity, 0.f);

	// A received 2D sample is a different state from "no sample ever received".
	startGps(horizontalGps());
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	ASSERT_GT(_ekf->get_gps_sample_delayed().time_us, 0u);
	ASSERT_FALSE(_ekf->get_gps_sample_delayed().vel_d_valid);
	_ekf->getGpsVelPosInnov(horizontal_velocity, vertical_velocity, horizontal_position, vertical_position);
	EXPECT_TRUE(std::isnan(vertical_velocity));
	_ekf->getGpsVelPosInnovVar(horizontal_velocity, vertical_velocity, horizontal_position, vertical_position);
	EXPECT_TRUE(std::isnan(vertical_velocity));
	_ekf->getGpsVelPosInnovRatio(horizontal_velocity_ratio, vertical_velocity, horizontal_position_ratio,
				   vertical_position);
	EXPECT_TRUE(std::isnan(vertical_velocity));
}

TEST_F(EkfGpsVelocityDimensionsTest, horizontalOnlyStartsAndFusesWithoutDownObservation)
{
	gpsMessage gps = horizontalGps();
	// A conspicuous storage value must not be used when the dimension is absent.
	gps.vel_ned(2) = 75.f;
	const uint8_t vertical_resets = _ekf->get_velD_reset_count();
	startGps(gps);

	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	const auto &aid = _ekf->aid_src_gnss_vel();
	EXPECT_TRUE(aid.fused);
	EXPECT_FALSE(aid.innovation_rejected);
	EXPECT_TRUE(PX4_ISFINITE(aid.observation[0]));
	EXPECT_TRUE(PX4_ISFINITE(aid.observation[1]));
	EXPECT_TRUE(std::isnan(aid.observation[2]));
	EXPECT_TRUE(std::isnan(aid.innovation[2]));
	EXPECT_TRUE(std::isnan(aid.innovation_variance[2]));
	EXPECT_TRUE(std::isnan(aid.test_ratio[2]));
	EXPECT_FALSE(_ekf->get_gps_sample_delayed().vel_d_valid);
	EXPECT_FALSE(_ekf->isVerticalVelocityAidingActive());
	EXPECT_EQ(_ekf->get_velD_reset_count(), vertical_resets);
	EXPECT_NEAR(_ekf->getVelocity()(2), 0.f, 0.01f);

	float horizontal_ratio{};
	float vertical_ratio{};
	float position_ratio{};
	float height_ratio{};
	_ekf->getGpsVelPosInnovRatio(horizontal_ratio, vertical_ratio, position_ratio, height_ratio);
	EXPECT_TRUE(PX4_ISFINITE(horizontal_ratio));
	EXPECT_TRUE(std::isnan(vertical_ratio));
}

TEST_F(EkfGpsVelocityDimensionsTest, horizontalOnlyRestartResetsNEButNotDown)
{
	startGps(horizontalGps());
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11.f);
	ASSERT_FALSE(_ekf_wrapper.isIntendingGpsFusion());

	ResetLoggingChecker resets(_ekf);
	resets.capturePreResetState();
	const float down_velocity = _ekf->getVelocity()(2);
	gpsMessage gps = horizontalGps();
	gps.vel_ned = Vector3f(0.5f, 1.f, 75.f);
	_sensor_simulator._gps.setData(gps);
	_sensor_simulator.startGps();
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runMicroseconds(100000);

	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	resets.capturePostResetState();
	EXPECT_TRUE(resets.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(resets.isVerticalVelocityResetCounterIncreasedBy(0));
	EXPECT_NEAR(_ekf->getVelocity()(0), gps.vel_ned(0), 0.01f);
	EXPECT_NEAR(_ekf->getVelocity()(1), gps.vel_ned(1), 0.01f);
	EXPECT_NEAR(_ekf->getVelocity()(2), down_velocity, 0.01f);
	EXPECT_FALSE(_ekf->gps_check_fail_status_flags().vspeed);
}

TEST_F(EkfGpsVelocityDimensionsTest, legacyThreeDimensionalRestartStillResetsAllAxes)
{
	const gpsMessage initial = _sensor_simulator._gps.getDefaultGpsData();
	ASSERT_FALSE(initial.vel_ne_valid);
	ASSERT_TRUE(initial.vel_ned_valid);
	startGps(initial);
	ASSERT_TRUE(_ekf->isVerticalVelocityAidingActive());
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11.f);
	ASSERT_FALSE(_ekf_wrapper.isIntendingGpsFusion());

	ResetLoggingChecker resets(_ekf);
	resets.capturePreResetState();
	gpsMessage gps = initial;
	gps.vel_ned = Vector3f(0.5f, 1.f, -0.3f);
	_sensor_simulator._gps.setData(gps);
	_sensor_simulator.startGps();
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runMicroseconds(100000);

	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	resets.capturePostResetState();
	EXPECT_TRUE(resets.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(resets.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_NEAR(_ekf->getVelocity()(2), gps.vel_ned(2), 0.01f);
	EXPECT_TRUE(_ekf->get_gps_sample_delayed().vel_d_valid);
	EXPECT_TRUE(PX4_ISFINITE(_ekf->aid_src_gnss_vel().observation[2]));
}

TEST_F(EkfGpsVelocityDimensionsTest, dimensionSwitchClearsAndRestoresDownStatus)
{
	startGps(_sensor_simulator._gps.getDefaultGpsData());
	ASSERT_TRUE(_ekf->isVerticalVelocityAidingActive());
	const uint8_t vertical_resets = _ekf->get_velD_reset_count();
	gpsMessage gps = horizontalGps();
	gps.vel_ned(2) = 75.f;
	_sensor_simulator._gps.setData(gps);
	_sensor_simulator.runSeconds(1.f);

	EXPECT_TRUE(_ekf->aid_src_gnss_vel().fused);
	EXPECT_FALSE(_ekf->isVerticalVelocityAidingActive());
	EXPECT_TRUE(std::isnan(_ekf->aid_src_gnss_vel().observation[2]));
	EXPECT_EQ(_ekf->get_velD_reset_count(), vertical_resets);

	_sensor_simulator._gps.setData(_sensor_simulator._gps.getDefaultGpsData());
	_sensor_simulator.runSeconds(1.f);
	EXPECT_TRUE(_ekf->aid_src_gnss_vel().fused);
	EXPECT_TRUE(_ekf->isVerticalVelocityAidingActive());
	EXPECT_TRUE(PX4_ISFINITE(_ekf->aid_src_gnss_vel().observation[2]));
	EXPECT_EQ(_ekf->get_velD_reset_count(), vertical_resets);
}

TEST_F(EkfGpsVelocityDimensionsTest, unknownAccuracyUsesNoiseFloorButDoesNotStartGsf)
{
	startGps(horizontalGps(0.f));
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf->aid_src_gnss_vel().fused);
	EXPECT_FLOAT_EQ(_ekf->get_gps_sample_delayed().sacc, 0.f);
	EXPECT_FLOAT_EQ(_ekf->aid_src_gnss_vel().observation_variance[0], 0.3f * 0.3f);
	EXPECT_FLOAT_EQ(_ekf->aid_src_gnss_vel().observation_variance[1], 0.3f * 0.3f);
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(3.f);
	EXPECT_FALSE(gsfHasStarted());
	EXPECT_FALSE(_ekf->isYawEmergencyEstimateAvailable());
}

TEST_F(EkfGpsVelocityDimensionsTest, reportedAccuracyAllowsHorizontalOnlyGsfInput)
{
	startGps(horizontalGps(0.2f));
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_FALSE(gsfHasStarted()); // Ground handling must not start the GSF EKFs.
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(1.f);
	EXPECT_TRUE(gsfHasStarted());
	// Starting the filter is not a claim that stationary yaw is observable or converged.
}

TEST_F(EkfGpsVelocityDimensionsTest, negativeAccuracyIsRejected)
{
	expectMalformedAccuracyRejected(-0.1f);
}

TEST_F(EkfGpsVelocityDimensionsTest, nanAccuracyIsRejected)
{
	expectMalformedAccuracyRejected(NAN);
}

TEST_F(EkfGpsVelocityDimensionsTest, infiniteAccuracyIsRejected)
{
	expectMalformedAccuracyRejected(INFINITY);
}

TEST_F(EkfGpsVelocityDimensionsTest, malformedDeclaredDownVelocityIsNotSilentlyAcceptedAs2D)
{
	gpsMessage gps = _sensor_simulator._gps.getDefaultGpsData();
	gps.vel_ned(2) = NAN;
	startGps(gps);
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf->gps_check_fail_status_flags().vspeed);
	EXPECT_EQ(_ekf->get_gps_sample_delayed().time_us, 0u);
	EXPECT_TRUE(_ekf->getVelocity().isAllFinite());
}
