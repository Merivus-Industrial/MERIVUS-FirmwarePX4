/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#include "MotorEffectivenessEstimator.hpp"
#include "CommandAlignment.hpp"
#include "RigidBodyObserver.hpp"
#include <gtest/gtest.h>
#include <math.h>

namespace
{
using Estimator = MotorEffectivenessEstimator;
struct Fixture {
	Estimator estimator;
	Estimator::Configuration config{};
	uint64_t time{1000000};
	void run(float seconds, float lambda = 1.f, bool maneuver = true, bool saturated = false, bool correlated = false)
	{
		for (int k = 0; k < int(seconds / 0.02f); ++k) {
			Estimator::Input input{};
			input.timestamp = time;
			input.motor_count = 4;
			input.valid = input.aligned = true;
			input.saturated = saturated;
			for (int i = 0; i < 4; ++i) {
				input.control[i] = 0.5f + (maneuver ? 0.12f * sinf(float(time) * 1e-6f * (correlated ? 2.f : 1.2f + i * 0.7f)) : 0.f);
				input.geometry[0][i] = i == 0 || i == 3 ? 0.2f : -0.2f;
				input.geometry[1][i] = i < 2 ? 0.2f : -0.2f;
				input.geometry[2][i] = i % 2 ? 0.04f : -0.04f;
				for (int a = 0; a < 3; ++a) {
					input.angular_acceleration[a] += input.geometry[a][i] * input.control[i] * (i == 0 ? lambda : 1.f) * 40.f;
				}
			}
			estimator.update(0.02f, input, config);
			time += 20000;
		}
	}
};
}
TEST(MotorEffectivenessEstimator, DefaultBaselineAndLowExcitationLifecycle)
{
	Fixture f;
	f.run(35.f);
	ASSERT_TRUE(f.estimator.output().baseline_learned);
	ASSERT_TRUE(f.estimator.output().estimate_valid);
	f.run(7.f, 1.f, false);
	EXPECT_TRUE(f.estimator.output().baseline_learned);
	EXPECT_FALSE(f.estimator.output().current_observable);
	EXPECT_EQ(f.estimator.output().state, Estimator::TEMPORARILY_UNOBSERVABLE);
	const float uncertainty = f.estimator.output().uncertainty[0];
	const uint64_t last_valid = f.estimator.output().last_valid_timestamp;
	f.run(20.f, 1.f, false);
	EXPECT_TRUE(f.estimator.output().baseline_learned);
	EXPECT_EQ(f.estimator.output().state, Estimator::STALE);
	EXPECT_FALSE(f.estimator.output().estimate_valid);
	EXPECT_GT(f.estimator.output().estimate_age, 10.f);
	EXPECT_GT(f.estimator.output().uncertainty[0], uncertainty);
	EXPECT_EQ(f.estimator.output().last_valid_timestamp, last_valid);
}
TEST(MotorEffectivenessEstimator, LowExcitationCannotManufactureConfidence)
{
	Fixture f;
	f.run(60.f, 1.f, false);
	EXPECT_FALSE(f.estimator.output().baseline_learned);
	EXPECT_FALSE(f.estimator.output().estimate_valid);
	EXPECT_FLOAT_EQ(f.estimator.output().confidence[0], 0.f);
}
TEST(MotorEffectivenessEstimator, PausedUpdateCannotCertifyPreviouslyInvalidEstimate)
{
	Fixture f;
	f.config.confidence_minimum = 1.f;
	f.run(35.f);
	ASSERT_TRUE(f.estimator.output().baseline_learned);
	ASSERT_GT(f.estimator.output().update_count, 0u);
	ASSERT_FALSE(f.estimator.output().estimate_valid);
	ASSERT_EQ(f.estimator.output().last_valid_timestamp, 0u);
	// A quality gate change without an observation must not certify a previously rejected estimate.
	f.config.confidence_minimum = 0.6f;
	f.run(1.f, 1.f, true, true);
	EXPECT_FALSE(f.estimator.output().estimate_valid);
	EXPECT_EQ(f.estimator.output().last_valid_timestamp, 0u);
	f.run(1.f);
	EXPECT_TRUE(f.estimator.output().estimate_valid);
	EXPECT_GT(f.estimator.output().last_valid_timestamp, 0u);
}
TEST(MotorEffectivenessEstimator, ValidHistorySurvivesPausedUpdatesAndExpires)
{
	Fixture f;
	f.run(35.f);
	ASSERT_TRUE(f.estimator.output().estimate_valid);
	const auto last_valid = f.estimator.output().last_valid_timestamp;
	ASSERT_GT(last_valid, 0u);
	f.run(2.f, 1.f, true, true);
	EXPECT_TRUE(f.estimator.output().estimate_valid);
	EXPECT_EQ(f.estimator.output().last_valid_timestamp, last_valid);
	EXPECT_GT(f.estimator.output().estimate_age, 1.9f);
	f.run(12.f, 1.f, false, true);
	EXPECT_TRUE(f.estimator.output().baseline_learned);
	EXPECT_FALSE(f.estimator.output().estimate_valid);
	EXPECT_EQ(f.estimator.output().last_valid_timestamp, last_valid);
	EXPECT_EQ(f.estimator.output().state, Estimator::STALE);
}
TEST(MotorEffectivenessEstimator, CorrelatedCollectiveIsRankDeficient)
{
	Fixture f;
	f.run(45.f, 1.f, true, false, true);
	EXPECT_FALSE(f.estimator.output().current_observable);
	EXPECT_FALSE(f.estimator.output().baseline_learned);
	EXPECT_GT(f.estimator.output().condition_number, 1000.f);
}
TEST(MotorEffectivenessEstimator, SaturationPausesUpdatesWithoutErasingBaseline)
{
	Fixture f;
	f.run(35.f);
	const uint32_t updates = f.estimator.output().update_count;
	const float lambda = f.estimator.output().effectiveness[0];
	f.run(4.f, 0.7f, true, true);
	EXPECT_TRUE(f.estimator.output().baseline_learned);
	EXPECT_FALSE(f.estimator.output().update_allowed);
	EXPECT_EQ(f.estimator.output().update_count, updates);
	EXPECT_FLOAT_EQ(f.estimator.output().effectiveness[0], lambda);
	EXPECT_GT(f.estimator.output().estimate_age, 3.f);
}
TEST(MotorEffectivenessEstimator, SyntheticTruthLevels)
{
	for (float truth : {1.f, 0.9f, 0.8f, 0.7f}) {
		Fixture f;
		f.run(35.f);
		ASSERT_TRUE(f.estimator.output().baseline_learned);
		f.run(35.f, truth);
		EXPECT_NEAR(f.estimator.output().effectiveness[0], truth, 0.06f) << truth;
		for (int i = 1; i < 4; ++i) { EXPECT_GT(f.estimator.output().effectiveness[i], 0.93f); }
		EXPECT_TRUE(f.estimator.output().estimate_valid) << truth;
	}
}
TEST(CommandAlignment, UsesActuatorPublicationTimeAndRejectsGaps)
{
	CommandAlignment buffer;
	float a[12] {}, b[12] {}, output[12] {};
	a[0] = 0.4f; b[0] = 0.8f;
	buffer.push(100000, a); buffer.push(120000, b);
	ASSERT_TRUE(buffer.sample(150000, 40000, output));
	EXPECT_FLOAT_EQ(output[0], 0.4f);
	ASSERT_TRUE(buffer.sample(165000, 40000, output));
	EXPECT_FLOAT_EQ(output[0], 0.8f);
	EXPECT_FALSE(buffer.sample(300000, 40000, output));
	EXPECT_FALSE(buffer.sample(90000, 40000, output));
	buffer.push(1000, a);
	EXPECT_FALSE(buffer.sample(150000, 40000, output));
}
TEST(RigidBodyObserver, RequiresIndependentThrustScaleAndExpires)
{
	RigidBodyObserver observer;
	for (int i = 0; i < 300; ++i) { observer.update(0.02f, 0.f, 9.81f, true); }
	EXPECT_FALSE(observer.valid());
	for (int i = 0; i < 300; ++i) { observer.update(0.02f, 19.62f, 9.81f, true); }
	ASSERT_TRUE(observer.valid());
	EXPECT_NEAR(observer.mass(), 2.f, 0.001f);
	observer.update(3.f, 19.62f, 9.81f, false);
	EXPECT_FALSE(observer.valid());
}
