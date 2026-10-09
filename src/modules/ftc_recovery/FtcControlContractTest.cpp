/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#include "FtcRecoveryController.hpp"
#include "FtcRecoveryArbiter.hpp"
#include "../control_allocator/FtcAllocationPolicy.hpp"
#include <gtest/gtest.h>

TEST(FtcAllocationPolicy, DisabledIsNominalAndFaultMustPersist)
{
	FtcAllocationPolicy policy;
	FtcAllocationPolicy::Input in{};
	in.count = 4; in.landed = false; in.armed = in.supported = in.model_valid = in.authority_valid = true;
	in.attitude_authority = in.thrust_authority = in.yaw_authority = 1.f;
	in.estimate_age = 0.f;
	for (unsigned i = 0; i < 4; ++i) { in.lambda[i] = i ? 1.f : 0.8f; in.uncertainty[i] = 0.03f; }
	for (unsigned k = 1; k < 200; ++k) {
		in.now = in.model_timestamp = in.authority_timestamp = k * 20000;
		policy.update(0.02f, in);
	}
	EXPECT_FALSE(policy.active);
	EXPECT_FLOAT_EQ(policy.lambda(0), 1.f);
	in.enabled = true;
	for (unsigned k = 200; k < 225; ++k) {
		in.now = in.model_timestamp = in.authority_timestamp = k * 20000; policy.update(0.02f, in);
	}
	EXPECT_FALSE(policy.active);
	for (unsigned k = 225; k < 400; ++k) {
		in.now = in.model_timestamp = in.authority_timestamp = k * 20000; policy.update(0.02f, in);
	}
	ASSERT_TRUE(policy.active);
	EXPECT_NEAR(policy.lambda(0), 0.8f, 0.001f);
	in.now += 300000;
	policy.update(0.02f, in);
	EXPECT_EQ(policy.state, FtcAllocationPolicy::FALLBACK);
	EXPECT_NE(policy.reason & FtcAllocationPolicy::STALE_INPUT, 0u);
	for (unsigned k = 0; k < 60; ++k) { in.now += 20000; policy.update(0.02f, in); }
	EXPECT_FALSE(policy.active);
	EXPECT_FLOAT_EQ(policy.lambda(0), 1.f);
}
TEST(FtcAllocationPolicy, ResetAndUncertaintyCannotActivate)
{
	FtcAllocationPolicy policy;
	FtcAllocationPolicy::Input in{};
	in.enabled = in.armed = in.supported = in.model_valid = in.authority_valid = true;
	in.count = 4; in.landed = false; in.estimate_age = 0.f;
	in.attitude_authority = in.thrust_authority = in.yaw_authority = 1.f;
	for (unsigned i = 0; i < 4; ++i) { in.lambda[i] = 0.8f; in.uncertainty[i] = 0.5f; }
	for (unsigned k = 1; k < 200; ++k) {
		in.now = in.model_timestamp = in.authority_timestamp = k*20000;
		policy.update(0.02f, in);
	}
	EXPECT_FALSE(policy.active);
	EXPECT_NE(policy.reason & FtcAllocationPolicy::UNCERTAINTY, 0u);
}
TEST(FtcRecoveryArbiter, BlendTimeoutAndDisabledIdentity)
{
	FtcRecoveryArbiter arb;
	FtcRecoveryArbiter::Input in{};
	in.now = in.normal_timestamp = in.candidate_timestamp = 1000000;
	in.normal[0] = 0.17f; in.normal[3] = -0.5f;
	in.candidate[0] = -1.f; in.candidate[3] = -0.6f;
	in.flight_allowed = in.candidate_valid = in.mode_matches = true;
	arb.update(0.01f, in);
	EXPECT_FLOAT_EQ(arb.output[0], in.normal[0]);
	EXPECT_FALSE(arb.active);
	in.enabled = true;
	for (unsigned i = 0; i < 120; ++i) {
		in.now += 10000; in.normal_timestamp = in.candidate_timestamp = in.now; arb.update(0.01f, in);
	}
	ASSERT_TRUE(arb.active);
	EXPECT_NEAR(arb.output[0], -1.f, 0.001f);
	for (unsigned i = 0; i < 100; ++i) {
		in.now += 10000; in.normal_timestamp = in.now; arb.update(0.01f, in);
	}
	EXPECT_FALSE(arb.active);
	EXPECT_FLOAT_EQ(arb.output[0], in.normal[0]);
	EXPECT_NE(arb.reason & FtcRecoveryArbiter::STALE, 0u);
}
TEST(FtcRecoveryController, CompleteStateSequenceAndReentry)
{
	FtcRecoveryController controller;
	FtcRecoveryController::Input in{};
	in.enabled = in.fresh = in.controllable = in.eligible = in.mode_allowed = in.vertical_valid = in.position_valid = true;
	in.mode = 3; in.z = -10.f;
	in.now = 1000000;
	controller.update(0.02f, in);
	ASSERT_EQ(controller.output().state, FtcRecoveryController::MONITORING);
	in.armed = true; in.landed = false; in.trigger = 1;
	bool seen[12] {};
	for (unsigned i = 0; i < 600; ++i) {
		in.now += 20000;
		controller.update(0.02f, in);
		seen[controller.output().state] = true;
	}
	for (unsigned state : {3u,4u,5u,6u,7u,11u}) { EXPECT_TRUE(seen[state]) << state; }
	EXPECT_EQ(controller.output().state, FtcRecoveryController::MONITORING);
	EXPECT_FALSE(controller.output().candidate_valid);
}
TEST(FtcRecoveryController, UncontrollableFailsAndFailsafeAborts)
{
	for (bool failsafe : {false, true}) {
		FtcRecoveryController controller;
		FtcRecoveryController::Input in{};
		in.enabled = in.fresh = in.eligible = in.mode_allowed = in.vertical_valid = true;
		in.now = 1000000;
		controller.update(0.02f, in);
		in.armed = true; in.landed = false; in.trigger = 1; in.failsafe = failsafe;
		in.now += 20000;
		controller.update(0.02f, in);
		EXPECT_EQ(controller.output().state, failsafe ? FtcRecoveryController::ABORTED : FtcRecoveryController::FAILED);
		EXPECT_FALSE(controller.output().candidate_valid);
	}
}
TEST(FtcRecoveryController, PositionLossUsesControlledVerticalDescent)
{
	FtcRecoveryController controller;
	FtcRecoveryController::Input in{};
	in.enabled = in.fresh = in.eligible = in.mode_allowed = in.vertical_valid = in.controllable = true;
	in.now = 1000000; in.z = -10.f;
	controller.update(0.02f, in);
	in.armed = true; in.landed = false; in.trigger = 1;
	for (unsigned i = 0; i < 300; ++i) { in.now += 20000; controller.update(0.02f, in); }
	EXPECT_EQ(controller.output().state, FtcRecoveryController::EMERGENCY_LAND);
	EXPECT_NEAR(controller.output().vertical_speed, 0.7f, 0.001f);
	EXPECT_LT(controller.output().thrust, 0.f);
	EXPECT_GT(controller.output().thrust, -1.f);
}

TEST(FtcAllocationPolicy, StructuralGateRestoresNominalImmediately)
{
 FtcAllocationPolicy policy;
 FtcAllocationPolicy::Input in{};
 in.enabled = in.armed = in.supported = in.model_valid = in.authority_valid = true;
 in.count = 4; in.landed = false; in.estimate_age = 0.f;
 in.attitude_authority = in.thrust_authority = 1.f; in.yaw_authority = 0.1f;
 for (unsigned i = 0; i < 4; ++i) { in.lambda[i] = 0.7f; in.uncertainty[i] = 0.02f; }
 for (unsigned k = 1; k < 200; ++k) {
  in.now = in.model_timestamp = in.authority_timestamp = k*20000; policy.update(0.02f, in);
 }
 ASSERT_TRUE(policy.active); EXPECT_LT(policy.yaw_weight, 0.1f);
 in.supported = false; policy.update(0.02f, in);
 EXPECT_FALSE(policy.active); EXPECT_FLOAT_EQ(policy.lambda(0), 1.f); EXPECT_FLOAT_EQ(policy.yaw_weight, 1.f);
}
TEST(FtcRecoveryArbiter, RejectsOutOfBoundsCandidateAndModeChange)
{
 FtcRecoveryArbiter arb;
 FtcRecoveryArbiter::Input in{};
 in.enabled = in.flight_allowed = in.candidate_valid = in.mode_matches = true;
 in.now = in.normal_timestamp = in.candidate_timestamp = 1000000;
 in.normal[3] = -0.5f; in.candidate[3] = 0.5f;
 arb.update(0.02f, in); EXPECT_FALSE(arb.active); EXPECT_NE(arb.reason & FtcRecoveryArbiter::INVALID, 0u);
 in.candidate[3] = -0.5f; arb.update(0.02f, in); ASSERT_TRUE(arb.active);
 in.mode_matches = false; in.hard_exit = true; arb.update(0.02f, in);
 EXPECT_FALSE(arb.active); EXPECT_FLOAT_EQ(arb.output[3], in.normal[3]);
}
TEST(FtcRecoveryController, InvertedTimeoutCannotCommandVerticalDescent)
{
 FtcRecoveryController controller;
 FtcRecoveryController::Input in{};
 in.enabled = in.fresh = in.eligible = in.mode_allowed = in.vertical_valid = in.controllable = true;
 in.now = 1000000; controller.update(0.02f, in);
 in.armed = true; in.landed = false; in.trigger = 1; in.q[0] = 0.f; in.q[1] = 1.f;
 for (unsigned i = 0; i < 1100; ++i) { in.now += 20000; controller.update(0.02f, in); }
 EXPECT_EQ(controller.output().state, FtcRecoveryController::FAILED);
 EXPECT_FALSE(controller.output().candidate_valid); EXPECT_NE(controller.output().fallback_reason, 0u);
}
TEST(FtcRecoveryController, ReentryWaitsForNormalSetpointMatching)
{
 FtcRecoveryController controller;
 FtcRecoveryController::Input in{};
 in.enabled = in.fresh = in.eligible = in.mode_allowed = in.vertical_valid = in.position_valid = in.controllable = true;
 in.now = 1000000; controller.update(0.02f, in);
 in.armed = true; in.landed = false; in.trigger = 1; in.normal_rates[0] = 1.5f;
 for (unsigned i = 0; i < 550; ++i) { in.now += 20000; controller.update(0.02f, in); }
 EXPECT_EQ(controller.output().state, FtcRecoveryController::CONTROL_REENTRY);
 EXPECT_FALSE(controller.output().reentry_ready); EXPECT_FLOAT_EQ(controller.output().reentry_weight, 1.f);
}
