#include "src/modules/control_allocator/FtcAllocationPolicy.hpp"
#include "src/modules/ftc_recovery/FtcRecoveryArbiter.hpp"
#include "src/modules/ftc_recovery/FtcRecoveryController.hpp"
#include <gtest/gtest.h>

static FtcAllocationPolicy::Input allocationInput()
{
    FtcAllocationPolicy::Input in{};
    in.enabled = in.armed = in.supported = in.model_valid = in.authority_valid = true;
    in.count = 4; in.estimate_age = 0; in.landed = false;
    in.attitude_authority = in.thrust_authority = in.yaw_authority = 1;
    for (unsigned i = 0; i < 4; ++i) { in.lambda[i] = i ? 1.f : .7f; in.uncertainty[i] = .04f; }
    return in;
}

TEST(AllocationSafety, AllSoftFallbacksReturnToExactNominal)
{
    for (int scenario = 0; scenario < 9; ++scenario) {
        SCOPED_TRACE(scenario);
        FtcAllocationPolicy policy; auto in = allocationInput();
        for (int i = 0; i < 200; ++i) {
            in.now += 20000; in.model_timestamp = in.authority_timestamp = in.now;
            policy.update(.02f, in);
        }
        ASSERT_TRUE(policy.active);
        for (int i = 0; i < 60; ++i) {
            in.now += 20000; in.model_timestamp = in.authority_timestamp = in.now;
            switch (scenario) {
            case 0: in.model_valid = false; break;
            case 1: in.estimate_age = 3; break;
            case 2: in.uncertainty[0] = .5f; break;
            case 3: in.authority_valid = false; break;
            case 4: in.enabled = false; break;
            case 5: in.model_timestamp = 1; break;
            case 6: in.authority_timestamp = 1; break;
            case 7: in.lambda[0] = NAN; break;
            case 8: in.thrust_authority = .1f; break;
            }
            const float before = policy.lambda(0); policy.update(.02f, in);
            EXPECT_LE(fabsf(policy.lambda(0)-before), .02001f);
        }
        EXPECT_FALSE(policy.active); EXPECT_FLOAT_EQ(policy.lambda(0), 1.f);
    }
}

TEST(AllocationSafety, NegativeEstimateAgeCannotActivate)
{
    FtcAllocationPolicy policy; auto in = allocationInput(); in.estimate_age = -1.f;
    for (int i = 0; i < 200; ++i) {
        in.now += 20000; in.model_timestamp = in.authority_timestamp = in.now; policy.update(.02f, in);
    }
    EXPECT_FALSE(policy.active);
}

TEST(AllocationSafety, LandingAndDisarmClearAppliedMatrix)
{
    for (int scenario = 0; scenario < 3; ++scenario) {
        FtcAllocationPolicy policy; auto in = allocationInput();
        for (int i = 0; i < 200; ++i) {
            in.now += 20000; in.model_timestamp = in.authority_timestamp = in.now; policy.update(.02f, in);
        }
        ASSERT_TRUE(policy.active);
        if (scenario == 0) in.landed = true;
        if (scenario == 1) in.armed = false;
        if (scenario == 2) in.supported = false;
        policy.update(.02f, in);
        EXPECT_FALSE(policy.active); EXPECT_FLOAT_EQ(policy.lambda(0), 1.f);
        EXPECT_FLOAT_EQ(policy.yaw_weight, 1.f);
    }
}

TEST(ArbitrationSafety, InvalidNormalImmediatelyReleasesFtcOwnership)
{
    FtcRecoveryArbiter arb; FtcRecoveryArbiter::Input in{};
    in.enabled = in.flight_allowed = in.candidate_valid = in.mode_matches = true;
    in.normal[3] = in.candidate[3] = -.5f;
    for (int i = 0; i < 100; ++i) {
        in.now += 20000; in.normal_timestamp = in.candidate_timestamp = in.now; arb.update(.02f, in);
    }
    ASSERT_TRUE(arb.active);
    in.normal[0] = NAN; arb.update(.02f, in);
    EXPECT_FALSE(arb.active); EXPECT_FLOAT_EQ(arb.weight, 0.f);
    EXPECT_NE(arb.reason & FtcRecoveryArbiter::INVALID, 0u);
    EXPECT_FLOAT_EQ(arb.output[3], in.normal[3]);
}

static FtcRecoveryController::Input recoveryInput()
{
    FtcRecoveryController::Input in{};
    in.enabled = in.fresh = in.eligible = in.mode_allowed = in.vertical_valid = in.position_valid = in.controllable = true;
    in.now = 1000000; in.z = -10;
    return in;
}

TEST(RecoverySafety, NonfiniteVerticalInputCannotProduceCandidate)
{
    for (int scenario = 0; scenario < 3; ++scenario) {
        SCOPED_TRACE(scenario);
        FtcRecoveryController controller; auto in = recoveryInput(); controller.update(.02f, in);
        in.armed = true; in.landed = false; in.trigger = 1;
        if (scenario == 0) in.z = NAN;
        if (scenario == 1) in.vz = NAN;
        if (scenario == 2) in.hover_thrust = NAN;
        for (int i = 0; i < 250; ++i) {
            in.now += 20000; controller.update(.02f, in);
            EXPECT_FALSE(controller.output().candidate_valid);
        }
    }
}

TEST(RecoverySafety, FlightAndAuthorityLossTerminateCandidate)
{
    for (int scenario = 0; scenario < 8; ++scenario) {
        SCOPED_TRACE(scenario);
        FtcRecoveryController controller; auto in = recoveryInput(); controller.update(.02f, in);
        in.armed = true; in.landed = false; in.trigger = 1;
        in.now += 20000; controller.update(.02f, in); ASSERT_TRUE(controller.output().candidate_valid);
        switch (scenario) {
        case 0: in.fresh = false; break;
        case 1: in.failsafe = true; break;
        case 2: in.mode_allowed = false; break;
        case 3: in.mode++; break;
        case 4: in.armed = false; break;
        case 5: in.landed = true; break;
        case 6: in.controllable = false; break;
        case 7: in.eligible = false; break;
        }
        in.now += 20000; controller.update(.02f, in); EXPECT_FALSE(controller.output().candidate_valid);
    }
}
