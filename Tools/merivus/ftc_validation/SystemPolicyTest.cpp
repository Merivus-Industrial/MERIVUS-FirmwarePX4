#include "src/modules/ftc_supervisor/FtcSystemPolicy.hpp"
#include <gtest/gtest.h>

struct SystemPolicyTest : testing::Test {
    uint64_t now{1000000};
    bool enabled{true};
    motor_health_status_s health{};
    ftc_model_status_s model{};
    ftc_control_authority_s authority{};
    ftc_extreme_state_s extreme{};
    ftc_recovery_status_s recovery{};
    ftc_allocation_status_s allocation{};
    ftc_arbitration_status_s arbitration{};
    ftc_allocation_shadow_s shadow{};
    SystemPolicyTest() {
        health.timestamp = model.timestamp = authority.timestamp = extreme.timestamp = recovery.timestamp = now;
        allocation.timestamp = arbitration.timestamp = shadow.timestamp = now;
        model.valid = model.baseline_learned = model.current_observable = authority.valid = extreme.valid = true;
        model.model_quality = authority.minimum_attitude_authority = 1.f;
        recovery.state = ftc_recovery_status_s::MONITORING;
    }
    ftc_system_status_s status() {
        return FtcSystemPolicy::evaluate(now, enabled, health, model, authority, extreme, recovery, allocation, arbitration, shadow);
    }
};
using S = ftc_system_status_s;
TEST_F(SystemPolicyTest, Disabled) { enabled = false; EXPECT_EQ(status().state, +S::DISABLED); }
TEST_F(SystemPolicyTest, Initializing) { model.timestamp = 0; EXPECT_EQ(status().state, +S::INITIALIZING); }
TEST_F(SystemPolicyTest, Calibrating) { model.valid = model.baseline_learned = false; EXPECT_EQ(status().state, +S::CALIBRATING); }
TEST_F(SystemPolicyTest, Unobservable) { model.current_observable = false; EXPECT_EQ(status().state, +S::UNOBSERVABLE); }
TEST_F(SystemPolicyTest, InvalidCannotBeNormal) { model.valid = false; EXPECT_EQ(status().state, +S::UNOBSERVABLE); }
TEST_F(SystemPolicyTest, Ready) { authority.valid = false; EXPECT_EQ(status().state, +S::READY); }
TEST_F(SystemPolicyTest, NormalWithoutShadow) { EXPECT_EQ(status().state, +S::NORMAL); EXPECT_EQ(status().mode, 1); }
TEST_F(SystemPolicyTest, ShadowNeedsFreshValidCandidate) {
    shadow.valid = true; EXPECT_EQ(status().state, +S::SHADOW); EXPECT_EQ(status().mode, 2);
    shadow.timestamp = 1; EXPECT_EQ(status().state, +S::NORMAL);
}
TEST_F(SystemPolicyTest, Degraded) { health.degraded_mask = 1; EXPECT_EQ(status().state, +S::DEGRADED); }
TEST_F(SystemPolicyTest, FailureOutranksDegradedAuthorityAndImpact) {
    health.failed_mask = 1; authority.state = ftc_control_authority_s::DEGRADED_CONTROL;
    extreme.impact_detected = true; EXPECT_EQ(status().state, +S::FAULT_CONFIRMED);
    EXPECT_TRUE(status().fault_confirmed);
}
TEST_F(SystemPolicyTest, Candidate) { recovery.candidate_valid = true; EXPECT_EQ(status().state, +S::RECOVERY_CANDIDATE); }
TEST_F(SystemPolicyTest, ActiveAllocation) { allocation.active = true; EXPECT_EQ(status().state, +S::ACTIVE_ALLOCATION); }
TEST_F(SystemPolicyTest, ActiveRecovery) { arbitration.active = true; EXPECT_EQ(status().state, +S::RECOVERY_ACTIVE); }
TEST_F(SystemPolicyTest, DualActiveKeepsBothOwnershipFlags) {
    allocation.active = arbitration.active = true;
    EXPECT_TRUE(status().allocation_active); EXPECT_TRUE(status().recovery_active);
    EXPECT_EQ(status().state, +S::RECOVERY_ACTIVE);
}
TEST_F(SystemPolicyTest, EmergencyLand) {
    arbitration.active = true; recovery.state = ftc_recovery_status_s::EMERGENCY_LAND;
    EXPECT_EQ(status().state, +S::EMERGENCY_LAND);
}
TEST_F(SystemPolicyTest, Failed) { recovery.state = ftc_recovery_status_s::FAILED; EXPECT_EQ(status().state, +S::FAILED); }
TEST_F(SystemPolicyTest, DisableDuringBlendStillReportsActualActive) {
    enabled = false; allocation.active = true; EXPECT_EQ(status().state, +S::ACTIVE_ALLOCATION); EXPECT_EQ(status().mode, 4);
}
TEST_F(SystemPolicyTest, StaleAndFutureAreNotActive) {
    allocation.active = arbitration.active = true; allocation.timestamp = 1; arbitration.timestamp = now+1;
    EXPECT_FALSE(status().intervention_enabled); EXPECT_EQ(status().state, +S::NORMAL);
}
