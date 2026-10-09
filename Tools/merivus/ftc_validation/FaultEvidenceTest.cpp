#include "src/modules/motor_health_monitor/PropulsionFaultEvidence.hpp"
#include <gtest/gtest.h>
using H = motor_health_status_s;
TEST(FaultEvidence, EffectivenessAloneDoesNotAttributePhysicalDamage)
{
    EXPECT_EQ(classifyPropulsionFault(false, false, false), +H::FAULT_UNKNOWN_PROPULSION_DEGRADATION);
}
TEST(FaultEvidence, StoppedMotorNeedsRotorOrEscEvidence)
{
    EXPECT_EQ(classifyPropulsionFault(false, true, false), +H::FAULT_MOTOR_STOP);
}
TEST(FaultEvidence, EscFailureHasPriorityOverIntermittentEstimate)
{
    EXPECT_EQ(classifyPropulsionFault(true, true, true), +H::FAULT_ESC_OR_POWER_FAILURE);
}
TEST(FaultEvidence, TimeVaryingPropulsionEvidenceDoesNotIdentifyAnEsc)
{
    EXPECT_EQ(classifyPropulsionFault(false, false, true), +H::FAULT_INTERMITTENT_PROPULSION_FAILURE);
}
