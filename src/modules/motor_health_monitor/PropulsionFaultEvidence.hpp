#pragma once
#include <uORB/topics/motor_health_status.h>

// Effectiveness and vehicle-wide vibration cannot identify a damaged physical component.
inline uint8_t classifyPropulsionFault(bool esc_fault, bool motor_stopped, bool intermittent)
{
    if (esc_fault) { return motor_health_status_s::FAULT_ESC_OR_POWER_FAILURE; }
    if (motor_stopped) { return motor_health_status_s::FAULT_MOTOR_STOP; }
    if (intermittent) { return motor_health_status_s::FAULT_INTERMITTENT_PROPULSION_FAILURE; }
    return motor_health_status_s::FAULT_UNKNOWN_PROPULSION_DEGRADATION;
}
