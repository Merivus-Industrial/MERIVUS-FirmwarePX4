# Remaining control authority derived from the dynamic effectiveness matrix
uint64 timestamp

uint8 FULL_CONTROL = 0
uint8 DEGRADED_CONTROL = 1
uint8 YAW_UNCONTROLLABLE = 2
uint8 ATTITUDE_DEGRADED = 3
uint8 THRUST_INSUFFICIENT = 4
uint8 RECOVERY_ONLY = 5
uint8 UNCONTROLLABLE = 6
uint8 state

bool valid
bool matrix_valid
uint8 motor_count

# Reachable control increments around current command in nominal matrix units
float32[3] positive_authority
float32[3] negative_authority
float32 thrust_up
float32 thrust_down
float32 reachable_residual
uint16 saturated_mask

float32 roll_authority
float32 pitch_authority
float32 yaw_authority
float32 thrust_authority
float32 minimum_attitude_authority
float32 actuator_headroom
