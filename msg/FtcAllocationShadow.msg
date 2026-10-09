# Candidate actuator compensation calculated for observation only
uint64 timestamp
uint64 timestamp_sample

uint8 motor_count
bool valid
uint16 saturated_mask
bool yaw_sacrificed

uint8 NUM_MOTORS = 12
float32[12] nominal
float32[12] candidate
float32[12] effectiveness

float32[3] residual_torque
float32[3] residual_thrust
float32 residual_norm
