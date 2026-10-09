# Recovery manager state and disconnected candidate setpoint
uint64 timestamp

uint8 DISABLED = 0
uint8 MONITORING = 1
uint8 DISTURBANCE_DETECTED = 2
uint8 RATE_DAMPING = 3
uint8 THRUST_VECTOR_RECOVERY = 4
uint8 ATTITUDE_RECOVERY = 5
uint8 ALTITUDE_STABILIZATION = 6
uint8 CONTROL_REENTRY = 7
uint8 EMERGENCY_LAND = 8
uint8 ABORTED = 9
uint8 FAILED = 10
uint8 state

bool eligible
bool intervention_enabled
bool active
bool candidate_valid
uint32 trigger_mask
uint32 inhibit_reason_mask
float32 progress
float32 state_elapsed

float32[3] body_rate_setpoint
float32[3] thrust_body
float32[4] attitude_setpoint_q
bool yaw_sacrifice_requested

uint8 VERTICAL_SPEED_RECOVERY = 11
uint8 original_nav_state
float32 vertical_speed_setpoint
float32 altitude_reference
float32 reentry_weight
bool reentry_ready
uint32 fallback_reason
