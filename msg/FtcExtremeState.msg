# Impact, hard-landing and loss-of-control observation
uint64 timestamp
uint64 event_timestamp

uint8 IMPACT_NONE = 0
uint8 IMPACT_EXTERNAL = 1
uint8 IMPACT_HARD_LANDING = 2
uint8 impact_type

uint8 LOC_NORMAL = 0
uint8 LOC_DISTURBED = 1
uint8 LOC_RECOVERY_RECOMMENDED = 2
uint8 LOC_LOSS_OF_CONTROL = 3
uint8 LOC_UNRECOVERABLE = 4
uint8 loc_state

bool valid
bool impact_detected
bool hard_landing
float32 impact_score
float32 impact_confidence
float32 impact_severity
float32[3] impact_axis

float32 loss_of_control_score
uint32 loss_of_control_reason_mask
float32 attitude_error
float32 rate_error
float32 jerk
float32 acceleration_magnitude
float32 angular_rate
float32 angular_acceleration
