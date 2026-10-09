# Continuous motor effectiveness and health observation
uint64 timestamp
uint64 timestamp_sample

uint8 STATE_DISABLED = 0
uint8 STATE_CALIBRATING = 1
uint8 STATE_VALID = 2
uint8 STATE_INVALID = 3
uint8 state

uint8 motor_count
bool model_valid
bool esc_data_available
bool imu_only

uint8 FAULT_NONE = 0
uint8 FAULT_UNKNOWN_PROPULSION_DEGRADATION = 1
uint8 FAULT_PROP_DAMAGE = 2
uint8 FAULT_MOTOR_DEGRADATION = 3
uint8 FAULT_MOTOR_STOP = 4
uint8 FAULT_ESC_OR_POWER_FAILURE = 5
uint8 FAULT_INTERMITTENT_PROPULSION_FAILURE = 6
uint8 FAULT_MECHANICAL_IMBALANCE = 7
uint8 FAULT_EXTERNAL_DISTURBANCE = 8
uint8 FAULT_MODEL_MISMATCH = 9

uint16 degraded_mask
uint16 failed_mask

float32 model_residual
float32 excitation
float32 maneuver_intensity
float32 external_disturbance_score
float32 vibration_score

uint8 NUM_MOTORS = 12
float32[12] effectiveness
float32[12] health
float32[12] confidence
float32[12] residual
float32[12] fault_probability
float32[12] fault_confidence
float32[12] fault_persistence
uint8[12] fault_type

uint8 NO_EVIDENCE = 0
uint8 UNKNOWN = 1
uint8 UNOBSERVABLE = 2
uint8 VALID_HEALTHY = 3
uint8 DEGRADED = 4
uint8 FAILED = 5
uint8[12] diagnosis_state
