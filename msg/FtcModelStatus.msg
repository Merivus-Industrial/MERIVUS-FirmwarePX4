# Online propulsion and rigid-body model observation
uint64 timestamp
uint64 timestamp_sample

bool valid
bool effectiveness_valid
bool mass_valid
bool inertia_valid
bool cg_valid

uint8 estimator_type
uint8 motor_count
float32 excitation
float32 model_quality
float32 rotational_residual

uint8 NUM_MOTORS = 12
float32[12] motor_effectiveness
float32[12] motor_confidence

float32 mass
float32[3] inertia
float32[3] cg_offset

uint8 UNINITIALIZED = 0
uint8 CALIBRATING = 1
uint8 BASELINE_LEARNED = 2
uint8 OBSERVABLE = 3
uint8 TEMPORARILY_UNOBSERVABLE = 4
uint8 VALID = 5
uint8 STALE = 6
uint8 INVALID = 7
uint8 state
bool baseline_learned
bool current_observable
bool update_allowed
bool saturated
bool authority_limited
bool timing_aligned
uint64 last_valid_timestamp
uint32 update_count
uint32 reset_count
float32 estimate_age
float32[12] estimate_uncertainty
float32 condition_number
float32 model_prediction_residual
float32 rigid_body_activity
float32[3] predicted_response
float32[3] measured_response
float32 actuator_headroom
float32 control_residual
float32 mass_age
uint8 NOT_IMPLEMENTED = 0
uint8 NOT_OBSERVABLE = 1
uint8 ESTIMATE_AVAILABLE = 2
uint8 mass_state
uint8 inertia_state
uint8 cg_state
