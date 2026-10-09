# Read-only snapshot of the control allocator effectiveness contract
uint64 timestamp

uint8 matrix_index
uint8 num_axes
uint8 num_actuators
uint8 num_motors
bool valid
bool normalize_rpy

uint8 NUM_AXES = 6
uint8 NUM_ACTUATORS = 16
uint16 MATRIX_SIZE = 96
float32[96] effectiveness
float32[16] trim
float32[16] linearization_point
float32[16] minimum
float32[16] maximum
