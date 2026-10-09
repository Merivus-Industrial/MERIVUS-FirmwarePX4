# Aggregated FTC subsystem state
uint64 timestamp

uint8 DISABLED = 0
uint8 NORMAL = 1
uint8 DEGRADED = 2
uint8 FAULT_CONFIRMED = 3
uint8 RECOVERY_READY = 4
uint8 RECOVERY_ACTIVE = 5
uint8 EMERGENCY_LAND = 6
uint8 FAILED = 7
uint8 state

bool monitor_enabled
bool model_valid
bool fault_confirmed
bool control_authority_valid
bool recovery_eligible
bool intervention_enabled
uint16 degraded_motor_mask
uint16 failed_motor_mask
uint32 reason_mask
float32 system_confidence

uint8 INITIALIZING = 8
uint8 CALIBRATING = 9
uint8 READY = 10
uint8 UNOBSERVABLE = 11
uint8 SHADOW = 12
uint8 RECOVERY_CANDIDATE = 13
uint8 ACTIVE_ALLOCATION = 14
uint8 mode
bool allocation_active
bool recovery_active
uint32 allocation_fallback
uint32 recovery_fallback
