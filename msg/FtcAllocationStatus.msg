# Applied matrix policy, published by the sole actual control allocator
uint64 timestamp
uint64 model_timestamp
uint8 NOMINAL = 0
uint8 WAITING = 1
uint8 BLENDING = 2
uint8 ACTIVE = 3
uint8 FALLBACK = 4
uint8 state
uint32 fallback_reason
bool enabled
bool active
bool supported
float32 yaw_weight
float32[12] applied_effectiveness
