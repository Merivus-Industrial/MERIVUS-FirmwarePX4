/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

/**
 * Enable the fault-tolerant motor health monitor
 *
 * Observation alone does not change actuator outputs. Active allocation also
 * requires FTC_CA_EN and the allocator's runtime safety gates.
 *
 * @boolean
 * @reboot_required true
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_MON_EN, 0);

/**
 * Enable adaptive allocation shadow calculation
 *
 * Shadow mode publishes a candidate output for logging only and never writes
 * to the actuator pipeline.
 *
 * @boolean
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_CA_SHADOW, 0);

/**
 * Enable experimental adaptive control allocation
 *
 * Gated active control path. Default off; SITL/HITL/flight validation pending.
 *
 * @boolean
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_CA_EN, 0);

/**
 * Enable recovery state machine and candidate generation
 *
 * @boolean
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_REC_EN, 0);

/**
 * Enable connection of recovery candidates to the flight-control pipeline
 *
 * Gated active control path. Default off; SITL/HITL/flight validation pending.
 *
 * @boolean
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_REC_ACT, 0);
