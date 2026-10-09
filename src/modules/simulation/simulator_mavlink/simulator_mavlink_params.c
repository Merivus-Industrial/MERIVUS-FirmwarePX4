/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

/**
 * Enable SITL motor effectiveness injection
 *
 * This parameter is implemented only by the POSIX MAVLink simulator bridge.
 * It never changes hardware actuator outputs.
 *
 * @boolean
 * @group Fault Tolerant Control Simulation
 */
PARAM_DEFINE_INT32(FTC_SIM_EN, 0);

/**
 * SITL motor selected for effectiveness injection
 *
 * Motor numbering is one based.
 *
 * @min 1
 * @max 12
 * @group Fault Tolerant Control Simulation
 */
PARAM_DEFINE_INT32(FTC_SIM_MOT, 1);

/**
 * SITL target motor effectiveness
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control Simulation
 */
PARAM_DEFINE_FLOAT(FTC_SIM_EFF, 1.0f);

/**
 * SITL effectiveness ramp duration for a full-scale change
 *
 * Zero applies the target immediately.
 *
 * @unit s
 * @min 0.0
 * @max 120.0
 * @decimal 1
 * @group Fault Tolerant Control Simulation
 */
PARAM_DEFINE_FLOAT(FTC_SIM_RAMP, 0.0f);

/**
 * SITL intermittent failure period
 *
 * Zero disables intermittency. A positive value alternates between the target
 * effectiveness and nominal effectiveness for half a period each.
 *
 * @unit s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @group Fault Tolerant Control Simulation
 */
PARAM_DEFINE_FLOAT(FTC_SIM_INT, 0.0f);
