/****************************************************************************
 *
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 *
 ****************************************************************************/

/**
 * Minimum mean motor command used for estimation
 *
 * @unit norm
 * @min 0.05
 * @max 0.8
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_MIN_THR, 0.15f);

/**
 * Input and angular acceleration low-pass time constant
 *
 * @unit s
 * @min 0.02
 * @max 2.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_LPF_TC, 0.20f);

/**
 * Minimum per-motor command excitation standard deviation
 *
 * @unit norm
 * @min 0.005
 * @max 0.3
 * @decimal 3
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EXC_MIN, 0.025f);

/**
 * Healthy baseline learning duration with sufficient excitation
 *
 * @unit s
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_BASE_T, 5.0f);

/**
 * Maximum effectiveness estimate change rate
 *
 * @unit 1/s
 * @min 0.02
 * @max 2.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_RATE, 0.30f);

/**
 * Maximum normalized model residual for a valid estimate
 *
 * @min 0.05
 * @max 5.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_RES_THR, 1.0f);

/**
 * Health threshold for degraded classification
 *
 * @min 0.1
 * @max 0.95
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_HLTH_MIN, 0.70f);

/**
 * Health threshold for failed classification
 *
 * @min 0.0
 * @max 0.7
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_FAIL_MIN, 0.25f);

/**
 * Minimum estimator confidence for fault classification
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_CONF_MIN, 0.60f);

/**
 * Fault threshold persistence time
 *
 * @unit s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_FAIL_T, 1.0f);

/**
 * Estimator forgetting factor
 *
 * @min 0.90
 * @max 1.0
 * @decimal 3
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_FORG, 0.995f);

/**
 * Lower bound for estimated motor effectiveness
 *
 * @min 0.05
 * @max 0.8
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_LMIN, 0.10f);

/**
 * Nominal roll inertia approximation
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 100
 * @decimal 4
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_IXX, 0.02f);

/**
 * Nominal pitch inertia approximation
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 100
 * @decimal 4
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_IYY, 0.02f);

/**
 * Nominal yaw inertia approximation
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 100
 * @decimal 4
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_IZZ, 0.04f);

/**
 * Fault probability threshold
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_FAULT_P, 0.65f);

/**
 * Mechanical imbalance vibration threshold
 *
 * @unit m/s^2
 * @min 0.1
 * @max 100
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_FAULT_VIB, 8.0f);

/**
 * External disturbance score threshold
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_FAULT_EXT, 0.70f);

/**
 * Minimum normalized attitude authority
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_CA_ATT_MIN, 0.35f);

/**
 * Minimum normalized yaw authority
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_CA_YAW_MIN, 0.20f);

/**
 * Minimum normalized thrust authority
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_CA_THR_MIN, 0.25f);

/**
 * Enable impact and hard-landing observation
 *
 * @boolean
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_IMPACT_EN, 0);

/**
 * Impact acceleration threshold
 *
 * @unit m/s^2
 * @min 5
 * @max 200
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_IMPACT_ACC, 30.0f);

/**
 * Impact jerk threshold
 *
 * @unit m/s^3
 * @min 10
 * @max 1000
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_IMPACT_JRK, 120.0f);

/**
 * Enable loss-of-control observation
 *
 * @boolean
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_INT32(FTC_LOC_EN, 0);

/**
 * Loss-of-control score threshold
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_LOC_THR, 0.70f);

/**
 * Recovery maximum commanded body rate
 *
 * @unit rad/s
 * @min 0.1
 * @max 10
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_REC_RATE, 2.0f);

/**
 * Recovery rate damping gain
 *
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_REC_KD, 0.8f);

/**
 * Recovery minimum altitude above ground estimate
 *
 * @unit m
 * @min 0
 * @max 100
 * @decimal 1
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_REC_ALT, 3.0f);

/**
 * Command to angular-response delay
 *
 * Align actuator publication time with IMU sample time. Calibrate per airframe.
 * @unit s
 * @min 0
 * @max 0.4
 * @decimal 3
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_DELAY, 0.04f);

/**
 * Maximum age of effectiveness parameter update
 * @unit s
 * @min 0.2
 * @max 30
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_EST_AGE, 10.f);

/**
 * Calibrated total thrust at full command, mass observation only
 *
 * Zero means uncalibrated; mass remains unavailable. Assumes equal motors
 * and a linear command-to-thrust contract. Never used for active control.
 * @unit N
 * @min 0
 * @max 1000
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_THR_MAX, 0.f);
