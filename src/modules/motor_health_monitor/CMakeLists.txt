############################################################################
#
# Copyright (c) 2026 Merivus Industrial. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__motor_health_monitor
	MAIN motor_health_monitor
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		EffectivenessEstimator.hpp
		MotorEffectivenessEstimator.cpp
		MotorEffectivenessEstimator.hpp
		MotorHealthMonitor.cpp
		MotorHealthMonitor.hpp
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(
	SRC
		MotorEffectivenessEstimatorTest.cpp
	EXTRA_SRCS
		MotorEffectivenessEstimator.cpp
)
