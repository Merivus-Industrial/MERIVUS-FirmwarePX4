############################################################################
#
# Copyright (c) 2026 Merivus Industrial. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__ftc_control_monitor
	MAIN ftc_control_monitor
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		FtcControlMonitor.cpp
		FtcControlMonitor.hpp
	INCLUDES
		${PX4_SOURCE_DIR}/src/modules/control_allocator
	DEPENDS
		ControlAllocation
		px4_work_queue
)
