############################################################################
#
# Copyright (c) 2026 Merivus Industrial. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__ftc_extreme_state_monitor
	MAIN ftc_extreme_state_monitor
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		FtcExtremeStateMonitor.cpp
		FtcExtremeStateMonitor.hpp
	DEPENDS
		px4_work_queue
)
