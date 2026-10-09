############################################################################
#
# Copyright (c) 2026 Merivus Industrial. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__ftc_supervisor
	MAIN ftc_supervisor
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		FtcSupervisor.cpp
		FtcSupervisor.hpp
	DEPENDS
		px4_work_queue
)
