############################################################################
#
# Copyright (c) 2026 Merivus Industrial. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__ftc_recovery
	MAIN ftc_recovery
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		FtcRecovery.cpp
		FtcRecovery.hpp
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(SRC FtcControlContractTest.cpp)
