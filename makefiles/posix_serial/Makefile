###############################################################################

PROJECT_BUILD_TYPE = posix_serial

.DEFAULT_GOAL: $(PROJECT_BUILD_TYPE)

###############################################################################

include ../common/makefile.project_common
include ../common/makefile.posix_common
include ../common/makefile.build_common

# POSIX serial specific drivers
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/posix_serial/*.c)

TARGET_IMG = $(TARGET_ELF)

$(info Building for: $(PROJECT_BUILD_TYPE))
$(info Target: $(TARGET))

###############################################################################

posix_serial: $(TARGET_IMG)
