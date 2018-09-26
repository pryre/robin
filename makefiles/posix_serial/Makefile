###############################################################################

PROJECT_BUILD_TYPE = posix_serial

.DEFAULT_GOAL: posix_serial

###############################################################################

include ../common/makefile.project_common

# POSIX serial specific drivers
include ../common/makefile.posix_common
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/posix_serial/*.c)

include ../common/makefile.build_common

###############################################################################

$(info Building for: $(PROJECT_BUILD_TYPE))
$(info Target: $(TARGET))

###############################################################################

TARGET_IMG = $(TARGET_ELF)
posix_serial: $(TARGET_IMG)
