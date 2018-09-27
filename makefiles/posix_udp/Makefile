###############################################################################

PROJECT_BUILD_TYPE = posix_udp

###############################################################################

include ../common/makefile.project_common

# POSIX serial specific drivers
include ../common/makefile.posix_common
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/$(PROJECT_BUILD_TYPE)/*.c)

include ../common/makefile.build_common

TARGET_IMG = $(TARGET_ELF)

###############################################################################

$(info Building for: $(PROJECT_BUILD_TYPE))
$(info Target: $(TARGET_IMG))

###############################################################################

$(PROJECT_BUILD_TYPE): $(TARGET_IMG)

include ../common/makefile.target_common
