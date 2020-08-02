###############################################################################

PROJECT_BUILD_TYPE = posix_udp

###############################################################################

include ./makefiles/common/makefile.project_common

# POSIX serial specific drivers
include $(ROOT)/makefiles/common/makefile.posix_common
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/$(PROJECT_BUILD_TYPE)/*.c)

include $(ROOT)/makefiles/common/makefile.build_common

TARGET_IMG = $(TARGET_ELF)

###############################################################################

$(info Building for: $(PROJECT_BUILD_TYPE))
$(info Target: $(TARGET_IMG))
#$(info Device flags: $(DEVICE_FLAGS))

###############################################################################

$(PROJECT_BUILD_TYPE): $(TARGET_IMG)

include $(ROOT)/makefiles/common/makefile.target_common
