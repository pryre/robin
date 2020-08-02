###############################################################################

PROJECT_BUILD_TYPE = breezy_naze32_rev5

###############################################################################

include ./makefiles/common/makefile.project_common

# Naze32 rev5 specific drivers
include $(ROOT)/makefiles/common/makefile.breezy_naze32_common
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/$(PROJECT_BUILD_TYPE)/*.c)

include $(ROOT)/makefiles/common/makefile.build_common

TARGET_IMG = $(TARGET_HEX)

###############################################################################

$(info Building for: $(PROJECT_BUILD_TYPE))
$(info Target: $(TARGET_IMG))

###############################################################################

$(PROJECT_BUILD_TYPE): $(TARGET_IMG)

include $(ROOT)/makefiles/common/makefile.target_common
