###############################################################################

PROJECT_BUILD_TYPE = opencm3_naze32_rev6

###############################################################################

include ./makefiles/common/makefile.project_common

# OpenCM3 specific drivers
include $(ROOT)/makefiles/common/opencm3_common/stm32_f1_common.mk
include $(ROOT)/makefiles/common/opencm3_common/rules.mk
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/opencm3_naze32_common/*.c)
PROJECT_SRC_FILES += $(wildcard $(ROOT)/src/drivers/$(PROJECT_BUILD_TYPE)/*.c)

include $(ROOT)/makefiles/common/makefile.build_common

TARGET_IMG = $(TARGET_HEX)

###############################################################################

$(info Building for: $(PROJECT_BUILD_TYPE))
$(info Target: $(TARGET_IMG))

###############################################################################

$(PROJECT_BUILD_TYPE): $(TARGET_IMG)

include $(ROOT)/makefiles/common/makefile.target_common
