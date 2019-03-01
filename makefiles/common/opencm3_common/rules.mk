###############################################################################

DEVICE_BUILD_TOOL = arm-none-eabi-gcc
DEVICE_BUILD_OBJCOPY = arm-none-eabi-objcopy

###############################################################################

# Naze32 libopencm3 specific drivers
OPENCM3_DIR	:= $(ROOT)/lib/libopencm3
LDLIBS		+= -l$(LIBNAME)

###############################################################################
# New style, assume device is provided, and we're generating the rest.
ifeq ($(strip $(DEVICE)),)
# Old style, assume LDSCRIPT exists
#DEFS		+= -I$(OPENCM3_DIR)/include
#LDFLAGS		+= -L$(OPENCM3_DIR)/lib
#LDLIBS		+= -l$(LIBNAME)
#LDSCRIPT	?= $(BINARY).ld
else
# New style, assume device is provided, and we're generating the rest.
ifneq ($(strip $(LDSCRIPT)),)
$(error $(ERR_DEVICE_LDSCRIPT_CONFLICT))
endif
include $(OPENCM3_DIR)/mk/genlink-config.mk
endif

OPENCM3_SCRIPT_DIR = $(OPENCM3_DIR)/scripts
EXAMPLES_SCRIPT_DIR	= $(OPENCM3_DIR)/../scripts

# C flags
TGT_CFLAGS	+= $(OPT) $(CSTD) $(DEBUG_FLAGS)
TGT_CFLAGS	+= $(ARCH_FLAGS)
TGT_CFLAGS	+= -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS	+= -ffunction-sections -fdata-sections

# C++ flags
TGT_CXXFLAGS	+= $(OPT) $(CXXSTD) $(DEBUG_FLAGS)
TGT_CXXFLAGS	+= $(ARCH_FLAGS)
TGT_CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
TGT_CXXFLAGS	+= -ffunction-sections -fdata-sections

# C & C++ preprocessor common flags
TGT_CPPFLAGS	+= -MD
TGT_CPPFLAGS	+= -Wall -Wundef
TGT_CPPFLAGS	+= $(DEFS)

# Linker flags
TGT_LDFLAGS		+= --static -nostartfiles
TGT_LDFLAGS		+= -T$(LDSCRIPT)
TGT_LDFLAGS		+= $(ARCH_FLAGS) $(DEBUG_FLAGS)
TGT_LDFLAGS		+= -Wl,-Map=$(*).map -Wl,--cref
TGT_LDFLAGS		+= -Wl,--gc-sections
ifeq ($(V),99)
TGT_LDFLAGS		+= -Wl,--print-gc-sections
endif

###############################################################################

# Used libraries
LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# Generate ld script
#include $(OPENCM3_DIR)/mk/genlink-rules.mk

###############################################################################

DEVICE_SRC :=

DEVICE_INCLUDES := $(OPENCM3_DIR)/include

DEVICE_CFLAGS := $(TGT_CFLAGS)

DEVICE_LDFLAGS := -lm \
				  -nostartfiles \
				  -lc \
				  -static \
				  -Wl,-gc-sections,-Map,$(TARGET_MAP) \
				  -T$(LDSCRIPT) \
				  -L$(OBJECT_DIR)/lib/libopencm3 \
				  $(LDLIBS)
				  #--specs=rdimon.specs \
