# Makefile for BreezySTM32F1 examples
#
# Copyright (C) 2016 Simon D. Levy
#
# This file is part of BreezySTM32.
#
# BreezySTM32 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this code.  If not, see <http://www.gnu.org/licenses/>.

###############################################################################
BUILD_TYPE		?= posix

SERIAL_DEVICE	?= /dev/ttyUSB0
SERIAL_BAUD		?= 921600
SERIAL_DEVICE_2 ?=
###############################################################################

ARCH_FLAGS	= -DFIXMATH_NO_CACHE

# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s

# Working directories
ROOT		 = ..
HERE         = .
SRC_DIR		 = $(ROOT)
OBJECT_DIR	 = $(HERE)/build
BIN_DIR		 = $(HERE)/build

TARGET		?= $(PROJECT_NAME)_$(BUILD_TYPE)

TARGET_BIN	 = $(BIN_DIR)/$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP   = $(OBJECT_DIR)/$(TARGET).map

include makefiles/makefile.common

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG		?=

###############################################################################
# Things that need to be maintained as the source changes
#

# Source files common to all targets
$(TARGET)_SRC = $(PROJECT_SRC_FILES) \
			    $(PROJECT_SRC_PARAM_GEN) \
			    $(PROJECT_SRC_LIBFIXMATH) \
			    $(PROJECT_SRC_LIBFIXMATRIX) \
			    $(DEVICE_SRC)

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = $(DEVICE_BUILD_TOOL)
OBJCOPY	 = $(DEVICE_BUILD_OBJCOPY)

#
# Tool options.
#
INCLUDE_DIRS = ./include \
			   ./lib \
			   $(PROJECT_DIR_LIBFIXMATH) \
			   $(PROJECT_DIR_LIBFIXMATRIX) \
			   $(DEVICE_INCLUDES)

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -Og
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3

#XXX: (-Wno-char-subscripts -Wno-sign-compare) used to disable warnings
#	  in the libfixmath and libfixmatrix compilations
#XXX: (-Wno-unused-parameter) used to disable warnings
#	  for driver / generic function parameters
CFLAGS	 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(DEBUG_FLAGS) \
		   $(DEVICE_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Wall -pedantic -Wextra -Wshadow -Wunsafe-loop-optimizations \
		   -Wno-char-subscripts -Wno-sign-compare -Wno-unused-parameter \
		   -ffunction-sections \
		   -fdata-sections \
		   -D$(TARGET) \
		   -DGIT_VERSION_FLIGHT_STR=\"$(GIT_VERSION_FLIGHT)\" \
		   -DGIT_VERSION_OS_STR=\"$(GIT_VERSION_OS)\" \
		   -DGIT_VERSION_MAVLINK_STR=\"$(GIT_VERSION_MAVLINK)\" \
		   -DEEPROM_CONF_VERSION_STR=\"$(EEPROM_CONF_VERSION)\"

ASFLAGS	= $(ARCH_FLAGS) \
		  -x assembler-with-cpp \
		  $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LDFLAGS	= $(ARCH_FLAGS) \
		  $(LTO_FLAGS) \
		  $(DEBUG_FLAGS) \
		  $(DEVICE_LDFLAGS)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we can build
#
TARGET_BUILD_ID ?= hex
$(info Build target: $(TARGET_BUILD_ID))

ifeq ($(findstring bin,$(TARGET_BUILD_ID)),bin)
	TARGET_IMG = $(TARGET_BIN)
endif
ifeq ($(findstring hex,$(TARGET_BUILD_ID)),hex)
	TARGET_IMG = $(TARGET_HEX)
endif
ifeq ($(findstring elf,$(TARGET_BUILD_ID)),elf)
	TARGET_IMG = $(TARGET_ELF)
endif

# Set up for the default build target
.DEFAULT_GOAL := $(TARGET_IMG)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.
$(TARGET_BIN): $(TARGET_ELF)
		$(V0) $(OBJCOPY) -O binary $< $@

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

MKDIR_OBJDIR = @mkdir -p $(dir $@)

# Compile C
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Compile C++
$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

clean:
#	rm -rf *.o obj $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf ./build
	rm ./lib/param_generator/param_gen.h
	rm ./lib/param_generator/param_gen.c
	rm ./lib/param_generator/PARAMS.md

flash: flash_$(TARGET)

flash_$(TARGET): $(TARGET_IMG)
#	stty -F $(SERIAL_DEVICE) raw speed $(SERIAL_BAUD) -crtscts cs8 -parenb -cstopb -ixon
#	echo -n 'R' >$(SERIAL_DEVICE)
#	sleep 1
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

mavlink_bootloader:
	./lib/scripts/reboot_bootloader --device $(SERIAL_DEVICE) --baudrate $(SERIAL_BAUD)
	sleep 1

reflash: reflash_$(TARGET)

reflash_$(TARGET): $(TARGET_IMG) mavlink_bootloader flash_$(TARGET)

run: run_$(TARGET)

run_$(TARGET): $(TARGET_IMG)
	@exec ./lib/scripts/sitl_run.sh $(TARGET_IMG) $(SERIAL_DEVICE) $(SERIAL_DEVICE_2)

listen:
	#picocom $(SERIAL_DEVICE) -b $(SERIAL_BAUD)
	stty -F $(SERIAL_DEVICE) raw speed $(SERIAL_BAUD) -crtscts cs8 -parenb -cstopb -ixon
	od -x < $(SERIAL_DEVICE)
