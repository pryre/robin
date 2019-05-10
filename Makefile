###############################################################################

PROJECT_NAME = robin

###############################################################################

SERIAL_DEVICE	?= /dev/ttyUSB0
SERIAL_BAUD		?= 921600
MAVLINK_BAUD	?= 115200
SERIAL_DEVICE_2 ?=

TARGET_HEX =
TARGET_FOLDERS = $(shell ls makefiles | grep -v common)
TARGET_MODIFIERS = "[TARGET]_flash [TARGET]_reflash [TARGET]_run"
TARGET_OTHERS = "all clean lint param_gen mavlink_bootloader listen"

# Debugger optons, must be empty or GDB
DEBUG		?=

###############################################################################
# Default Target

list_targets:
	@echo "Available build targets:"
	@echo "$(TARGET_FOLDERS)" | tr ' ' '\n' | sed -e "s/^/\t/"
	@echo ""
	@echo "Available modifiers (target specific):"
	@echo $(TARGET_MODIFIERS) | tr ' ' '\n' | sed -e "s/^/\t/"
	@echo ""
	@echo "Other targets:"
	@echo "$(TARGET_OTHERS)" | tr ' ' '\n' | sed -e "s/^/\t/"

###############################################################################
# Tool Targets

all: $(TARGET_FOLDERS)

param_gen:
	@python3 lib/param_generator/gen_params.py ./lib/param_generator/definitions/ ./build/ >&2

sleep:
	@sleep 1

lint:
	@find ./include -name '*.h' -exec clang-format -i '{}' \;
	@find ./src -name '*.c' -exec clang-format -i '{}' \;

clean:
	@echo "Cleaning build files"
	@rm -rf ./build
	@cd lib/libopencm3 && $(MAKE) clean -j
	@echo "Preparing build directory"
	@mkdir ./build

mavlink_bootloader:
	./lib/scripts/reboot_bootloader --device $(SERIAL_DEVICE) --baudrate $(MAVLINK_BAUD)

listen:
	stty -F $(SERIAL_DEVICE) raw speed $(SERIAL_BAUD) -crtscts cs8 -parenb -cstopb -ixon
	od -x < $(SERIAL_DEVICE)

###############################################################################
### Build Targets

# POSIX UDP
posix_udp: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

posix_udp_run: posix_udp
	@exec ./lib/scripts/sitl_udp_run.sh

# POSIX Serial
posix_serial: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

posix_serial_run: posix_serial
	@exec ./lib/scripts/sitl_serial_run.sh $(SERIAL_DEVICE) $(SERIAL_DEVICE_2)

# Naze32 Rev.5 (Breezy)
naze32_rev5: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

naze32_rev5_flash: naze32_rev5
	stm32flash -w build/robin_naze32_rev5.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

naze32_rev5_reflash: naze32_rev5 mavlink_bootloader sleep naze32_rev5_flash

# Naze32 Rev.6 (Breezy)
naze32_rev6: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

naze32_rev6_flash: naze32_rev6
	stm32flash -w build/robin_naze32_rev6.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

naze32_rev6_reflash: naze32_rev6 mavlink_bootloader sleep naze32_rev6_flash

# Naze32 Rev.5 (LibOpenCM3)
opencm3_naze32_rev5: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

opencm3_naze32_rev5_flash: opencm3_naze32_rev5
	stm32flash -w build/robin_opencm3_naze32_rev5.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

opencm3_naze32_rev5_reflash: opencm3_naze32_rev5 mavlink_bootloader sleep opencm3_naze32_rev5_flash

# Naze32 Rev.5 Mini (LibOpenCM3)
opencm3_naze32_rev5_mini: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

opencm3_naze32_rev5_mini_flash: opencm3_naze32_rev5_mini
	stm32flash -w build/robin_opencm3_naze32_rev5_mini.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

opencm3_naze32_rev5_mini_reflash: opencm3_naze32_rev5_mini mavlink_bootloader sleep opencm3_naze32_rev5_mini_flash

# Naze32 Rev.6 Mini (LibOpenCM3)
opencm3_naze32_rev6: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

opencm3_naze32_rev6_flash: opencm3_naze32_rev6
	stm32flash -w build/robin_opencm3_naze32_rev6.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

opencm3_naze32_rev6_reflash: opencm3_naze32_rev6 mavlink_bootloader sleep opencm3_naze32_rev6_flash
