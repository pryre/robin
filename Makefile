

###############################################################################

PROJECT_NAME = robin

###############################################################################

SERIAL_DEVICE	?= /dev/ttyUSB0
SERIAL_BAUD		?= 921600
SERIAL_DEVICE_2 ?=

TARGET_HEX =

###############################################################################

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG		?=

posix_udp: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

posix_udp_run: posix_udp
	@exec ./lib/scripts/sitl_udp_run.sh

posix_serial: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

posix_serial_run: posix_serial
	@exec ./lib/scripts/sitl_serial_run.sh $(SERIAL_DEVICE) $(SERIAL_DEVICE_2)

naze32_rev5: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

naze32_rev5_flash: naze32_rev5
	stm32flash -w build/robin_naze32_rev5.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

naze32_rev5_reflash: naze32_rev5 mavlink_bootloader
	@sleep 1
	stm32flash -w build/robin_naze32_rev5.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

naze32_rev6: param_gen
	$(MAKE) -C makefiles/$@ PROJECT_NAME=$(PROJECT_NAME)

naze32_rev6_flash: naze32_rev6
	stm32flash -w build/robin_naze32_rev6.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

naze32_rev6_reflash: naze32_rev6 mavlink_bootloader
	@sleep 1
	stm32flash -w build/robin_naze32_rev6.hex -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

param_gen:
	@python3 lib/param_generator/gen_params.py ./lib/param_generator/definitions/ ./build/ >&2

lint:
	@find ./include -name '*.h' -exec clang-format -i '{}' \;
	@find ./src -name '*.c' -exec clang-format -i '{}' \;

clean:
	@rm -rf ./build
	@mkdir ./build

#flash:
#	stty -F $(SERIAL_DEVICE) raw speed $(SERIAL_BAUD) -crtscts cs8 -parenb -cstopb -ixon
#	echo -n 'R' >$(SERIAL_DEVICE)
#	sleep 1
#	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b $(SERIAL_BAUD) $(SERIAL_DEVICE)

mavlink_bootloader:
	./lib/scripts/reboot_bootloader --device $(SERIAL_DEVICE) --baudrate $(SERIAL_BAUD)

#reflash: mavlink_bootloader flash

listen:
	#picocom $(SERIAL_DEVICE) -b $(SERIAL_BAUD)
	stty -F $(SERIAL_DEVICE) raw speed $(SERIAL_BAUD) -crtscts cs8 -parenb -cstopb -ixon
	od -x < $(SERIAL_DEVICE)
