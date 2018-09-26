

###############################################################################

PROJECT_NAME = robin

###############################################################################

SERIAL_DEVICE	?= /dev/ttyUSB0
SERIAL_BAUD		?= 921600
SERIAL_DEVICE_2 ?=

###############################################################################

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG		?=

posix_serial:
	$(MAKE) -C makefiles/posix_serial PROJECT_NAME=$(PROJECT_NAME)

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
