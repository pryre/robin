# robin
The ROS Offboard Integration for Naze32-based flight controllers.

## Documentation
Additional documentation on features, interfacing, and packaged tools, [please check here](documents/README.md) (`/documents/README.md`).

## Flashing Pre-Compiled Releases
Pre-compiled releases can be found in [Releases](https://github.com/qutas/robin/releases) section. Make sure to get the correct release revision for your board model.

A binary of `stm32flash` can be found on the [official repository](https://sourceforge.net/projects/stm32flash/). Download and extract `stm32flash.exe` to a folder of your choice.

You can not only pass the version flag in the command line, but you can also pass the serial parameters to use as well:
```sh
make flash SERIAL_DEVICE=/dev/ttyACM0 SERIAL_BAUD=57600 NAZE32_REV=5
```

#### Flashing the Naze32
Before you can flash the the Naze32, you must first put it into bootloader mode. For the initial flash, you must connect the bootloader pins while powering on the device. The location of the boot pins is detailed [here](documents/PINOUT.md).

Depending on your version of `stm32flash`, you may have to call the flash command multiple times before the flashing will actually start. If the flash is not successful, try using a slower baud rate, such as `115200`. Another common problem is that `stm32flash` sometimes misses the initializer message from the autopilot if plugged in via USB, or it gets confused by other data in the serial buffer. The best work around for this is to do the following:
- Unplug and unpower the autopilot completely
- Plug a USB-Serial adaptor into your computer
- Connect the USB-Serial Tx/Rx to the autopilot
- Power on the autopilot directly into bootloader mode

Use the following commands to flash the firmware to your flight controller. You will need to replace the filename (`robin_naze32_revX.hex`) and device name (`/dev/ttyUSBX` or `COMX`) with the correct options for your setup.

#### Flashing (Linux)
```sh
stm32flash -w robin_naze32_revX.hex -v -g 0x0 -b 921600 /dev/ttyUSBX
```

#### Flashing (Windows)
Copy the firmware to the same folder as the one where you extracted `stm32flash.exe`. You can then `Shift`+`Right Click` and select `Open Command Prompt Here`.

In the command prompt, run the following:
```sh
./stm32flash.exe -w robin_naze32_revX.hex -v -g 0x0 -b 921600 COMX
```

## Working with the Source Code

#### Preperation
###### Linux - Ubuntu
```sh
sudo apt install gcc-arm-none-eabi  stm32flash python3-yaml
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

###### Linux - Arch
```sh
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib python-yaml
```
You'll also need to get the `stm32flash` package from the AUR or compile it manually

```sh
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

###### Windows
Compiling the source code is not currently possible on Windows, however it is still possible to flash the precompiled firmware.


#### Compiling
```sh
cd ~/src/robin
make posix_udp
```

To compile for a differennt build target, such as `naze32_rev5` or `naze32_rev6`, simply specify this instead:
```sh
make naze32_rev5
```

You can list all primary build targets with the following command:
```sh
make list_targets
```

#### Flashing using the Makefile
The makefile assumes that the device is connected as `/dev/ttyUSB0` and will use a baud rate of `921600`. You may have to adjust these to suit your device.

Once in bootloader mode run the following command:
```sh
make naze32_rev5_flash SERIAL_DEVICE=/dev/ttyACM0 SERIAL_BAUD=57600
```
After the initial flash, you can use the MAVLINK command `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` to put the device into bootloader mode through the software. Additionally, the following command will attempt to send this directly from the CLI (but requires `pymavlink` to be installed):
```sh
make naze32_rev5_reflash
```
