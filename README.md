# robin
The ROS Offboard Integration for Naze32-based flight controllers.

## Preperation
#### Linux - Ubuntu
```sh
sudo apt install gcc-arm-none-eabi  stm32flash python3-yaml
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

#### Linux - Arch
```sh
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib python-yaml
```
You'll also need to get the `stm32flash` package from the AUR or compile it manually

```sh
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

#### Windows
Compiling the source code is not currently possible on Windows, however it is still possible to flash the precompiled firmware.

A windows binary of `stm32flash` can be found on the [official repository](https://sourceforge.net/projects/stm32flash/). Download and extract `stm32flash.exe` to a folder of your choice.

## Flashing Pre-Compiled Releases
Pre-compiled releases can be found in [Releases](https://github.com/qutas/robin/releases) section. Make sure to get the correct release revision for your board model.

Use the following command to flash the firmware to your flight controller. You will need to replace the filename (`robin_naze32_rX.hex`) and device name (`/dev/ttyUSBX` or `COMX`) with the correct options for your setup.

#### Manually Flashing (Linux)
```sh
stm32flash -w robin_naze32_rX.hex -v -g 0x0 -b 921600 /dev/ttyUSBX
```

#### Manually Flashing (Windows)
Copy the firmware to the same folder as the one where you extracted `stm32flash.exe`. You can then `Shift`+`Right Click` and select `Open Command Prompt Here`.

In the command prompt, run the following:
```sh
stm32flash.exe -w robin_naze32_rX.hex -v -g 0x0 -b 921600 COMX
```

## Documentation
Additional documentation on features, interfacing, and packaged tools, [please check here](documents/README.md) (`/documents/README.md`).

## Compiling from Source
```sh
cd ~/src/robin
make
```

#### Version Options
By default, robin will compile for the Naze32 Rev6. This can be overridden with the `NAZE32_REV` flag for other devices, such as the Naze32 Rev5 or the Naze32 Mini Rev3 (both of these use Rev5):
```sh
make NAZE32_REV=5
```

If you are constantly reflashing the firmware, you can adjust the default on the first few lines of the `makefile`.

#### Flashing

---
**Note:** Some Naze32 models do not allow for flashing to be done via USB, and need to be connected to the UART1 port directly

---

Before you can flash the the Naze32, you must first put it into bootloader mode. For the initial flash, you can short out the bootloader pins and power on the device.

The makefile assumes that the device is connected as `/dev/ttyUSB0` and will use a baud rate of `921600`. You may have to adjust these to suit your device. If the flash is not successful, try using a slower baud rate, such as `115200`.

Once in bootloader mode run the following command:
```sh
make flash
```

Depending on your version of `stm32flash`, you may have to call the flash command multiple times before the flashing will actually start. Another common problem is that `stm32flash` sometimes misses the initializer message from the autopilot if plugged in via USB, or it gets confused by other data in the serial buffer. The best work around for this is to do the following:
- Unplug and unpower the autopilot completely
- Plug a USB-Serial adaptor into your computer
- Connect the USB-Serial Tx/Rx to the autopilot
- Power on the autopilot directly into bootloader mode

After the initial flash, you can use the MAVLINK command `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` to put the device into bootloader mode through the software. Additionally, the following command will attempt to send this directly from the CLI (but requires `pymavlink` to be installed):
```sh
make reflash
```

#### Flashing Options
You can not only pass the version flag in the command line, but you can also pass the serial parameters to use as well:
```sh
make flash SERIAL_DEVICE=/dev/ttyACM0 SERIAL_BAUD=57600 NAZE32_REV=5
```







