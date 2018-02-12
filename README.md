# robin
The ROS Offboard Integration for the Naze32 Rev.5 (or similar Naze32-based board that uses the MPU-6050).

## Preperation
#### Ubuntu
```sh
sudo apt install gcc-arm-none-eabi  stm32flash python3-yaml
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

#### Arch
```sh
sudo apt install arm-none-eabi-gcc arm-none-eabi-newlib python-yaml
```
You'll need to get the `stm32flash` package from the AUR or compile it manually

```sh
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

## Compiling
```sh
cd ~/src/robin
make
```

#### Version Options
By default, robin will compile for the Naze32 Rev6. This can be overridden with the `NAZE32_REV` flag for other devices, such as the Naze32 Rev5 or the Naze32 Mini Rev3 (both of these use Rev5):
```sh
make NAZE32\_REV=5
```

If you are constantly reflashing the firmware, you can adjust the default on the first few lines of the `makefile`.

## Flashing

---
**Note:** Some Naze32 models don not allow for flashing to be done via USB, and need to be connected to the UART1 port directly

---

Before you can flash the the Naze32, you must first put it into bootloader mode. For the initial flash, you can short out the bootloader pins and power on the device.

The makefile assumes that the device is connected as `/dev/ttyUSB0` and will use a baud rate of `921600`. You may have to adjust these to suit your device. If the flash is not successful, try using a slower baud rate, such as `115200`.

Depending on your version of `stm32flash`, you may have to call the flash command multiple times before the flashing will actually start.

Once in bootloader mode run the following command:
```sh
make flash
```

After the initial flash, you can use the MAVLINK command `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` to put the device into bootloader mode through the software. Additionally, the following command will attempt to send this directly from the CLI (but it may not work for all systems):
```sh
make reflash
```

For a reflash, you may get the best results if you set the baud rate to whatever your `PARAM_BAUD_RATE_0` is set to.

#### Flashing Options
You can not only pass the version flag in the command line, but you can also pass the serial parameters to use as well:
```sh
make flash SERIAL\_DEVICE=/dev/ttyACM0 SERIAL_BAUD=57600 NAZE32\_REV=5
```

## Documentation
Additional documentation on features, interfacing, and packaged tools, [please check here](documents/README.md)







