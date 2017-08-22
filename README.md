# robin
The ROS Offboard Integration for the Naze32 Rev.5 (or similar Naze32-based board that uses the MPU-6050). 

## Preperation
### Ubuntu
```sh
sudo apt install gcc-arm-eabi-none stm32flash
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/qutas/robin/
```

## Compiling
```sh
cd ~/src/robin
make
```

## Flashing
Before you can flash the the Naze32, you must first put it into bootloader mode. For the initial flash, you can short out the bootloader pins and power on the device.

The makefile assumes that the device is connected as `/dev/ttyUSB0` and will use a baud rate of `921600`. You may have to adjust these to suit your device. If the flash is not successful, try using a slower baud rate.

Once in bootloader mode run the following command:
```sh
make flash
```

After the initial flash, you can use the MAVLINK command `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` to put the device into bootloader mode through the software. Additionally, the following command will attempt to send this directly from the CLI (but it may not work for all systems):
```sh
make reflash
```

## Interfacing
The Robin flight software can be interfaced using MAVLINK, and is compatible to much of the specification. Use of the MAVROS software is highly recommended.

The primary input for controlling the UAV is through the `SET_ATTITUDE_TARGET` message. If using MAVROS, it is recommended that you use the plugin `setpoint_raw` for sending commands. All `type_mask` options are supported for this message.

## Calibration
The robin flight software supports the `MAV_CMD_PREFLIGHT_CALIBRATION` command, and will issue instructions through the `STATUSTEXT` messages for the user to follow.

If using MAVROS, you should be able to use the following process to perform a calibration.

### Gyroscope
To perform a gyroscope calibration:
1. Place the autopilot on a flat surface
2. Do not bump the device then issue the following command:
```sh
rosrun mavros mavcmd long 241 1 0 0 0 0 0 0
```
3. The calibration should only take a few seconds

### Accelerometer
To perform a accelerometer calibration:
1. Place the autopilot on a flat surface, with the Z-axis pointing down (in the NED frame)
2. Hold the device steady, then issue the following command:
```sh
rosrun mavros mavcmd long 241 0 0 0 0 1 0 0
```
3. Repeat step 2, following the instructions sent back to MAVROS
4. The calibration should only take a few seconds per axis, but it will need to be done for each axis, in both directions

## Saving Parameters to the On-Board Flash Memory
To save any parameters that have been changed to make them persist across power downs, they must be saved to the onboard memory. This can be done using the `MAV_CMD_PREFLIGHT_STORAGE` message. In MAVROS:
```sh
rosrun mavros mavcmd long 245 1 0 0 0 0 0 0
```

## Rebooting and Rebooting to Bootloader
The autopilot can be commanded to reboot (when not in flight) using the `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` message, and optionally, can be put into bootloader mode for flashing.

### Reboot
In MAVROS:
```sh
rosrun mavros mavcmd long 246 1 0 0 0 0 0 0
```

### Reboot to Bootloader
In MAVROS:
```sh
rosrun mavros mavcmd long 246 3 0 0 0 0 0 0
```







