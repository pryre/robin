# Additional Information

## Rebooting and Rebooting to Bootloader
The autopilot can be commanded to reboot (when not in flight) using the `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` message, and optionally, can be put into bootloader mode for flashing.

#### Reboot
In MAVROS:
```sh
rosrun mavros mavcmd long 246 1 0 0 0 0 0 0
```

#### Reboot to Bootloader
In MAVROS:
```sh
rosrun mavros mavcmd long 246 3 0 0 0 0 0 0
```
