# Calibration
The robin flight software supports the `MAV_CMD_PREFLIGHT_CALIBRATION` command, and will issue instructions through the `STATUSTEXT` messages for the user to follow.

If using MAVROS, you should be able to use the following process to perform a calibration.

## Gyroscope
To perform a gyroscope calibration:
1. Place the autopilot on a flat surface
2. Do not bump the device then issue the following command:
```sh
rosrun mavros mavcmd long 241 1 0 0 0 0 0 0
```
3. The calibration should only take a few seconds

## Accelerometer
To perform a accelerometer calibration:
1. Place the autopilot on a flat surface, with the Z-axis pointing down (in the NED frame)
2. Hold the device steady, then issue the following command:
```sh
rosrun mavros mavcmd long 241 0 0 0 0 1 0 0
```
3. Repeat step 2, following the instructions sent back to MAVROS
4. The calibration should only take a few seconds per axis, but it will need to be done for each axis, in both directions
