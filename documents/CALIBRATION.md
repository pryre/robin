# Calibration
[Back to index](README.md).

The robin flight software supports the `MAV_CMD_PREFLIGHT_CALIBRATION` command, and will issue instructions through the `STATUSTEXT` messages for the user to follow. It is recommended that you leave the flight controller on for ~5 minutes to allow all of the components to heat up, as IMU temperature compensation is currently not implemented. It should also be noted that to ensure correct procedure, only 1 calibration can be performed at a time.

If using QGCS, the [custom interface](TOOLS.md#qgroundcontrol) can be used to issue the corresponding commands. It is recommended that you open the _notices_ menu so you can easily see the calibration instructions.

If using MAVROS, you should be able to use the following process to perform a calibration. It is recommended that you open the command interface so you can easily see the calibration instructions.

## Gyroscope
To perform a gyroscope calibration:
1. Place the autopilot on a flat surface
2. Do not bump the device then issue the calibration command:
   - MAVROS: `rosrun mavros mavcmd long 241 1 0 0 0 0 0 0`
   - QGCS Widget: Press `Send gyroscope calibration command`
3. The calibration should only take a few seconds

## Accelerometer
The flight controller uses the NED frame of reference. During the calibration steps you will be instructed to orient each axis of the flight controller, with the axis oriented both pointing up to the sky and pointing down to the ground.

![http://www.chrobotics.com/wp-content/uploads/2012/11/Inertial-Frame.png](https://raw.githubusercontent.com/qutas/robin/master/documents/Inertial-Frame.png)

To perform a accelerometer calibration:
1. Place the autopilot on a flat surface, with the Z-axis pointing down (in the NED frame)
2. Hold the device steady, then issue the calibration command:
   - MAVROS: `rosrun mavros mavcmd long 241 0 0 0 0 1 0 0`
   - QGCS Widget: Press `Send accelerometer calibration command`
3. Repeat step 2, following the instructions sent back from the autopilot
4. The calibration should only take a few seconds per axis, but it will need to be done for each axis, in both directions

## Radio Control
Before performing a radio control calibration, you must first connect a PPM compatible RC reciever. For the complete calibtation, you should also set the parameters `RC_MAP_ROLL`, `RC_MAP_PITCH`, `RC_MAP_YAW`, and `RC_MAP_THROTTLE` before proceding.

To perform an RC calibration:
1. Ensure RC transmitter and reciever are bound and ready. Send the initial RC calibration command:
   - MAVROS: `rosrun mavros mavcmd long 241 0 0 0 1 0 0 0`
   - QGCS Widget: Press `Send RC calibration command`

![RC Calibration Stage 1](https://raw.githubusercontent.com/qutas/robin/master/documents/rc_step_1.png)

2. Set RC to stick centered, then reissue the calibration command.

![RC Calibration Stage 2](https://raw.githubusercontent.com/qutas/robin/master/documents/rc_step_2.png)

3. Set RC to lower inner corners, then reissue the calibration command.

![RC Calibration Stage 3](https://raw.githubusercontent.com/qutas/robin/master/documents/rc_step_3.png)

4. Move all sticks and switches to extremes, then reissue the calibration command when done.

5. Calibration should now be complete.

## Battery Monitoring
Battery monitoring can be performed in two steps.
1. Enable monitoring by setting the number of cells of your battery with the `BATT_N_CELLS` parameter (currently only supports LIPO batteries).
2. Attached the battery voltage to the `VBATT` pins on the flight controller

---
**Note:** The flight controllers are not protected from reverse polarity across the `VBATT` port (as it is much higher than the operating voltage). Connecting this port backwards **will destroy your flight controller**. It is highly recommended that you makke this connection in a manner that cannot be accidentally reversed (i.e. solder wires directly from your PDB to the appropriate pints on the flight controller.

---

Once configured and connected, you can adjust your battery readout, and the various feedback warnings via the other `BATT_*` parameters (see the parameter reference for more details.
