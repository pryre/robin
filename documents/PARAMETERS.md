# Parameters
[Back to index](README.md).

A parameter listing is dynamically generated on compile. During this time, the parameter headers, initializing functions, update funcitons, and a parameter reference document are generated and placed in `lib/param_generator`.

## Parameter Reference Document
The parameter reference document (`documents/autogen/PARAMS_LIST.md`) can be [found here](/documents/autogen/PARAMS_LIST.md).

This document contains a listing of the following headings:
- Name: Parameter name as sent through MAVLINK
- Type: The type of parameter this is (uint, int, float)
- Description: A brief description of what this parameter does in the code
- Default: The default value or option for this parameter
- Unit: Information on the parameter unit (if applicable)
- Options: Provides informatiom about the parameter option (list values, min/max, etc.)
- Reboot: Whether the flight controller needs to be rebooted for the parameter to take effect

## Saving Parameters to the On-Board Flash Memory
To save any parameters that have been changed to make them persist across power downs, they must be saved to the onboard memory. This can be done using the `MAV_CMD_PREFLIGHT_STORAGE` message.

In MAVROS:
```sh
rosrun mavros mavcmd long 245 1 0 0 0 0 0 0
```

In QGCS, you must use the "Write to EEPROM" button available in the QGCS [widget](https://github.com/qutas/robin/blob/master/lib/qgroundcontrol_plugins/RobinCommandPanel.qml).

## Useful parameter settings
This is a listing of potentially useful parameters, or parameters that users should be aware exist. The list has been condensed with square-bracket listing, so if looking up these parameters in the [full parameter reference](/lib/param_generator/PARAMS.md), you will need to search for a single option from the square brackets

#### System parameters
```
SYS_AUTOSTART - Sets the mixer to use based on the airframe type
SYS_AUTOCONFIG - Tells the autopilot to reset to default on the next boot
BOARD_REV - Sets the board revision, primarily for sensors and pinouts
VERSION_[FW,SW] - Generated during the build process, helps with debugging
RELAXED_SET - Relaxes specified vs. expected parameter type during parameter changes (required for mavros and QGCS)
```

#### MAVLINK Input/Output
```
BAUD_RATE_0 - Sets the primary telemetry baud rate
MAV_[SYS,COMP]_ID - Sets the system and component IDs of this system
COMMS_WAIT - Tells the system to silently wait until it is contacted before sending telemetry
GCS_MATCH - Tells the system to only accept commands from a single GCS (set by GCS_[SYS,COMP]_ID)
GCS_[SYS,COMP]_ID - Sets the GCS expected if only listening to a single GCS (GCS_MATCH)
STRM0_[HRTBT,SYS_STAT,HR_IMU,ATT_Q,ATT_T,RC_IN,SRV_OUT,BATTSTAT,LPQ] - Sets the update rate of streamed telemetry
```

#### Sensors
```
CBRK_[IMU,MAG,BARO,SONAR,EXT_POSE,SAFETY] - Allows disabling of optional sensors
STRM_NUM_[IMU,BARO,SONAR,RC_IN,EXT_P,MAG,OB_H,OB_C,RC_C] - Sets how many messages must be recieved before sensor can be trusted
TIMEOUT_[IMU,BARO,SONAR,RC_IN,EXT_P,MAG,OB_HRBT,OB_CTRL,RC_CTRL] - Sets the period for new data to arrive before a sensors is ignored
```

#### Control & Tuning
```
MC_[ROLLRATE,PITCHRATE,YAWRATE]_[P,I,D,MAX] - Low level tuning for roll, pitch, and yaw (tune this if there are issues holding steady)
MC_[ROLL,PITCH,YAW]_P - High level tuning for roll, pitch, and yaw (will affect how aggressive the response is in angular/stabilized mode
MAX_[ROLL,PITCH]_A - Sets the maximum bank angles for input commands (affects RC command range too)
```

#### Battery Monitoring
```
BAT_TYPE - Sets the reported battery type
BAT_FUNC - Sets the reported battery function
BAT_N_CELLS - Sets the number of cells to use when calculating voltages
BAT_V_[EMPTY,CHARGED] - Sets the expected charge range for cells
BAT_[LOW,CRIT,EMERGEN]_THR - Sets the battery reporting thresholds for battery health
```

#### RC Input & Calibration
```
RC_MAP_[ROLL,PITCH,YAW,THROTTLE,MODE_SW] - Sets which RC inputs control which which command input
RC_DEFAULT_MODE - Allows for a default mode to be set if no mode switch is set (RC_MAP_MODE_SW)
RC[1,2,3,4,5,6,7,8]_[MIN,TRIM,MAX,DZ,REV] - Calibration data for minimum, maximum, trim settings, stick deadzones, and input reverse
```

#### Additional Settings
```
PWM_IDLE - Sets motor idle speed when system is armed
DO_ESC_CAL - Prepare to perform an ESC calibration on the next boot
FAILSAFE_THRTL - Sets the failsafe throttle percentage to use if control input is lost during flight
TIMEOUT_THRTL - Sets a timeout for throttle input after system is armed (system will disarm if no throttle is input before this time)
```
