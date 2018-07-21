# Parameter File Reference

Name | Type | Description | Default | Unit | Options | Reboot
--- | --- | --- | ---:| --- | --- | ---
BOARD_REV | uint | A compile-time selector for what version of the board to setup sensors for |  | | | False
VERSION_FW | uint | A compile-time stamp for the flight firmware version |  | | | False
VERSION_SW | uint | A compile-time stamp for the OS firmware version |  | | | False
BAUD_RATE_0 | uint | Baud rate for the the COMM_0 port - set to 0 to disable | 921600 |  | [0, 9600, 57600, 115200, 921600] | True
BAUD_RATE_1 | uint | Baud rate for the the COMM_1 port - set to 0 to disable | 115200 |  | [0, 9600, 57600, 115200, 921600] | True
COMMS_WAIT | uint | Instructs autopilot to remain silent until it receives a valid heartbeat message (False:0,True:1) | 1 | 0 / 1 | boolean | True
TIMESYNC_ALPHA | float | TODO - also check min-max values | 0.8 |  | [min:0.0, max:1.0] | False
MAV_SYS_ID | uint | Sets the MAVLINK System ID parameter for this mav | 1 |  | [min:0, max:255] | True
MAV_COMP_ID | uint | Sets the MAVLINK Component ID parameter for this mav | 1 |  | [min:0, max:255] | True
GCS_MATCH | uint | Sets the MAVLINK System ID parameter for the ground station | 0 | 0 / 1 | boolean | True
GCS_SYS_ID | uint | Sets the MAVLINK System ID parameter for the ground station | 1 |  | [min:0, max:255] | True
GCS_COMP_ID | uint | Sets the MAVLINK Component ID parameter for the ground station | 240 |  | [min:0, max:255] | True
RELAXED_SET | uint | Allows for 'unit' type parameters to be set when send as 'int' type | 1 | 0 / 1 | boolean | False
STRM0_HRTBT | float | Communication update rate for system heartbeat (set to 0 to turn off stream) | 1.0 | Hz | scalar | False
STRM0_SYS_STAT | float | Communication update rate for system status (set to 0 to turn off stream) | 0.2 | Hz | scalar | False
STRM0_HR_IMU | float | Communication update rate for IMU readings (set to 0 to turn off stream) | 100.0 | Hz | scalar | False
STRM0_ATT | float | Communication update rate for current attitude euler angles (set to 0 to turn off stream) | 0.0 | Hz | scalar | False
STRM0_ATT_Q | float | Communication update rate for current attitude quaternion (set to 0 to turn off stream) | 50.0 | Hz | scalar | False
STRM0_ATT_T | float | Communication update rate for current attitude target (set to 0 to turn off stream) | 50.0 | Hz | scalar | False
STRM0_RC_IN | float | Communication update rate for RC input channels (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM0_SRV_OUT | float | Communication update rate for commanded servo output (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM0_TIMESYNC | float | Communication update rate for timesync (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM0_BATTSTAT | float | Communication update rate for battery status (set to 0 to turn off stream) | 2.0 | Hz | scalar | False
STRM0_LPQ | float | Communication update rate for all other messages (set to 0 to turn off stream) | 100.0 | Hz | scalar | False
STRM1_HRTBT | float | Communication update rate for system heartbeat (set to 0 to turn off stream) | 1.0 | Hz | scalar | False
STRM1_SYS_STAT | float | Communication update rate for system status (set to 0 to turn off stream) | 0.2 | Hz | scalar | False
STRM1_HR_IMU | float | Communication update rate for IMU readings (set to 0 to turn off stream) | 20.0 | Hz | scalar | False
STRM1_ATT | float | Communication update rate for current attitude euler angles (set to 0 to turn off stream) | 0.0 | Hz | scalar | False
STRM1_ATT_Q | float | Communication update rate for current attitude quaternion (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM1_ATT_T | float | Communication update rate for current attitude target (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM1_RC_IN | float | Communication update rate for RC input channels (set to 0 to turn off stream) | 5.0 | Hz | scalar | False
STRM1_SRV_OUT | float | Communication update rate for commanded servo output (set to 0 to turn off stream) | 5.0 | Hz | scalar | False
STRM1_TIMESYNC | float | Communication update rate for timesync (set to 0 to turn off stream) | 2.0 | Hz | scalar | False
STRM1_BATTSTAT | float | Communication update rate for battery status (set to 0 to turn off stream) | 2.0 | Hz | scalar | False
STRM1_LPQ | float | Communication update rate for all other messages (set to 0 to turn off stream) | 40.0 | Hz | scalar | False
CBRK_IMU | uint | Sensor circuit breaker arming check for IMU (set to 0 to disable checking device) | 1 | 0 / 1 | boolean | True
CBRK_SAFETY | uint | Sensor circuit breaker arming check for safety button (set to 0 to disable checking device) | 1 | 0 / 1 | boolean | True
CHK_RATE_MAG | float | Sensor update rate for magnetometer (set to 0 to disable, typical is 20Hz) | 0.0 | Hz | scalar | True
CHK_RATE_BARO | float | Sensor update rate for barometer (set to 0 to disable, typical is 20Hz) | 0.0 | Hz | scalar | True
CHK_RATE_SONAR | float | Sensor update rate for sonar (TODO) (set to 0 to disable, typical is 5Hz) | 0.0 | Hz | scalar | True
STRM_NUM_IMU | uint | Number of IMU readings that must be recieved before a stream is established | 1000 |  | scalar | False
STRM_NUM_BARO | uint | Number of barometer readings that must be recieved before a stream is established | 50 |  | scalar | False
STRM_NUM_SONAR | uint | Number of sonar readings that must be recieved before a stream is established | 50 |  | scalar | False
STRM_NUM_RC_IN | uint | Number of valid RC input readings that must be recieved before a stream is established | 2000 |  | scalar | False
STRM_NUM_EXT_P | uint | Number of external pose readings that must be recieved before a stream is established | 10 |  | scalar | False
STRM_NUM_MAG | uint | Number of magnetometer readings that must be recieved before a stream is established | 50 |  | scalar | False
STRM_NUM_OB_H | uint | Number of offboard heartbeat messages that must be recieved before a stream is established | 2 |  | scalar | False
STRM_NUM_OB_C | uint | Number of offboard control messages that must be recieved before a stream is established | 100 |  | scalar | False
STRM_NUM_RC_C | uint | Number of PWM control messages that must be recieved before a stream is established | 100 |  | scalar | False
TIMEOUT_IMU | uint | Time that new data must be read before an IMU timeout is declared | 2000 | us | scalar | False
TIMEOUT_MAG | uint | Time that new data must be read before a magnetometer timeout is declared | 100000 | us | scalar | False
TIMEOUT_BARO | uint | Time that new data must be read before a barometer timeout is declared | 100000 | us | scalar | False
TIMEOUT_SONAR | uint | Time that new data must be read before a sonar timeout is declared | 100000 | us | scalar | False
TIMEOUT_RC_IN | uint | Time that new data must be read before a RC input timeout is declared | 1000000 | us | scalar | False
TIMEOUT_EXT_P | uint | Time that new data must be read before a external pose timeout is declared | 500000 | us | scalar | False
TIMEOUT_OB_HRBT | uint | Time that new data must be read before a off-board heartbeat timeout is declared | 5000000 | us | scalar | False
TIMEOUT_OB_CTRL | uint | Time that new data must be read before a off-board control timeout is declared | 200000 | us | scalar | False
TIMEOUT_RC_CTRL | uint | Time that new data must be read before a PWM control timeout is declared | 200000 | us | scalar | False
CAL_IMU_PASSES | uint | Number of samples to collect for IMU calibrations | 1000 |  | scalar | False
EST_FLTR_INIT | uint | Time from boot where the estimator will use quick convergence | 3000 | ms | scalar | True
EST_ACC_COR | uint | Toggle to enable accelerometer correction (required for angular control) in the attitude estimator (0:false,1:true) | 1 |  | scalar | False
EST_MAT_EXP | uint | Toggle to enable matrix expotential approximation (adds decent estimation accuracy) in the attitude estimator (0:false,1:true) | 1 |  | scalar | False
EST_QUAD_INT | uint | Toggle to enable quadratic integration (adds a small amount of additional accuracy) in the attitude estimator (0:false,1:true) | 1 |  | scalar | False
EST_ADPT_BIAS | uint | Toggle to enable adaptive bias estimation (accounts for some additional sensor fluctuations) in the attitude estimator (0:false,1:true) | 0 |  | scalar | False
EST_LVL_HORZ | uint | Toggle to enable level horizon calibration for the attitude estimation (0:false,1:true) | 1 |  | scalar | False
EST_LVL_HORZ_W | float | Level horizon offset quaternion calibration data (W) | 1.0 |  | scalar | False
EST_LVL_HORZ_X | float | Level horizon offset quaternion calibration data (X) | 0.0 |  | scalar | False
EST_LVL_HORZ_Y | float | Level horizon offset quaternion calibration data (Y) | 0.0 |  | scalar | False
EST_LVL_HORZ_Z | float | Level horizon offset quaternion calibration data (Z) | 0.0 |  | scalar | False
EST_FLTR_KP | float | Adjusts the amount of proportional filtering done on the attitude estimation | 1.0 |  | scalar | False
EST_FLTR_KI | float | Adjusts the amount of integral filtering done on the attitude estimation | 0.05 |  | scalar | False
EST_LPF_GYRO_A | float | Alpha parameter for gyroscope measurement low pass filter (noise reduction) | 0.04 |  | scalar | False
EST_LPF_ACC_A | float | Alpha parameter for gyroscope measurement low pass filter (noise reduction) | 0.04 |  | scalar | False
FSE_EXT_HDG_W | float | Weighting for external heading fusion (0 means don't trust, 1 means trust fully) | 0.2 |  | scalar | False
FSE_MAG_HDG_W | float | Weighting for compass heading fusion (0 means don't trust, 1 means trust fully) | 0.5 |  | scalar | False
GYRO_X_BIAS | int | Bias correction for gyroscope measurements (X axis) | 0 |  | scalar | False
GYRO_Y_BIAS | int | Bias correction for gyroscope measurements (Y axis) | 0 |  | scalar | False
GYRO_Z_BIAS | int | Bias correction for gyroscope measurements (Z axis) | 0 |  | scalar | False
ACC_X_BIAS | int | Bias correction for accelerometer measurements (X axis) | 0 |  | scalar | False
ACC_Y_BIAS | int | Bias correction for accelerometer measurements (Y axis) | 0 |  | scalar | False
ACC_Z_BIAS | int | Bias correction for accelerometer measurements (Z axis) | 0 |  | scalar | False
ACC_X_S_POS | float | Scaling correction for accelerometer measurements (X positive axis) | 1.0 |  | scalar | False
ACC_X_S_NEG | float | Scaling correction for accelerometer measurements (X negative axis) | 1.0 |  | scalar | False
ACC_Y_S_POS | float | Scaling correction for accelerometer measurements (Y positive axis) | 1.0 |  | scalar | False
ACC_Y_S_NEG | float | Scaling correction for accelerometer measurements (Y negative axis) | 1.0 |  | scalar | False
ACC_Z_S_POS | float | Scaling correction for accelerometer measurements (Z positive axis) | 1.0 |  | scalar | False
ACC_Z_S_NEG | float | Scaling correction for accelerometer measurements (Z negative axis) | 1.0 |  | scalar | False
ACC_X_TEMP_COMP | float | Temperature correction for accelerometer measurements (X axis) | 0.0 |  | scalar | False
ACC_Y_TEMP_COMP | float | Temperature correction for accelerometer measurements (Y axis) | 0.0 |  | scalar | False
ACC_Z_TEMP_COMP | float | Temperature correction for accelerometer measurements (Z axis) | 0.0 |  | scalar | False
RATE_CONTROL | float | Update rate of the controller | 250.0 |  | scalar | True
MC_ROLLRATE_P | float | Proportional gain for roll rate PID | 0.02 |  | scalar | False
MC_ROLLRATE_I | float | Integral gain for roll rate PID | 0.01 |  | scalar | False
MC_ROLLRATE_D | float | Derivative gain for roll rate PID | 0.001 |  | scalar | False
MC_ROLLRATE_MAX | float | Maximum allowed command for roll rate | 3.14159 | rad/s | scalar | False
MC_PITCHRATE_P | float | Proportional gain for pitch rate PID | 0.02 |  | scalar | False
MC_PITCHRATE_I | float | Integral gain for pitch rate PID | 0.01 |  | scalar | False
MC_PITCHRATE_D | float | Derivative gain for pitch rate PID | 0.001 |  | scalar | False
MC_PITCHRATE_MAX | float | Maximum allowed command for pitch rate | 3.14159 | rad/s | scalar | False
MC_YAWRATE_P | float | Proportional gain for yaw rate PID | 0.05 |  | scalar | False
MC_YAWRATE_I | float | Integral gain for yaw rate PID | 0.01 |  | scalar | False
MC_YAWRATE_D | float | Derivative gain for yaw rate PID | 0.0 |  | scalar | False
MC_YAWRATE_MAX | float | Maximum allowed command for yaw rate | 1.57079 | rad/s | scalar | False
MC_ROLL_P | float | Feed-forward gain for roll angle | 6.5 |  | scalar | False
MAX_ROLL_A | float | Maximum allowed command roll angle (TODO) | 0.786 | rad | scalar | False
MC_PITCH_P | float | Feed-forward gain for pitch angle | 6.5 |  | scalar | False
MAX_PITCH_A | float | Maximum allowed command pitch angle (TODO) | 0.786 | rad | scalar | False
MC_YAW_P | float | Feed-forward gain for yaw angle | 2.8 |  | scalar | False
BAT_TYPE | uint | See the MAV_BATTERY_TYPE enum | 0 |  | scalar | False
BAT_FUNC | uint | See the MAV_BATTERY_FUNCTION enum | 0 |  | scalar | False
BAT_N_CELLS | uint | Number of cells in the battery | 0 |  | scalar | False
BAT_V_EMPTY | float | Low cell voltage | 3.7 |  | scalar | False
BAT_V_CHARGED | float | High cell voltage | 4.2 |  | scalar | False
BAT_FILTER | float | A smoothing filter for the voltage reading (0->1) | 0.8 |  | scalar | False
BAT_V_DIV | float | Resistor voltage divider for battery sensing (-1 to use board default) | -1.0 |  | scalar | False
BAT_LOW_THR | float | Charge state cutoff for low battery remaining (as a percentage) | 0.2 |  | scalar | False
BAT_CRIT_THR | float | Charge state cutoff for critical battery remaining (as a percentage) | 0.1 |  | scalar | False
BAT_EMERGEN_THR | float | Charge state cutoff for emergency battery remaining (as a percentage) | 0.05 |  | scalar | False
PWM_RATE | uint | Update rate for PWM outputs | 400 | ? | scalar | True
PWM_IDLE | uint | Idle output for motors (when armed) | 1150 | pwm | scalar | False
PWM_MIN | uint | Minimum output for motors | 1000 | pwm | scalar | False
PWM_MAX | uint | Maximum output for motors | 2000 | pwm | scalar | False
RC_MAP_ROLL | uint | Channel to use for RC roll inputs | 0 | channel | scalar | False
RC_MAP_PITCH | uint | Channel to use for RC pitch inputs | 0 | channel | scalar | False
RC_MAP_YAW | uint | Channel to use for RC yaw inputs | 0 | channel | scalar | False
RC_MAP_THROTTLE | uint | Channel to use for RC throttle inputs | 0 | channel | scalar | False
RC_MAP_MODE_SW | uint | Channel to use for RC mode select | 0 | channel | scalar | False
RC_MAP_AUX1 | uint | RC channel to use for AUX1 passthrough, if there are free mixer channels (set to 0 to disable) | 0 | channel | scalar | True
RC_MAP_AUX2 | uint | RC channel to use for AUX2 passthrough, if there are free mixer channels (set to 0 to disable) | 0 | channel | scalar | True
RC_MAP_AUX3 | uint | RC channel to use for AUX3 passthrough, if there are free mixer channels (set to 0 to disable) | 0 | channel | scalar | True
RC_MAP_AUX4 | uint | RC channel to use for AUX4 passthrough, if there are free mixer channels (set to 0 to disable) | 0 | channel | scalar | True
RC_MODE_DEFAULT | uint | Configures the system to set a specific mode on RC connect if PARAM_RC_MAP_MODE_SW is unset (set to 0 to disable) | 0 | channel | scalar | False
RC_MODE_PWM_RNG | uint | Sets the range (+ or -) that counts as a selection for any PWM mode selection | 100 | channel | scalar | False
RC_MODE_PWM_STAB | uint | The PWM value to use to select manual stabilized mode (set to 0 to disable) | 1100 | channel | scalar | False
RC_MODE_PWM_ACRO | uint | The PWM value to use to select manual acro mode (set to 0 to disable) | 0 | channel | scalar | False
RC_MODE_PWM_OFFB | uint | The PWM value to use to select offboard mode (set to 0 to disable) | 1900 | channel | scalar | False
COM_RC_ARM_HYST | uint | Time threshold to activate RC arming | 1000000 | us | scalar | False
RC1_MIN | uint | RC minimum value for channel 1 calibration | 1000 | channel | scalar | False
RC1_TRIM | uint | RC median value for channel 1 calibration | 1500 | channel | scalar | False
RC1_MAX | uint | RC max value for channel 1 calibration | 2000 | channel | scalar | False
RC1_REV | uint | RC reverse reading channel 1 calibration | 0 | 0 / 1 | boolean | False
RC1_DZ | float | RC deadzone for channel 1 | 0.02 |  | scalar | False
RC2_MIN | uint | RC minimum value for channel 2 calibration | 1000 | channel | scalar | False
RC2_TRIM | uint | RC median value for channel 2 calibration | 1500 | channel | scalar | False
RC2_MAX | uint | RC max value for channel 2 calibration | 2000 | channel | scalar | False
RC2_REV | uint | RC reverse reading channel 2 calibration | 0 | 0 / 1 | boolean | False
RC2_DZ | float | RC deadzone for channel 2 | 0.02 |  | scalar | False
RC3_MIN | uint | RC minimum value for channel 3 calibration | 1000 | channel | scalar | False
RC3_TRIM | uint | RC median value for channel 3 calibration | 1500 | channel | scalar | False
RC3_MAX | uint | RC max value for channel 3 calibration | 2000 | channel | scalar | False
RC3_REV | uint | RC reverse reading channel 3 calibration | 0 | 0 / 1 | boolean | False
RC3_DZ | float | RC deadzone for channel 3 | 0.02 |  | scalar | False
RC4_MIN | uint | RC minimum value for channel 4 calibration | 1000 | channel | scalar | False
RC4_TRIM | uint | RC median value for channel 4 calibration | 1500 | channel | scalar | False
RC4_MAX | uint | RC max value for channel 4 calibration | 2000 | channel | scalar | False
RC4_REV | uint | RC reverse reading channel 4 calibration | 0 | 0 / 1 | boolean | False
RC4_DZ | float | RC deadzone for channel 4 | 0.02 |  | scalar | False
RC5_MIN | uint | RC minimum value for channel 5 calibration | 1000 | channel | scalar | False
RC5_TRIM | uint | RC median value for channel 5 calibration | 1500 | channel | scalar | False
RC5_MAX | uint | RC max value for channel 5 calibration | 2000 | channel | scalar | False
RC5_REV | uint | RC reverse reading channel 5 calibration | 0 | 0 / 1 | boolean | False
RC5_DZ | float | RC deadzone for channel 5 | 0.02 |  | scalar | False
RC6_MIN | uint | RC minimum value for channel 6 calibration | 1000 | channel | scalar | False
RC6_TRIM | uint | RC median value for channel 6 calibration | 1500 | channel | scalar | False
RC6_MAX | uint | RC max value for channel 6 calibration | 2000 | channel | scalar | False
RC6_REV | uint | RC reverse reading channel 6 calibration | 0 | 0 / 1 | boolean | False
RC6_DZ | float | RC deadzone for channel 6 | 0.02 |  | scalar | False
RC7_MIN | uint | RC minimum value for channel 7 calibration | 1000 | channel | scalar | False
RC7_TRIM | uint | RC median value for channel 7 calibration | 1500 | channel | scalar | False
RC7_MAX | uint | RC max value for channel 7 calibration | 2000 | channel | scalar | False
RC7_REV | uint | RC reverse reading channel 7 calibration | 0 | 0 / 1 | boolean | False
RC7_DZ | float | RC deadzone for channel 7 | 0.02 |  | scalar | False
RC8_MIN | uint | RC minimum value for channel 8 calibration | 1000 | channel | scalar | False
RC8_TRIM | uint | RC median value for channel 8 calibration | 1500 | channel | scalar | False
RC8_MAX | uint | RC max value for channel 8 calibration | 2000 | channel | scalar | False
RC8_REV | uint | RC reverse reading channel 8 calibration | 0 | 0 / 1 | boolean | False
RC8_DZ | float | RC deadzone for channel 8 | 0.02 |  | scalar | False
DO_ESC_CAL | uint | When set to true, a motor calibration will be performed on the next boot (False:0,True:1) | 0 | 0 / 1 | boolean | True
FAILSAFE_THRTL | float | Throttle percentage output when in failsafe mode | 0.25 |  | scalar | False
TIMEOUT_THRTL | uint | Throttle timeout in to prevent accidentally leaving armed | 10000000 | us | scalar | False
MAV_TYPE | uint | Convenience parameter, this is over-ridden during mixer selection on startup | 0 |  | scalar | False
SYS_AUTOSTART | uint | Mixer type to use (see mixer_type_t enum) [and write params imidiately to survive qgcs reboot] | 0 |  | scalar | True
SYS_AUTOCONFIG | uint | Tells the system to reset all parameters to default on next boot | 0 |  | scalar | True
