# Parameter File Reference

Name | Type | Description | Default | Unit | Options | Reboot
--- | --- | --- | ---:| --- | --- | ---
BOARD_REV | uint | A compile-time selector for what version of the board to setup sensors for |  | | | False
FW_VERSION | uint | A compile-time stamp for the flight firmware version |  | | | False
SW_VERSION | uint | A compile-time stamp for the OS firmware version |  | | | False
BAUD_RATE_0 | uint | Baud rate for the the COMM_0 port - set to 0 to disable | 921600 |  | [0, 9600, 57600, 115200, 921600] | True
BAUD_RATE_1 | uint | DISABLED: Baud rate for the the COMM_1 port - set to 0 to disable | 0 |  | [0, 9600, 57600, 115200, 921600] | True
COMMS_WAIT | uint | Instructs autopilot to remain silent until it receives a valid heartbeat message (False:0,True:1) | 0 | 0 / 1 | boolean | True
TIMESYNC_ALPHA | float | TODO - also check min-max values | 0.8 |  | [min:0.0, max:1.0] | False
MAV_SYS_ID | uint | Sets the MAVLINK System ID parameter for this mav | 1 |  | [min:0, max:255] | True
MAV_COMP_ID | uint | Sets the MAVLINK Component ID parameter for this mav | 1 |  | [min:0, max:255] | True
GCS_SYS_ID | uint | Sets the MAVLINK System ID parameter for the ground station | 1 |  | [min:0, max:255] | True
GCS_COMP_ID | uint | Sets the MAVLINK Component ID parameter for the ground station | 240 |  | [min:0, max:255] | True
RELAXED_SET | uint | Allows for 'unit' type parameters to be set when send as 'int' type | 1 | 0 / 1 | boolean | False
STRM0_HRTBT | float | Communication update rate for system heartbeat (set to 0 to turn off stream) | 1.0 | Hz | scalar | False
STRM0_SYS_STAT | float | Communication update rate for system status (set to 0 to turn off stream) | 0.2 | Hz | scalar | False
STRM0_HR_IMU | float | Communication update rate for IMU readings (set to 0 to turn off stream) | 100.0 | Hz | scalar | False
STRM0_ATT | float | Communication update rate for current attitude euler angles (set to 0 to turn off stream) | 0.0 | Hz | scalar | False
STRM0_ATT_Q | float | Communication update rate for current attitude quaternion (set to 0 to turn off stream) | 50.0 | Hz | scalar | False
STRM0_ATT_T | float | Communication update rate for current attitude target (set to 0 to turn off stream) | 50.0 | Hz | scalar | False
STRM0_SRV_OUT | float | Communication update rate for commanded servo output (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM0_TIMESYNC | float | Communication update rate for timesync (set to 0 to turn off stream) | 10.0 | Hz | scalar | False
STRM0_BATTSTAT | float | Communication update rate for battery status (set to 0 to turn off stream) | 2.0 | Hz | scalar | False
STRM0_LPQ | float | Communication update rate for all other messages (set to 0 to turn off stream) | 100.0 | Hz | scalar | False
CBRK_IMU | uint | Sensor circuit breaker arming check for IMU (set to 0 to disable checking device) | 1 | 0 / 1 | boolean | True
CBRK_MAG | uint | Sensor circuit breaker arming check for magnetometer (set to 0 to disable checking device) | 0 | 0 / 1 | boolean | True
CBRK_BARO | uint | Sensor circuit breaker arming check for barometer (set to 0 to disable checking device) | 0 | 0 / 1 | boolean | True
CBRK_SONAR | uint | Sensor circuit breaker arming check for sonar (set to 0 to disable checking device) | 0 | 0 / 1 | boolean | True
CBRK_EXT_POSE | uint | Sensor circuit breaker arming check for external pose estimate (set to 0 to disable checking device) | 1 | 0 / 1 | boolean | True
CBRK_SAFETY | uint | Sensor circuit breaker arming check for safety button (set to 0 to disable checking device) | 0 | 0 / 1 | boolean | True
CHK_RATE_BARO | float | Sensor update rate for barometer (TODO) | 0.0 | Hz | scalar | False
CHK_RATE_SONAR | float | Sensor update rate for sonar (TODO) | 0.0 | Hz | scalar | False
CHK_RATE_MAG | float | Sensor update rate for magnetometer (TODO) | 0.0 | Hz | scalar | False
STRM_NUM_IMU | uint | Number of IMU readings that must be recieved before a stream is established | 1000 |  | scalar | False
STRM_NUM_BARO | uint | Number of barometer readings that must be recieved before a stream is established | 50 |  | scalar | False
STRM_NUM_SONAR | uint | Number of sonar readings that must be recieved before a stream is established | 50 |  | scalar | False
STRM_NUM_EXT_P | uint | Number of external pose readings that must be recieved before a stream is established | 10 |  | scalar | False
STRM_NUM_MAG | uint | Number of magnetometer readings that must be recieved before a stream is established | 50 |  | scalar | False
STRM_NUM_OB_H | uint | Number of offboard heartbeat messages that must be recieved before a stream is established | 2 |  | scalar | False
STRM_NUM_OB_C | uint | Number of offboard control messages that must be recieved before a stream is established | 100 |  | scalar | False
STRM_NUM_RC_C | uint | Number of PWM control messages that must be recieved before a stream is established | 100 |  | scalar | False
TIMEOUT_IMU | uint | Time that new data must be read before an IMU timeout is declared | 2000 | us | scalar | False
TIMEOUT_BARO | uint | Time that new data must be read before a barometer timeout is declared | 20000 | us | scalar | False
TIMEOUT_SONAR | uint | Time that new data must be read before a sonar timeout is declared | 20000 | us | scalar | False
TIMEOUT_EXT_P | uint | Time that new data must be read before a external pose timeout is declared | 500000 | us | scalar | False
TIMEOUT_MAG | uint | Time that new data must be read before a magnetometer timeout is declared | 20000 | us | scalar | False
TIMEOUT_OB_HRBT | uint | Time that new data must be read before a off-board heartbeat timeout is declared | 5000000 | us | scalar | False
TIMEOUT_OB_CTRL | uint | Time that new data must be read before a off-board control timeout is declared | 200000 | us | scalar | False
TIMEOUT_RC_CTRL | uint | Time that new data must be read before a PWM control timeout is declared | 200000 | us | scalar | False
CAL_IMU_PASSES | uint | Number of samples to collect for IMU calibrations | 1000 |  | scalar | False
FILTER_INIT_T | uint | Time from boot where the estimator will use quick convergence | 3000 | ms | scalar | True
EST_ACC_COR | uint | Toggle to enable accelerometer correction (required for angular control) in the attitude estimator (0:false,1:true) | 1 |  | scalar | False
EST_MAT_EXP | uint | Toggle to enable matrix expotential approximation (adds decent estimation accuracy) in the attitude estimator (0:false,1:true) | 1 |  | scalar | False
EST_QUAD_INT | uint | Toggle to enable quadratic integration (adds a small amount of additional accuracy) in the attitude estimator (0:false,1:true) | 1 |  | scalar | False
EST_ADPT_BIAS | uint | Toggle to enable adaptive bias estimation (accounts for some additional sensor fluctuations) in the attitude estimator (0:false,1:true) | 0 |  | scalar | False
FILTER_KP | float | Adjusts the amount of proportional filtering done on the attitude estimation | 1.0 |  | scalar | False
FILTER_KI | float | Adjusts the amount of integral filtering done on the attitude estimation | 0.05 |  | scalar | False
EST_LPF_GYRO_A | float | Alpha parameter for gyroscope measurement low pass filter (noise reduction) | 0.6 |  | scalar | False
EST_LPF_ACC_A | float | Alpha parameter for gyroscope measurement low pass filter (noise reduction) | 0.6 |  | scalar | False
FSE_EXT_HDG_W | float | TODO: Weighting for external heading fusion (0 means don't trust, 1 means trust fully) | 0.2 |  | scalar | False
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
PID_ROLL_R_P | float | Proportional gain for roll rate PID | 0.05 |  | scalar | False
PID_ROLL_R_I | float | Integral gain for roll rate PID | 0.0 |  | scalar | False
PID_ROLL_R_D | float | Derivative gain for roll rate PID | 0.005 |  | scalar | False
MAX_ROLL_R | float | Maximum allowed command for roll rate (TODO) | 3.14159 | rad/s | scalar | False
PID_PITCH_R_P | float | Proportional gain for pitch rate PID | 0.05 |  | scalar | False
PID_PITCH_R_I | float | Integral gain for pitch rate PID | 0.0 |  | scalar | False
PID_PITCH_R_D | float | Derivative gain for pitch rate PID | 0.005 |  | scalar | False
MAX_PITCH_R | float | Maximum allowed command for pitch rate (TODO) | 3.14159 | rad/s | scalar | False
PID_YAW_R_P | float | Proportional gain for yaw rate PID | 0.2 |  | scalar | False
PID_YAW_R_I | float | Integral gain for yaw rate PID | 0.1 |  | scalar | False
PID_YAW_R_D | float | Derivative gain for yaw rate PID | 0.0 |  | scalar | False
MAX_YAW_R | float | Maximum allowed command for yaw rate (TODO) | 1.57079 | rad/s | scalar | False
PID_ROLL_ANG_P | float | Feed-forward gain for roll angle | 6.5 |  | scalar | False
MAX_ROLL_A | float | Maximum allowed command roll angle (TODO) | 0.786 | rad | scalar | False
PID_PITCH_ANG_P | float | Feed-forward gain for pitch angle | 6.5 |  | scalar | False
MAX_PITCH_A | float | Maximum allowed command pitch angle (TODO) | 0.786 | rad | scalar | False
PID_YAW_ANG_P | float | Feed-forward gain for yaw angle | 6.5 |  | scalar | False
PID_TAU | float | Time parameter for PID controllers | 0.05 |  | scalar | False
BATT_TYPE | uint | See the MAV_BATTERY_TYPE enum | 0 |  | scalar | False
BATT_FUNC | uint | See the MAV_BATTERY_FUNCTION enum | 0 |  | scalar | False
BATT_CELL_NUM | uint | Number of cells in the battery | 0 |  | scalar | False
BATT_CELL_MIN | float | Low cell voltage | 3.7 |  | scalar | False
BATT_CELL_MAX | float | High cell voltage | 4.2 |  | scalar | False
BATT_FILTER | float | A smoothing filter for the voltage reading (0->1) | 0.8 |  | scalar | False
MOTOR_PWM_RATE | uint | Update rate for PWM outputs | 400 | ? | scalar | True
MOTOR_PWM_IDLE | uint | Idle output for motors (when armed) | 1150 | pwm | scalar | False
MOTOR_PWM_MIN | uint | Minimum output for motors | 1000 | pwm | scalar | False
MOTOR_PWM_MAX | uint | Maximum output for motors | 2000 | pwm | scalar | False
DO_ESC_CAL | uint | When set to true, a motor calibration will be performed on the next boot (False:0,True:1) | 0 | 0 / 1 | boolean | True
FAILSAFE_THRTL | float | Throttle percentage output when in failsafe mode | 0.25 |  | scalar | False
TIMEOUT_THRTL | uint | Throttle timeout in to prevent accidentally leaving armed | 10000000 | us | scalar | False
SYS_AUTOSTART | uint | Mixer type to use (see mixer_type_t enum) | 0 |  | scalar | True
MAV_TYPE | uint | Convenience parameter, this is over-ridden during mixer selection on startup | 0 |  | scalar | False
SYS_AUTOCONFIG | uint | Tells the system to reset all parameters to default on next boot | 0 |  | scalar | True
