#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sensors.h"
#include "controller.h"
#include "estimator.h"
#include "safety.h"
#include "params.h"
#include "mavlink_system.h"
#include "breezystm32.h"
#include "drv_mpu.h"
#include "drv_hmc5883l.h"
#include "pwm.h"
#include "adc.h"
#include "gpio.h"

#include "fix16.h"
#include "fixvector3d.h"
#include "fixquat.h"
#include "fixextra.h"

//#include "stdio.h"
#include <stdlib.h>

#define GYRO_HIGH_BIAS_WARN 200

//==-- Local Variables
int32_t _imu_time_read = 0;
static volatile uint8_t accel_status = 0;
static volatile uint8_t gyro_status = 0;
static volatile uint8_t temp_status = 0;

static int16_t read_accel_raw[3];
static int16_t read_gyro_raw[3];
static volatile int16_t read_temp_raw;

static volatile uint8_t mag_status = 0;
static int16_t read_mag_raw[3];

static uint16_t rc_cal[8][3];
static bool rc_rev[8];
static fix16_t rc_dz[8];

sensor_readings_t _sensors;
sensor_calibration_t _sensor_calibration;

system_status_t _system_status;
command_input_t _cmd_rc_input;

//==-- Functions
static void clock_init(void) {
	_sensors.clock.present = true;

	_sensors.clock.start = 0;
	_sensors.clock.end = 0;
	_sensors.clock.dt = 0;

	_sensors.clock.counter = 0;
	_sensors.clock.max = 0;
	_sensors.clock.min = 1000;

	_sensors.clock.imu_time_read = 0;

	_sensors.clock.rt_offset_ns = 0;
	_sensors.clock.rt_drift = 1.0;
	_sensors.clock.rt_ts_last = 0;
	_sensors.clock.rt_tc_last = 0;
	_sensors.clock.rt_sync_last = 0;
}

static void sensors_imu_disable(void) {
}

void sensors_imu_poll(void) {
	//==-- Timing setup get loop time
	_imu_time_read = micros();

	mpu_request_async_accel_read(read_accel_raw, &accel_status);
	mpu_request_async_gyro_read(read_gyro_raw, &gyro_status);
	mpu_request_async_temp_read(&(read_temp_raw), &temp_status);
}

static void sensor_status_init(sensor_status_t *status, bool sensor_present) {
	status->present = sensor_present;
	status->new_data = false;
	status->time_read = 0;
}

void sensors_cal_init(void) {
	_sensor_calibration.type = SENSOR_CAL_NONE;

	_sensor_calibration.data.gyro.count = 0;
	_sensor_calibration.data.gyro.sum_x = 0;
	_sensor_calibration.data.gyro.sum_y = 0;
	_sensor_calibration.data.gyro.sum_z = 0;

	_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_INIT;
	_sensor_calibration.data.accel.waiting = false;
	_sensor_calibration.data.accel.data.count = 0;

	_sensor_calibration.data.accel.data.t_sum = 0;
	_sensor_calibration.data.accel.data.x_sum = 0;
	_sensor_calibration.data.accel.data.y_sum = 0;
	_sensor_calibration.data.accel.data.z_sum = 0;

	_sensor_calibration.data.accel.data.t_av_sum = 0;
	_sensor_calibration.data.accel.data.x_flat_av_sum = 0;
	_sensor_calibration.data.accel.data.y_flat_av_sum = 0;
	_sensor_calibration.data.accel.data.z_flat_av_sum = 0;
	_sensor_calibration.data.accel.data.x_up_av = 0;
	_sensor_calibration.data.accel.data.x_down_av = 0;
	_sensor_calibration.data.accel.data.y_up_av = 0;
	_sensor_calibration.data.accel.data.y_down_av = 0;
	_sensor_calibration.data.accel.data.z_up_av = 0;
	_sensor_calibration.data.accel.data.z_down_av = 0;

	_sensor_calibration.data.rc.waiting = false;
	_sensor_calibration.data.rc.step = SENSOR_CAL_RC_RANGE_INIT;
	for(int i=0;i<8;i++) {
		//XXX: Init all to "true stick centre"
		_sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MIN] = 1500;
		_sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MID] = 1500;
		_sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MAX] = 1500;
		_sensor_calibration.data.rc.rc_rev[i] = false;
	}

	_sensor_calibration.data.accel.temp_scale = fix16_from_float(340.0f);
	_sensor_calibration.data.accel.temp_shift = fix16_from_float(36.53f);
}

void sensors_deinit_imu(void) {
	mpu_register_interrupt_cb(&sensors_imu_disable, get_param_uint(PARAM_BOARD_REVISION));
	while( i2c_job_queued() ); //Wait for jobs to finish
    //mpuWriteRegisterI2C(MPU_RA_PWR_MGMT_1, MPU_BIT_DEVICE_RESET);
	//delay(500);
}

void sensors_init_imu(void) {
	sensor_status_init(&_sensors.imu.status, (bool)get_param_uint(PARAM_SENSOR_IMU_CBRK));
	mpu_register_interrupt_cb(&sensors_imu_poll, get_param_uint(PARAM_BOARD_REVISION));

	switch(get_param_uint(PARAM_BOARD_REVISION)) {
		case 5: {
			//Get the 1g gravity scale (raw->g's)
			_sensor_calibration.data.accel.acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
			break;
		}
		case 6: {
			//Get the 1g gravity scale (raw->g's)
			_sensor_calibration.data.accel.acc1G = mpu6500_init(INV_FSR_8G, INV_FSR_2000DPS);
			break;
		}
		default: {
			//Could not determine IMU!
			failureMode(5);
		}
	}

	_sensors.imu.accel_scale = fix16_div(_fc_gravity, fix16_from_int(_sensor_calibration.data.accel.acc1G));	//Get the m/s scale (raw->g's->m/s/s)
	_sensors.imu.gyro_scale = fix16_from_float(MPU_GYRO_SCALE);	//Get radians scale (raw->rad/s)
}

void sensors_init_internal(void) {
	//==-- IMU-MPU6050
	sensors_init_imu();

	//==-- Calibrations
	sensors_cal_init();

	//==-- Timer
	clock_init();

	//==-- ADC
	adcInit(false);
}

void sensors_init_external(void) {
	//==-- Mag
	if( (bool)get_param_uint( PARAM_SENSOR_MAG_CBRK ) ) {
		mavlink_queue_broadcast_error("[SENSOR] Mag support is not stable!");

		sensor_status_init( &_sensors.mag.status, hmc5883lInit(get_param_uint( PARAM_BOARD_REVISION ) ) );

		//If we expected it to be present, but it failed
		if(!_sensors.mag.status.present)
			mavlink_queue_broadcast_error("[SENSOR] Unable to configure mag, disabling!");
	} else {
		sensor_status_init( &_sensors.mag.status, false );
	}

	_sensors.mag.period_update = fix16_to_int( fix16_div(_fc_1000, get_param_fix16(PARAM_SENSOR_SONAR_UPDATE_RATE)));

	_sensors.mag.raw.x = 0;
	_sensors.mag.raw.y = 0;
	_sensors.mag.raw.z = 0;
	_sensors.mag.scaled.x = 0;
	_sensors.mag.scaled.y = 0;
	_sensors.mag.scaled.z = 0;
	_sensors.mag.q.a = _fc_1;
	_sensors.mag.q.b = 0;
	_sensors.mag.q.c = 0;
	_sensors.mag.q.d = 0;

	//==-- Baro
	sensor_status_init( &_sensors.baro.status, (bool)get_param_uint( PARAM_SENSOR_BARO_CBRK ) );

	//==-- Sonar
	sensor_status_init( &_sensors.sonar.status, (bool)get_param_uint( PARAM_SENSOR_SONAR_CBRK ) );

	//==-- External Pose
	sensor_status_init( &_sensors.ext_pose.status, false );

	//==-- RC Input
	sensor_status_init( &_sensors.rc_input.status, true );
	_sensors.rc_input.p_r = 0;
	_sensors.rc_input.p_p = 0;
	_sensors.rc_input.p_y = 0;
	_sensors.rc_input.p_T = 0;
	_sensors.rc_input.p_m = 0;
	_sensors.rc_input.c_r = 0;
	_sensors.rc_input.c_p = 0;
	_sensors.rc_input.c_y = 0;
	_sensors.rc_input.c_T = 0;
	_sensors.rc_input.c_m = 0;
	sensors_update_rc_cal();

	//RC Safety toggle
	sensor_status_init( &_sensors.rc_safety_toggle.status, true );
	_sensors.rc_safety_toggle.arm_req_made = false;
	_sensors.rc_safety_toggle.timer_start_us = 0;

	//==-- Safety button
	sensor_status_init(&_sensors.safety_button.status, (bool)get_param_uint(PARAM_SENSOR_SAFETY_CBRK));
	_sensors.safety_button.gpio_p = GPIOA;
	_sensors.safety_button.pin = Pin_6;

	gpio_config_t safety_button_cfg;
    safety_button_cfg.pin = _sensors.safety_button.pin;
    safety_button_cfg.mode = Mode_IPU;
    safety_button_cfg.speed = Speed_2MHz;
    gpioInit(_sensors.safety_button.gpio_p, &safety_button_cfg);

	_sensors.safety_button.state = false;
	_sensors.safety_button.period_us = 100000;		//100ms update rate
	_sensors.safety_button.time_db_read = 0;
	_sensors.safety_button.period_db_us = 50000;	//50ms debounce period
	_sensors.safety_button.state_db = false;

	//==-- Voltage Monitor
	sensor_status_init(&_sensors.voltage_monitor.status, true);
	/*
	_sensors.safety_button.gpio_p = GPIOA;
	_sensors.safety_button.pin = Pin_4;

	gpio_config_t voltage_monitor_cfg;
    voltage_monitor_cfg.pin = _sensors.voltage_monitor.pin;
    voltage_monitor_cfg.mode = Mode_AIN;
    voltage_monitor_cfg.speed = Speed_2MHz;
    gpioInit(_sensors.voltage_monitor.gpio_p, &voltage_monitor_cfg);
	*/

	_sensors.voltage_monitor.state_raw = 0;
	_sensors.voltage_monitor.state_calc = 0;
	_sensors.voltage_monitor.state_filtered = 0;
}

bool i2c_job_queued(void) {
	bool a_done = (accel_status == I2C_JOB_DEFAULT) || (accel_status == I2C_JOB_COMPLETE);
	bool g_done = (gyro_status == I2C_JOB_DEFAULT) || (gyro_status == I2C_JOB_COMPLETE);
	bool t_done = (temp_status == I2C_JOB_DEFAULT) || (temp_status == I2C_JOB_COMPLETE);
	bool m_done = (mag_status == I2C_JOB_DEFAULT) || (mag_status == I2C_JOB_COMPLETE);

	return !a_done || !g_done || !t_done || !m_done;
}

bool sensors_read(void) {
	bool imu_job_complete = false;
	bool mag_job_complete = false;

	//Check IMU status
	if( (accel_status == I2C_JOB_COMPLETE) &&
		(gyro_status == I2C_JOB_COMPLETE) &&
		(temp_status == I2C_JOB_COMPLETE) ) {
		imu_job_complete = true;
		accel_status = I2C_JOB_DEFAULT;
		gyro_status = I2C_JOB_DEFAULT;
		temp_status = I2C_JOB_DEFAULT;

		_sensors.clock.imu_time_read = _imu_time_read;

		//==-- Save raw data
		//XXX: Some values need to be inversed to be in the NED frame
		_sensors.imu.accel_raw.x = -read_accel_raw[0];
		_sensors.imu.accel_raw.y = read_accel_raw[1];
		_sensors.imu.accel_raw.z = read_accel_raw[2];

		_sensors.imu.gyro_raw.x = read_gyro_raw[0];
		_sensors.imu.gyro_raw.y = -read_gyro_raw[1];
		_sensors.imu.gyro_raw.z = -read_gyro_raw[2];

		_sensors.imu.temp_raw = read_temp_raw;

		//==-- Handle raw data
		//Convert temperature SI units (degC, m/s^2, rad/s)
		// value = (_sensors.imu.temp_raw/temp_scale) + temp_shift
		_sensors.imu.temperature = fix16_add(fix16_div(fix16_from_int(_sensors.imu.temp_raw), _sensor_calibration.data.accel.temp_scale), _sensor_calibration.data.accel.temp_shift);

		//Accel
		//TODO: value = (raw - BIAS - (EMP_COMP * TEMP)) * scale
		// value = (raw - BIAS) * scale

		//Correct for measurement biases
		fix16_t accel_x_tmp = fix16_mul(fix16_from_int(_sensors.imu.accel_raw.x - get_param_int(PARAM_ACC_X_BIAS)), _sensors.imu.accel_scale);
		fix16_t accel_y_tmp = fix16_mul(fix16_from_int(_sensors.imu.accel_raw.y - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
		fix16_t accel_z_tmp = fix16_mul(fix16_from_int(_sensors.imu.accel_raw.z - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);

		//Scale the accelerometer to match 1G
		_sensors.imu.accel.x = fix16_mul(accel_x_tmp, ( accel_x_tmp > 0 ) ? get_param_fix16(PARAM_ACC_X_SCALE_POS) : get_param_fix16(PARAM_ACC_X_SCALE_NEG) );
		_sensors.imu.accel.y = fix16_mul(accel_y_tmp, ( accel_y_tmp > 0 ) ? get_param_fix16(PARAM_ACC_Y_SCALE_POS) : get_param_fix16(PARAM_ACC_Y_SCALE_NEG) );
		_sensors.imu.accel.z = fix16_mul(accel_z_tmp, ( accel_z_tmp > 0 ) ? get_param_fix16(PARAM_ACC_Z_SCALE_POS) : get_param_fix16(PARAM_ACC_Z_SCALE_NEG) );

		//Gyro
		// value = (raw - BIAS) * scale
		_sensors.imu.gyro.x = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw.x - get_param_int(PARAM_GYRO_X_BIAS)), _sensors.imu.gyro_scale);
		_sensors.imu.gyro.y = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw.y - get_param_int(PARAM_GYRO_Y_BIAS)), _sensors.imu.gyro_scale);
		_sensors.imu.gyro.z = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw.z - get_param_int(PARAM_GYRO_Z_BIAS)), _sensors.imu.gyro_scale);


		//Other IMU updates
		_sensors.imu.status.time_read = sensors_clock_imu_int_get();
		_sensors.imu.status.new_data = true;
		safety_update_sensor(&_system_status.sensors.imu);
	}

	if(mag_status == I2C_JOB_DEFAULT) {
		mag_job_complete = true;
	} else if(mag_status == I2C_JOB_COMPLETE) {
		mag_job_complete = true;

		mag_status = I2C_JOB_DEFAULT;

		//==-- Handle raw values
		//XXX: Some values need to be switched to be in the NED frame
		_sensors.mag.raw.x = read_mag_raw[0];
		_sensors.mag.raw.y = -read_mag_raw[1];
		_sensors.mag.raw.z = -read_mag_raw[2];

		_sensors.mag.scaled.x = fix16_div(fix16_from_int(_sensors.mag.raw.x),fix16_from_int(HMC5883L_GAIN_FACTOR));
		_sensors.mag.scaled.y = fix16_div(fix16_from_int(_sensors.mag.raw.y),fix16_from_int(HMC5883L_GAIN_FACTOR));
		_sensors.mag.scaled.z = fix16_div(fix16_from_int(_sensors.mag.raw.z),fix16_from_int(HMC5883L_GAIN_FACTOR));

		//TODO: Do remaining mag scaling / calibration steps

		//==-- Get a north estimate
		//Build a rotation matrix
		v3d mag_body_x;
		v3d_normalize(&mag_body_x, &_sensors.mag.scaled);
		v3d mag_body_y;
		v3d mag_body_z;
		v3d tmp_z;
		tmp_z.x = 0;
		tmp_z.y = 0;
		tmp_z.z = _fc_1;//(_sensors.imu.accel.z > 0) ? _fc_1 : -_fc_1; //XXX: Correct for the FC being upside-down

		v3d_cross(&mag_body_y, &tmp_z, &mag_body_x);
		v3d_normalize(&mag_body_y, &mag_body_y);

		v3d_cross(&mag_body_z, &mag_body_x, &mag_body_y);
		v3d_normalize(&mag_body_z, &mag_body_z);

		mf16 mag_body;
		dcm_from_basis(&mag_body, &mag_body_x, &mag_body_y, &mag_body_z);

		qf16 mag_q_body;
		matrix_to_qf16(&mag_q_body, &mag_body );
		qf16_normalize_to_unit(&mag_q_body, &mag_q_body);

		//De-rotate from the body back to the world
		qf16 q_tmp;
		qf16_inverse(&q_tmp, &mag_q_body);
		qf16_normalize_to_unit(&_sensors.mag.q, &q_tmp);

		//Other Mag updates
		_sensors.mag.status.time_read = micros();
		_sensors.mag.status.new_data = true;
		safety_update_sensor(&_system_status.sensors.mag);
	}

	//TODO: Check other status
	//TODO: May need to offset these so they don't all check at once(?)

	//Return the results
	return ( imu_job_complete && mag_job_complete );
}

uint32_t sensors_clock_ls_get(void) {
	return _sensors.clock.start;
}

void sensors_clock_ls_set(uint32_t time_us) {
	_sensors.clock.start = time_us;
}

void sensors_clock_update(uint32_t time_us) {
	_sensors.clock.end = time_us;
	_sensors.clock.dt = _sensors.clock.end - _sensors.clock.start;
	_sensors.clock.average_time += _sensors.clock.dt;
	_sensors.clock.counter++;
	_sensors.clock.max = (_sensors.clock.dt > _sensors.clock.max) ? _sensors.clock.dt : _sensors.clock.max;
	_sensors.clock.min = (_sensors.clock.dt < _sensors.clock.min) ? _sensors.clock.dt : _sensors.clock.min;
}

//==-- Low Pass Filter for time offsets
int64_t sensors_clock_smooth_time_skew(int64_t tc, int64_t tn) {
	/* The closer alpha is to 1.0, the faster the moving
	 * average updates in response to new offset samples.
	 */
	//Do this in floating point as fix16_t does not have an easy interface for uint64_t
	float alpha = fix16_to_float( get_param_fix16( PARAM_TIMESYNC_ALPHA ) );
	return (int64_t)( alpha * tn ) + (int64_t)( ( 1.0f - alpha ) * tc );
}

float sensors_clock_smooth_time_drift(float tc, float tn) {
	/* The closer alpha is to 1.0, the faster the moving
	 * average updates in response to new offset samples.
	 */
	//Do this in floating point as fix16_t does not have an easy interface for uint64_t
	float alpha = fix16_to_float( get_param_fix16( PARAM_TIMESYNC_ALPHA ) );
	return (int64_t)( alpha * tn ) + (int64_t)( ( 1.0f - alpha ) * tc );
}

uint64_t sensors_clock_rt_get(void) {
	return (uint64_t)( micros() * 1000LL ) + _sensors.clock.rt_offset_ns;
}

uint32_t sensors_clock_imu_int_get(void) {
	return _sensors.clock.imu_time_read;
}

static bool sensors_request_cal_state(void) {
	bool success = safety_request_state( MAV_STATE_CALIBRATING );

	if(!success) {
		mavlink_queue_broadcast_error("[SENSOR] Cannot enter calibration in this mode!");
	}

	return success;
}

bool sensors_request_cal(sensor_calibration_request_t req) {
	bool success = false;
	char text_reason[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

	switch(req) {
		case SENSOR_CAL_GYRO: {
			if( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {
				strncpy(text_reason,
						 "gyro",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		case SENSOR_CAL_MAG: {
			if( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {
				strncpy(text_reason,
						 "mag",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		case SENSOR_CAL_GND_PRESSURE: {
			if( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {
				strncpy(text_reason,
						 "gnd press",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		case SENSOR_CAL_RC: {
			if( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {

				if( !get_param_uint(PARAM_RC_MAP_ROLL) ||
					!get_param_uint(PARAM_RC_MAP_PITCH) ||
					!get_param_uint(PARAM_RC_MAP_YAW) ||
					!get_param_uint(PARAM_RC_MAP_THROTTLE) ) {

					mavlink_queue_broadcast_error("[SENSOR] RC mapping params no set");
				}

				strncpy(text_reason,
						 "rc",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		case SENSOR_CAL_ACCEL: {
			if( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {
				strncpy(text_reason,
						 "accel",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		case SENSOR_CAL_LEVEL_HORIZON: {
			if( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {
				strncpy(text_reason,
						 "level",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		case SENSOR_CAL_INTER: {
			//TODO: Need to check sensors required are present
			success = sensors_request_cal_state();

			break;
		}
		case SENSOR_CAL_BARO: {
			if( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
				success = sensors_request_cal_state();
			} else {
				strncpy(text_reason,
						 "baro",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}

			break;
		}
		default: {

			break;
		}
	}

	if(success) {
		_sensor_calibration.type = req;
	} else {
		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Cannot cal ";
		strncat(text, text_reason, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1);
		strncat(text, " no input detected!", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1);
		mavlink_queue_broadcast_error(text);
	}

	return success;
}

static void sensors_calibration_done(void) {
	safety_request_state( MAV_STATE_STANDBY );

	sensors_cal_init();	//Reset calibration data
}

static bool sensors_do_cal_gyro(void) {
	bool failed = false;

	_sensor_calibration.data.gyro.sum_x += _sensors.imu.gyro_raw.x;
	_sensor_calibration.data.gyro.sum_y += _sensors.imu.gyro_raw.y;
	_sensor_calibration.data.gyro.sum_z += _sensors.imu.gyro_raw.z;

	_sensor_calibration.data.gyro.count++;

	if (_sensor_calibration.data.gyro.count >= get_param_uint(PARAM_CAL_IMU_PASSES)) {
		int gyro_x_bias = _sensor_calibration.data.gyro.sum_x / _sensor_calibration.data.gyro.count;
		int gyro_y_bias = _sensor_calibration.data.gyro.sum_y / _sensor_calibration.data.gyro.count;
		int gyro_z_bias = _sensor_calibration.data.gyro.sum_z / _sensor_calibration.data.gyro.count;

		if( ( abs(gyro_x_bias) > GYRO_HIGH_BIAS_WARN ) ||
			( abs(gyro_y_bias) > GYRO_HIGH_BIAS_WARN ) ||
			( abs(gyro_z_bias) > GYRO_HIGH_BIAS_WARN ) ) {
			mavlink_queue_broadcast_error("[SENSOR] Warning: high gyro biases detected!");
		}

		set_param_int(PARAM_GYRO_X_BIAS, gyro_x_bias);
		set_param_int(PARAM_GYRO_Y_BIAS, gyro_y_bias);
		set_param_int(PARAM_GYRO_Z_BIAS, gyro_z_bias);

		reset_adaptive_gyro_bias();

		sensors_calibration_done();
		mavlink_queue_broadcast_notice("[SENSOR] Gyro calibration complete!");
	}

	return !failed;
}

static bool sensors_do_cal_mag(void) {
	bool failed = false;

	//TODO CAL MAG
	sensors_calibration_done();

	return !failed;
}

static bool sensors_do_cal_gnd_press(void) {
	bool failed = false;

	//TODO CAL BAROMETER
	sensors_calibration_done();

	return !failed;
}

static bool sensors_do_cal_rc(void) {
	bool failed = false;

	if(!_sensor_calibration.data.rc.waiting) {
		switch( _sensor_calibration.data.rc.step ) {
			case SENSOR_CAL_RC_RANGE_INIT: {
				_sensor_calibration.data.rc.waiting = true;
				_sensor_calibration.data.rc.step = SENSOR_CAL_RC_RANGE_MIDDOWN;
				mavlink_queue_broadcast_notice("[SENSOR] Set RC to stick and switch centres");

				break;
			}
			case SENSOR_CAL_RC_RANGE_MIDDOWN: {
				for(int i=0;i<8;i++) {
					_sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MID] = pwmRead(i);

					if( ( pwmRead(i) < 1300 ) || ( pwmRead(i) > 1700 ) ) {
						char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Possible bad trim on channel ";
						char mchar[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						itoa(i + 1, mchar, 10);
						strncat(text, mchar, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1);
						mavlink_queue_broadcast_error(text);
					}
				}

				//XXX Set the params here as it saves overloading the LPQ
				set_param_uint( PARAM_RC1_MID, _sensor_calibration.data.rc.rc_ranges[0][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC2_MID, _sensor_calibration.data.rc.rc_ranges[1][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC3_MID, _sensor_calibration.data.rc.rc_ranges[2][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC4_MID, _sensor_calibration.data.rc.rc_ranges[3][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC5_MID, _sensor_calibration.data.rc.rc_ranges[4][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC6_MID, _sensor_calibration.data.rc.rc_ranges[5][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC7_MID, _sensor_calibration.data.rc.rc_ranges[6][SENSOR_RC_CAL_MID]);
				set_param_uint( PARAM_RC8_MID, _sensor_calibration.data.rc.rc_ranges[7][SENSOR_RC_CAL_MID]);

				_sensor_calibration.data.rc.waiting = true;
				_sensor_calibration.data.rc.step = SENSOR_CAL_RC_RANGE_CORNERS;
				mavlink_queue_broadcast_notice("[SENSOR] Set RC to lower inner corners");

				break;
			}
			case SENSOR_CAL_RC_RANGE_CORNERS : {
				uint8_t chan_roll = get_param_uint(PARAM_RC_MAP_ROLL) - 1;
				uint8_t chan_pitch = get_param_uint(PARAM_RC_MAP_PITCH) - 1;
				uint8_t chan_yaw = get_param_uint(PARAM_RC_MAP_YAW) - 1;
				uint8_t chan_throttle = get_param_uint(PARAM_RC_MAP_THROTTLE) - 1;

				_sensor_calibration.data.rc.rc_rev[chan_roll] = (pwmRead(chan_roll) > SENSOR_RC_MIDSTICK);
				_sensor_calibration.data.rc.rc_rev[chan_pitch] = (pwmRead(chan_pitch) < SENSOR_RC_MIDSTICK);
				_sensor_calibration.data.rc.rc_rev[chan_yaw] = (pwmRead(chan_yaw) < SENSOR_RC_MIDSTICK);
				_sensor_calibration.data.rc.rc_rev[chan_throttle] = (pwmRead(chan_throttle) > SENSOR_RC_MIDSTICK);

				//XXX Set the params here as it saves overloading the LPQ
				set_param_uint( PARAM_RC1_REV, _sensor_calibration.data.rc.rc_rev[0]);
				set_param_uint( PARAM_RC2_REV, _sensor_calibration.data.rc.rc_rev[1]);
				set_param_uint( PARAM_RC3_REV, _sensor_calibration.data.rc.rc_rev[2]);
				set_param_uint( PARAM_RC4_REV, _sensor_calibration.data.rc.rc_rev[3]);
				set_param_uint( PARAM_RC5_REV, _sensor_calibration.data.rc.rc_rev[4]);
				set_param_uint( PARAM_RC6_REV, _sensor_calibration.data.rc.rc_rev[5]);
				set_param_uint( PARAM_RC7_REV, _sensor_calibration.data.rc.rc_rev[6]);
				set_param_uint( PARAM_RC8_REV, _sensor_calibration.data.rc.rc_rev[7]);

				_sensor_calibration.data.rc.step = SENSOR_CAL_RC_RANGE_EXTREMES;
				mavlink_queue_broadcast_notice("[SENSOR] Move all sticks and switches to extremes");
				mavlink_queue_broadcast_notice("[SENSOR] Resend cal command when done");

				break;
			}
			case SENSOR_CAL_RC_RANGE_EXTREMES: {
				for(int i=0;i<8;i++) {
					uint16_t pwmr = pwmRead(i);
					_sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MIN] = ( pwmr < _sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MIN] ) ? pwmr : _sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MIN];
					_sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MAX] = ( pwmr > _sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MAX] ) ? pwmr : _sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MAX];
				}
				break;
			}
			case SENSOR_CAL_RC_RANGE_DONE: {
				for(int i=0;i<8;i++) {
					if( ( _sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MIN] > 1300 ) ||
						( _sensor_calibration.data.rc.rc_ranges[i][SENSOR_RC_CAL_MAX] < 1700 ) ) {

						char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Possible bad min/max on channel ";
						char mchar[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						itoa(i + 1, mchar, 10);
						strncat(text, mchar, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1);
						mavlink_queue_broadcast_error(text);
					}
				}

				set_param_uint( PARAM_RC1_MIN, _sensor_calibration.data.rc.rc_ranges[0][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC1_MAX, _sensor_calibration.data.rc.rc_ranges[0][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC2_MIN, _sensor_calibration.data.rc.rc_ranges[1][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC2_MAX, _sensor_calibration.data.rc.rc_ranges[1][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC3_MIN, _sensor_calibration.data.rc.rc_ranges[2][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC3_MAX, _sensor_calibration.data.rc.rc_ranges[2][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC4_MIN, _sensor_calibration.data.rc.rc_ranges[3][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC4_MAX, _sensor_calibration.data.rc.rc_ranges[3][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC5_MIN, _sensor_calibration.data.rc.rc_ranges[4][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC5_MAX, _sensor_calibration.data.rc.rc_ranges[4][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC6_MIN, _sensor_calibration.data.rc.rc_ranges[5][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC6_MAX, _sensor_calibration.data.rc.rc_ranges[5][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC7_MIN, _sensor_calibration.data.rc.rc_ranges[6][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC7_MAX, _sensor_calibration.data.rc.rc_ranges[6][SENSOR_RC_CAL_MAX]);
				set_param_uint( PARAM_RC8_MIN, _sensor_calibration.data.rc.rc_ranges[7][SENSOR_RC_CAL_MIN]);
				set_param_uint( PARAM_RC8_MAX, _sensor_calibration.data.rc.rc_ranges[7][SENSOR_RC_CAL_MAX]);

				sensors_calibration_done();
				mavlink_queue_broadcast_notice("[SENSOR] RC calibration complete!");

				break;
			}
			default: {
				failed = true;

				sensors_calibration_done();
				mavlink_queue_broadcast_error("[SENSOR] Issue with RC cal, aborting");

				break;
			}
		}

		//If we are waiting inside this loop, a step has recently finished
		if(_sensor_calibration.data.rc.waiting) {
			mavlink_message_t msg;
			mavlink_prepare_command_ack(&msg,
										MAV_CMD_PREFLIGHT_CALIBRATION,
										MAV_RESULT_IN_PROGRESS,
										_sensor_calibration.req_sysid,
										_sensor_calibration.req_compid,
										_sensor_calibration.data.rc.step / SENSOR_CAL_ACCEL_DONE);
			lpq_queue_broadcast_msg(&msg);

			status_buzzer_success();
		}
	}

	return !failed;
}

static bool sensors_do_cal_accel(void) {
	bool failed = false;

	if(!_sensor_calibration.data.accel.waiting) {
		if( _sensor_calibration.data.accel.accel_cal_step == SENSOR_CAL_ACCEL_INIT ) {
			_sensor_calibration.data.accel.waiting = true;
			_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Z_DOWN;
			mavlink_queue_broadcast_notice("[SENSOR] Ready for Z-Down, send accel cal");
		} else if ( _sensor_calibration.data.accel.accel_cal_step == SENSOR_CAL_ACCEL_DONE ) {
			//==-- bias = sum / count
			//==-- //TODO: bias = (sum - (temp_comp*temp_sum)) / count
			int32_t x_bias = _sensor_calibration.data.accel.data.x_flat_av_sum / 4;
			int32_t y_bias = _sensor_calibration.data.accel.data.y_flat_av_sum / 4;
			int32_t z_bias = _sensor_calibration.data.accel.data.z_flat_av_sum / 4;


			//Correct for measurement biases
			fix16_t accel_x_down_1g = fix16_mul(fix16_from_int(_sensor_calibration.data.accel.data.x_down_av - get_param_int(PARAM_ACC_X_BIAS)), _sensors.imu.accel_scale);
			fix16_t accel_y_down_1g = fix16_mul(fix16_from_int(_sensor_calibration.data.accel.data.y_down_av - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
			fix16_t accel_z_down_1g = fix16_mul(fix16_from_int(_sensor_calibration.data.accel.data.z_down_av - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);
			fix16_t accel_x_up_1g = fix16_mul(fix16_from_int(_sensor_calibration.data.accel.data.x_up_av - get_param_int(PARAM_ACC_X_BIAS)), _sensors.imu.accel_scale);
			fix16_t accel_y_up_1g = fix16_mul(fix16_from_int(_sensor_calibration.data.accel.data.y_up_av - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
			fix16_t accel_z_up_1g = fix16_mul(fix16_from_int(_sensor_calibration.data.accel.data.z_up_av - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);

			fix16_t accel_x_scale_p = fix16_div( _fc_gravity, accel_x_down_1g );
			fix16_t accel_y_scale_p = fix16_div( _fc_gravity, accel_y_down_1g );
			fix16_t accel_z_scale_p = fix16_div( _fc_gravity, accel_z_down_1g );
			fix16_t accel_x_scale_n = fix16_div( -_fc_gravity, accel_x_up_1g );
			fix16_t accel_y_scale_n = fix16_div( -_fc_gravity, accel_y_up_1g );
			fix16_t accel_z_scale_n = fix16_div( -_fc_gravity, accel_z_up_1g );

			//Sanity check to make sure the scaling is positive and not far too large
			if( ( ( accel_x_scale_p > _fc_0_5 ) && ( accel_x_scale_p < _fc_2 ) ) &&
				( ( accel_y_scale_p > _fc_0_5 ) && ( accel_y_scale_p < _fc_2 ) ) &&
				( ( accel_z_scale_p > _fc_0_5 ) && ( accel_z_scale_p < _fc_2 ) ) &&
				( ( accel_x_scale_n > _fc_0_5 ) && ( accel_x_scale_n < _fc_2 ) ) &&
				( ( accel_y_scale_n > _fc_0_5 ) && ( accel_y_scale_n < _fc_2 ) ) &&
				( ( accel_z_scale_n > _fc_0_5 ) && ( accel_z_scale_n < _fc_2 ) ) ) {

				set_param_int( PARAM_ACC_X_BIAS, x_bias );
				set_param_int( PARAM_ACC_Y_BIAS, y_bias );
				set_param_int( PARAM_ACC_Z_BIAS, z_bias );

				set_param_fix16( PARAM_ACC_X_SCALE_POS, accel_x_scale_p );
				set_param_fix16( PARAM_ACC_Y_SCALE_POS, accel_y_scale_p );
				set_param_fix16( PARAM_ACC_Z_SCALE_POS, accel_z_scale_p );
				set_param_fix16( PARAM_ACC_X_SCALE_NEG, accel_x_scale_n );
				set_param_fix16( PARAM_ACC_Y_SCALE_NEG, accel_y_scale_n );
				set_param_fix16( PARAM_ACC_Z_SCALE_NEG, accel_z_scale_n );

				mavlink_queue_broadcast_notice("[SENSOR] Accel calibration complete!");
			} else {
				failed = true;

				mavlink_queue_broadcast_error("[SENSOR] Accel calibration failed, bad scaling!");
			}

			sensors_calibration_done();
		} else {
			_sensor_calibration.data.accel.data.t_sum += _sensors.imu.temp_raw;
			_sensor_calibration.data.accel.data.x_sum += _sensors.imu.accel_raw.x;
			_sensor_calibration.data.accel.data.y_sum += _sensors.imu.accel_raw.y;
			_sensor_calibration.data.accel.data.z_sum += _sensors.imu.accel_raw.z;

			_sensor_calibration.data.accel.data.count++;

			if (_sensor_calibration.data.accel.data.count >= get_param_uint(PARAM_CAL_IMU_PASSES)) {
				switch(_sensor_calibration.data.accel.accel_cal_step) {
					case SENSOR_CAL_ACCEL_Z_DOWN: {
						_sensor_calibration.data.accel.data.t_av_sum += _sensor_calibration.data.accel.data.t_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.x_flat_av_sum += _sensor_calibration.data.accel.data.x_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.y_flat_av_sum += _sensor_calibration.data.accel.data.y_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.z_down_av = _sensor_calibration.data.accel.data.z_sum / _sensor_calibration.data.accel.data.count;

						_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Z_UP;
						mavlink_queue_broadcast_notice("[SENSOR] Ready for Z-Up, send accel cal");

						break;
					}
					case SENSOR_CAL_ACCEL_Z_UP: {
						_sensor_calibration.data.accel.data.t_av_sum += _sensor_calibration.data.accel.data.t_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.x_flat_av_sum += _sensor_calibration.data.accel.data.x_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.y_flat_av_sum += _sensor_calibration.data.accel.data.y_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.z_up_av = _sensor_calibration.data.accel.data.z_sum / _sensor_calibration.data.accel.data.count;

						_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Y_DOWN;
						mavlink_queue_broadcast_notice("[SENSOR] Ready for Y-Down, send accel cal");

						break;
					}
					case SENSOR_CAL_ACCEL_Y_DOWN: {
						_sensor_calibration.data.accel.data.t_av_sum += _sensor_calibration.data.accel.data.t_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.x_flat_av_sum += _sensor_calibration.data.accel.data.x_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.y_down_av = _sensor_calibration.data.accel.data.y_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.z_flat_av_sum += _sensor_calibration.data.accel.data.z_sum / _sensor_calibration.data.accel.data.count;

						_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Y_UP;
						mavlink_queue_broadcast_notice("[SENSOR] Ready for Y-Up, send accel cal");

						break;
					}
					case SENSOR_CAL_ACCEL_Y_UP: {
						_sensor_calibration.data.accel.data.t_av_sum += _sensor_calibration.data.accel.data.t_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.x_flat_av_sum += _sensor_calibration.data.accel.data.x_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.y_up_av = _sensor_calibration.data.accel.data.y_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.z_flat_av_sum += _sensor_calibration.data.accel.data.z_sum / _sensor_calibration.data.accel.data.count;

						_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_X_DOWN;
						mavlink_queue_broadcast_notice("[SENSOR] Ready for X-Down, send accel cal");

						break;
					}
					case SENSOR_CAL_ACCEL_X_DOWN: {
						_sensor_calibration.data.accel.data.t_av_sum += _sensor_calibration.data.accel.data.t_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.x_down_av = _sensor_calibration.data.accel.data.x_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.y_flat_av_sum += _sensor_calibration.data.accel.data.y_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.z_flat_av_sum += _sensor_calibration.data.accel.data.z_sum / _sensor_calibration.data.accel.data.count;

						_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_X_UP;
						mavlink_queue_broadcast_notice("[SENSOR] Ready for X-Up, send accel cal");

						break;
					}
					case SENSOR_CAL_ACCEL_X_UP: {
						_sensor_calibration.data.accel.data.t_av_sum += _sensor_calibration.data.accel.data.t_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.x_up_av = _sensor_calibration.data.accel.data.x_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.y_flat_av_sum += _sensor_calibration.data.accel.data.y_sum / _sensor_calibration.data.accel.data.count;
						_sensor_calibration.data.accel.data.z_flat_av_sum += _sensor_calibration.data.accel.data.z_sum / _sensor_calibration.data.accel.data.count;

						_sensor_calibration.data.accel.accel_cal_step = SENSOR_CAL_ACCEL_DONE;

						break;
					}
					default: {
						failed = true;

						sensors_calibration_done();
						mavlink_queue_broadcast_error("[SENSOR] Issue with accel cal, aborting");

						break;
					}
				}

				mavlink_message_t msg;
				mavlink_prepare_command_ack(&msg,
											MAV_CMD_PREFLIGHT_CALIBRATION,
											MAV_RESULT_IN_PROGRESS,
											_sensor_calibration.req_sysid,
											_sensor_calibration.req_compid,
											_sensor_calibration.data.accel.accel_cal_step / SENSOR_CAL_ACCEL_DONE);
				lpq_queue_broadcast_msg(&msg);

				status_buzzer_success();

				//Reset for the next calibration
				_sensor_calibration.data.accel.data.count = 0;
				_sensor_calibration.data.accel.data.x_sum = 0;
				_sensor_calibration.data.accel.data.y_sum = 0;
				_sensor_calibration.data.accel.data.z_sum = 0;
				_sensor_calibration.data.accel.data.t_sum = 0;

				//Make sure we wait for confirmation before continuing
				if(_sensor_calibration.data.accel.accel_cal_step != SENSOR_CAL_ACCEL_DONE)
					_sensor_calibration.data.accel.waiting = true;
			}
		}
	}

	return !failed;
}

static bool sensors_do_cal_level_horizon(void) {
	bool failed = false;

	//TODO CAL LEVEL HORIZON
	sensors_calibration_done();

	return !failed;
}

static bool sensors_do_cal_inter(void) {
	bool failed = false;

	//TODO CAL INTERFERENCE
	sensors_calibration_done();

	return !failed;
}

static bool sensors_do_cal_baro(void) {
	bool failed = false;

	//TODO CAL BAROMETER
	sensors_calibration_done();

	return !failed;
}

//TODO: This does not take into account temperature
//Returns true if all calibrations are complete
static void sensors_calibrate(void) {
	bool cal_mode_error = false;

	//If we actually need to do a calibration
	if(_sensor_calibration.type != SENSOR_CAL_NONE) {
		switch(_sensor_calibration.type) {
			case SENSOR_CAL_GYRO: {
				cal_mode_error = sensors_do_cal_gyro();

				break;
			}
			case SENSOR_CAL_MAG: {
				cal_mode_error = sensors_do_cal_mag();

				break;
			}
			case SENSOR_CAL_GND_PRESSURE: {
				cal_mode_error = sensors_do_cal_gnd_press();

				break;
			}
			case SENSOR_CAL_RC: {
				cal_mode_error = sensors_do_cal_rc();

				break;
			}
			case SENSOR_CAL_ACCEL: {
				cal_mode_error = sensors_do_cal_accel();

				break;
			}
			case SENSOR_CAL_LEVEL_HORIZON: {
				cal_mode_error = sensors_do_cal_level_horizon();

				break;
			}
			case SENSOR_CAL_INTER: {
				cal_mode_error = sensors_do_cal_inter();

				break;
			}
			case SENSOR_CAL_BARO: {
				cal_mode_error = sensors_do_cal_baro();

				break;
			}
			default: {
				sensors_calibration_done();
				cal_mode_error = true;
				mavlink_queue_broadcast_error("[SENSOR] Invalid calibration mode, clearing");

				break;
			}
		}

		//If there was a calibration running, but it finished this pass
		if( _sensor_calibration.type == SENSOR_CAL_NONE ) {
			mavlink_message_t msg;

			if( _system_status.state == MAV_STATE_CALIBRATING ) {
				sensors_calibration_done();
				mavlink_queue_broadcast_error("[SENSOR] Invalid calibration state, clearing");
			}

			if(cal_mode_error) {
				//Send a message saying that it has failed
				mavlink_prepare_command_ack(&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_FAILED, _sensor_calibration.req_sysid, _sensor_calibration.req_compid, 0xFF);
				status_buzzer_failure();
			} else {
				//Send a message saying that it has completed 100%
				mavlink_prepare_command_ack(&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_IN_PROGRESS, _sensor_calibration.req_sysid, _sensor_calibration.req_compid, 100);
				status_buzzer_success();
			}

			lpq_queue_broadcast_msg(&msg);
		}
	}
}

static fix16_t dual_normalized_input(uint16_t pwm, uint16_t min, uint16_t mid, uint16_t max) {
	//Constrain from min to max
	int16_t pwmc = (pwm < min) ? min : (pwm > max) ? max : pwm;
	fix16_t pwmn = 0;

	//
	if(pwmc > mid) {
		pwmn = fix16_div(fix16_from_int(pwmc - mid), fix16_from_int(max - mid));
	} else if(pwmc < mid) {
		pwmn = -fix16_div(fix16_from_int(mid - pwmc), fix16_from_int(mid - min));
	} else {
		//Stick is perfectly centered
		pwmn = 0;
	}

	//Sanity check our dual normalize incase we have bad min/max params
	pwmn = ((pwmn < -_fc_1) || (pwmn > _fc_1)) ? 0 : pwmn;

	//XXX: Could do deadzone here, but it would mean throttle has a deadzone at 50% instead of 0
	//XXX: Could also do another deadzone in normalized_input()

	//dual normalized (-1 -> 1), 0 if error
	return pwmn;
}

static fix16_t normalized_input(uint16_t pwm, uint16_t min, uint16_t mid, uint16_t max) {
	fix16_t dni = dual_normalized_input(pwm, min, mid, max);
	//scale (-1 -> 1) to (0 -> 1), just renormalize with a range of 2
	return fix16_div(fix16_add(dni, _fc_1), _fc_2);
}

void sensors_update_rc_cal(void) {
	rc_cal[0][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC1_MIN);
	rc_cal[0][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC1_MID);
	rc_cal[0][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC1_MAX);
	rc_cal[1][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC2_MIN);
	rc_cal[1][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC2_MID);
	rc_cal[1][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC2_MAX);
	rc_cal[2][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC3_MIN);
	rc_cal[2][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC3_MID);
	rc_cal[2][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC3_MAX);
	rc_cal[3][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC4_MIN);
	rc_cal[3][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC4_MID);
	rc_cal[3][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC4_MAX);
	rc_cal[4][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC5_MIN);
	rc_cal[4][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC5_MID);
	rc_cal[4][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC5_MAX);
	rc_cal[5][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC6_MIN);
	rc_cal[5][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC6_MID);
	rc_cal[5][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC6_MAX);
	rc_cal[6][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC7_MIN);
	rc_cal[6][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC7_MID);
	rc_cal[6][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC7_MAX);
	rc_cal[7][SENSOR_RC_CAL_MIN] = get_param_uint(PARAM_RC8_MIN);
	rc_cal[7][SENSOR_RC_CAL_MID] = get_param_uint(PARAM_RC8_MID);
	rc_cal[7][SENSOR_RC_CAL_MAX] = get_param_uint(PARAM_RC8_MAX);

	rc_rev[0] = get_param_uint(PARAM_RC1_REV);
	rc_rev[1] = get_param_uint(PARAM_RC2_REV);
	rc_rev[2] = get_param_uint(PARAM_RC3_REV);
	rc_rev[3] = get_param_uint(PARAM_RC4_REV);
	rc_rev[4] = get_param_uint(PARAM_RC5_REV);
	rc_rev[5] = get_param_uint(PARAM_RC6_REV);
	rc_rev[6] = get_param_uint(PARAM_RC7_REV);
	rc_rev[7] = get_param_uint(PARAM_RC8_REV);

	rc_dz[0] = get_param_fix16(PARAM_RC1_DZ);
	rc_dz[1] = get_param_fix16(PARAM_RC2_DZ);
	rc_dz[2] = get_param_fix16(PARAM_RC3_DZ);
	rc_dz[3] = get_param_fix16(PARAM_RC4_DZ);
	rc_dz[4] = get_param_fix16(PARAM_RC5_DZ);
	rc_dz[5] = get_param_fix16(PARAM_RC6_DZ);
	rc_dz[6] = get_param_fix16(PARAM_RC7_DZ);
	rc_dz[7] = get_param_fix16(PARAM_RC8_DZ);
}

bool sensors_update(uint32_t time_us) {
	//bool update_success = false;
	//TODO: Remember not to expect all sensors to be ready

	//==-- Update IMU
	//XXX: Nothing to do, but could do a present check for posix if we go there

	//Mag
	if(_sensors.mag.status.present) {
		//Update the sensor if it's time (and it's not currently reading)
		if( ( (time_us - _sensors.mag.status.time_read) > _sensors.mag.period_update ) &&
			(mag_status == I2C_JOB_DEFAULT) ) {
			hmc5883l_request_async_read(read_mag_raw, &mag_status);
		}
	}

	//==-- RC Input & Saftety Toggle
	//Check that all channels have been set
	if( get_param_uint(PARAM_RC_MAP_ROLL) &&
		get_param_uint(PARAM_RC_MAP_PITCH) &&
		get_param_uint(PARAM_RC_MAP_YAW) &&
		get_param_uint(PARAM_RC_MAP_THROTTLE) ) {

		uint8_t chan_roll = get_param_uint(PARAM_RC_MAP_ROLL) - 1;
		uint8_t chan_pitch = get_param_uint(PARAM_RC_MAP_PITCH) - 1;
		uint8_t chan_yaw = get_param_uint(PARAM_RC_MAP_YAW) - 1;
		uint8_t chan_throttle = get_param_uint(PARAM_RC_MAP_THROTTLE) - 1;

		_sensors.rc_input.p_r = pwmRead(chan_roll);
		_sensors.rc_input.p_p = pwmRead(chan_pitch);
		_sensors.rc_input.p_y = pwmRead(chan_yaw);
		_sensors.rc_input.p_T = pwmRead(chan_throttle);

		if( (_sensors.rc_input.p_r > 0) &&
			(_sensors.rc_input.p_p > 0) &&
			(_sensors.rc_input.p_y > 0) &&
			(_sensors.rc_input.p_T > 0) ) {
			//We have a valid reading
			_sensors.rc_input.status.time_read = time_us;
			_sensors.rc_input.status.new_data = true;
			safety_update_sensor(&_system_status.sensors.rc_input);

			//Normailize readings
			fix16_t cmd_roll = dual_normalized_input(_sensors.rc_input.p_r,
														rc_cal[chan_roll][SENSOR_RC_CAL_MIN],
														rc_cal[chan_roll][SENSOR_RC_CAL_MID],
														rc_cal[chan_roll][SENSOR_RC_CAL_MAX]);
			fix16_t cmd_pitch = dual_normalized_input(_sensors.rc_input.p_p,
														rc_cal[chan_pitch][SENSOR_RC_CAL_MIN],
														rc_cal[chan_pitch][SENSOR_RC_CAL_MID],
														rc_cal[chan_pitch][SENSOR_RC_CAL_MAX]);
			fix16_t cmd_yaw = dual_normalized_input(_sensors.rc_input.p_y,
														rc_cal[chan_yaw][SENSOR_RC_CAL_MIN],
														rc_cal[chan_yaw][SENSOR_RC_CAL_MID],
														rc_cal[chan_yaw][SENSOR_RC_CAL_MAX]);
			fix16_t cmd_throttle = normalized_input(_sensors.rc_input.p_T,
														rc_cal[chan_throttle][SENSOR_RC_CAL_MIN],
														rc_cal[chan_throttle][SENSOR_RC_CAL_MID],
														rc_cal[chan_throttle][SENSOR_RC_CAL_MAX]);

			//Correct for axis reverse
			fix16_t cmd_roll_corrected = (rc_rev[chan_roll]) ? fix16_mul(-_fc_1, cmd_roll) : cmd_roll;
			fix16_t cmd_pitch_corrected = (rc_rev[chan_pitch]) ? fix16_mul(-_fc_1, cmd_pitch) : cmd_pitch;
			fix16_t cmd_yaw_corrected = (rc_rev[chan_yaw]) ? fix16_mul(-_fc_1, cmd_yaw) : cmd_yaw;
			//XXX:For throttle, we need to multiply then shift it to ensure it is still 0->1
			fix16_t cmd_throttle_corrected = (rc_rev[chan_throttle]) ? fix16_add(_fc_1, fix16_mul(-_fc_1, cmd_throttle)) : cmd_throttle;

			//Calculate deadzones and save outputs
			_sensors.rc_input.c_r = (fix16_abs(cmd_roll_corrected) < rc_dz[chan_roll]) ? 0 : cmd_roll_corrected;
			_sensors.rc_input.c_p = (fix16_abs(cmd_pitch_corrected) < rc_dz[chan_pitch]) ? 0 : cmd_pitch_corrected;
			_sensors.rc_input.c_y = (fix16_abs(cmd_yaw_corrected) < rc_dz[chan_yaw]) ? 0 : cmd_yaw_corrected;
			_sensors.rc_input.c_T = (fix16_abs(cmd_throttle_corrected) < rc_dz[chan_throttle]) ? 0 : cmd_throttle_corrected;
		}

		//Only do this is the RC is healthy
		if( (_system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK) &&
			(_system_status.state != MAV_STATE_CALIBRATING) ) {
			//If we're using a mode switch
			if( get_param_uint(PARAM_RC_MAP_MODE_SW) ) {
				_sensors.rc_input.p_m = pwmRead(get_param_uint(PARAM_RC_MAP_MODE_SW) - 1);

				if( _sensors.rc_input.p_m > 0 ) {
					compat_px4_main_mode_t c_m_last = _sensors.rc_input.c_m;

					if( abs( (int16_t)get_param_uint(PARAM_RC_MODE_PWM_STAB) - _sensors.rc_input.p_m ) < get_param_uint(PARAM_RC_MODE_PWM_RANGE) ) {
						_sensors.rc_input.c_m = MAIN_MODE_STABILIZED;
					} else if( abs( (int16_t)get_param_uint(PARAM_RC_MODE_PWM_ACRO) - _sensors.rc_input.p_m ) < get_param_uint(PARAM_RC_MODE_PWM_RANGE) ) {
						_sensors.rc_input.c_m = MAIN_MODE_ACRO;
					} else if( abs( (int16_t)get_param_uint(PARAM_RC_MODE_PWM_OFFBOARD) - _sensors.rc_input.p_m ) < get_param_uint(PARAM_RC_MODE_PWM_RANGE) ) {
						_sensors.rc_input.c_m = MAIN_MODE_OFFBOARD;
					} //Else no mode select match, don't change mode

					//If a new mode is requested
					if( _sensors.rc_input.c_m != c_m_last ) {
						if( !safety_request_control_mode(_sensors.rc_input.c_m) ) {
							mavlink_queue_broadcast_error("[SENSOR] RC mode switch rejected");
						}
					}
				}
			} else if( get_param_uint(PARAM_RC_MODE_DEFAULT) ) { //Else if RC should trigger a default mode
				if( !safety_request_control_mode(get_param_uint(PARAM_RC_MODE_DEFAULT)) ) {
					mavlink_queue_broadcast_error("[SENSOR] Error setting RC default mode");
				}
			}

			//Handle the logic for saftey toggling
			_sensors.rc_safety_toggle.status.time_read = time_us;
			_sensors.rc_safety_toggle.status.new_data = true;

			if( (_sensors.rc_input.c_T < _fc_0_05) &&
				(fix16_abs(_sensors.rc_input.c_y) > _fc_0_95) ) {

				if(_sensors.rc_safety_toggle.timer_start_us == 0)
					_sensors.rc_safety_toggle.timer_start_us = time_us;

				if(time_us > (_sensors.rc_safety_toggle.timer_start_us + get_param_uint(PARAM_RC_ARM_TIMER) ) ) {
					if(!_sensors.rc_safety_toggle.arm_req_made) {
						if(_sensors.rc_input.c_y < 0) {
							safety_request_disarm();
						} else {
							safety_request_arm();
						}

						_sensors.rc_safety_toggle.arm_req_made = true;
					}
				}

			} else {
				_sensors.rc_safety_toggle.arm_req_made = false;
				_sensors.rc_safety_toggle.timer_start_us = 0;
			}
		}
	}


	//==-- Safety Button
	bool safety_button_reading = false;

	safety_button_reading = digitalIn(_sensors.safety_button.gpio_p, _sensors.safety_button.pin);

	if(safety_button_reading != _sensors.safety_button.state_db )
		_sensors.safety_button.time_db_read = time_us;

	if( ( time_us - _sensors.safety_button.time_db_read ) > _sensors.safety_button.period_db_us ) {
		if(safety_button_reading != _sensors.safety_button.state) {	//The reading has changed
			_sensors.safety_button.state = safety_button_reading;

			_sensors.safety_button.status.time_read = time_us;
			_sensors.safety_button.status.new_data = true;
		}
	}

	_sensors.safety_button.state_db = safety_button_reading;

	//==-- Voltage Monitor
	if(get_param_uint(PARAM_BATTERY_CELL_NUM) > 0) {
		//_sensors.voltage_monitor.state_raw = digitalIn(_sensors.voltage_monitor.gpio_p, _sensors.voltage_monitor.pin);
		_sensors.voltage_monitor.state_raw = adcGetChannel(ADC_EXTERNAL_PAD);

		fix16_t voltage_res = fix16_div(fix16_from_int(0xFFF), _fc_3_3 );	//XXX: 0xFFF is from 12Bit adc
		//XXX: TODO: Should lookup board rev properly
		fix16_t voltage_div = ( get_param_fix16(PARAM_BATTERY_DIVIDER) == -_fc_1 ) ? SENSOR_VMON_DIVIDER_NAZE32 : get_param_fix16(PARAM_BATTERY_DIVIDER);

		_sensors.voltage_monitor.state_calc = fix16_mul(fix16_div(fix16_from_int(_sensors.voltage_monitor.state_raw), voltage_res), voltage_div);
		//Filter reading: value_lpf = ((1 - alpha) * value) + (alpha * value_lpf);
		fix16_t voltage_alpha = get_param_fix16(PARAM_BATTERY_READING_FILTER);
		_sensors.voltage_monitor.state_filtered = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, voltage_alpha), _sensors.voltage_monitor.state_calc), fix16_smul(voltage_alpha, _sensors.voltage_monitor.state_filtered));
		//Calculate percentage left
		fix16_t voltage_min = fix16_mul( fix16_from_int(get_param_uint(PARAM_BATTERY_CELL_NUM)), get_param_fix16(PARAM_BATTERY_CELL_MIN) );
		fix16_t voltage_max = fix16_mul( fix16_from_int(get_param_uint(PARAM_BATTERY_CELL_NUM)), get_param_fix16(PARAM_BATTERY_CELL_MAX) );
		fix16_t voltage_range = fix16_sub(voltage_max, voltage_min);
		//Only calc percentage if in range
		if( ( _sensors.voltage_monitor.state_filtered > voltage_min ) &&
			( _sensors.voltage_monitor.state_filtered < voltage_max ) ) {
			_sensors.voltage_monitor.precentage = fix16_div(fix16_sub(_sensors.voltage_monitor.state_filtered, voltage_min), voltage_range);
		} else {
			_sensors.voltage_monitor.precentage = 0;
		}

		_sensors.voltage_monitor.status.time_read = time_us;
		_sensors.voltage_monitor.status.new_data = true;
	}

	//==-- Calibrations
	if( _system_status.state == MAV_STATE_CALIBRATING ) {	//If any calibration is in progress
		//Run the rest of the calibration logic
		sensors_calibrate();
	}

	//TODO: This should be aware of failures
	return true;
}

