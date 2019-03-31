#include "drivers/drv_sensors.h"
#include "drivers/drv_system.h"

#include <libopencm3/stm32/i2c.h>
#include "drivers/opencm3_naze32_common/drv_i2c.h"

#include "drivers/opencm3_naze32_common/drv_bmp280.h"
#include "drivers/opencm3_naze32_common/drv_hmc5883l.h"
#include "drivers/opencm3_naze32_common/drv_mpu.h"

#include "calibration.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "mavlink_system.h"

#include "fix16.h"
#include "fixextra.h"


#define NAZE32_I2C_SENSOR_CHANNEL I2C2


sensor_readings_t _sensors;
calibration_data_t _calibrations;

static volatile uint32_t imu_time_ready_;
static volatile uint32_t imu_time_last_;

static volatile uint8_t accel_status = 0;
static volatile uint8_t gyro_status = 0;
static volatile uint8_t temp_status = 0;
static volatile uint8_t mag_status = 0;
static volatile uint8_t baro_status = 0;
static volatile uint8_t sonar_status = 0;

static volatile int16_t read_accel_raw[3];
static volatile int16_t read_gyro_raw[3];
static volatile int16_t read_temp_raw;

static volatile int16_t read_mag_raw[3];
static volatile int32_t read_baro_raw[2];

static void drv_sensors_imu_ready(void) {
	//==-- Timing setup get loop time
	imu_time_ready_ = system_micros();
}

static void drv_sensors_i2c_scan(void) {
	for(uint8_t addr=0; addr<0x80; ++addr) {
		system_pause_ms(50);

		if (drv_i2c_write_register(NAZE32_I2C_SENSOR_CHANNEL, addr, 0x00, 0x00)) {
			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
			snprintf(text,
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN,
					 "[PARAM] i2c_scan() found: 0x%x",
					 addr);
			mavlink_queue_broadcast_notice(text);
		}
	}
}

bool drv_sensors_i2c_init( void ) {
	imu_time_ready_ = 0;
	imu_time_last_ = 0;

	system_pause_ms( 100 ); // Wait for i2c devices to boot properly

	drv_i2c_init( NAZE32_I2C_SENSOR_CHANNEL );

	accel_status = I2C_JOB_DEFAULT;
	gyro_status = I2C_JOB_DEFAULT;
	temp_status = I2C_JOB_DEFAULT;
	mag_status = I2C_JOB_DEFAULT;
	baro_status = I2C_JOB_DEFAULT;
	sonar_status = I2C_JOB_DEFAULT;

	//==-- Initialize i2c sensors

	// IMU
	sensor_status_init( &_sensors.imu.status,
						drv_sensors_imu_init( NAZE32_I2C_SENSOR_CHANNEL,
											  &_sensors.imu.accel_scale,
											  &_sensors.imu.gyro_scale ) );


	if(_sensors.imu.status.present) {
		//Set up to be interrupt-driven data read
		drv_sensors_imu_configure_int(&drv_sensors_imu_ready);
	} else {
		// Something is wrong with the board / addresses
		// Do a scan to help out the user

		mavlink_queue_broadcast_error( "[SENSOR] Could not connect to IMU!" );

		drv_sensors_i2c_scan();
	}

	// Mag
	if ( get_param_fix16( PARAM_SENSOR_MAG_UPDATE_RATE ) > 0 ) {
		mavlink_queue_broadcast_error( "[SENSOR] Mag support is not stable!" );

		sensor_status_init( &_sensors.mag.status, drv_sensors_mag_init(NAZE32_I2C_SENSOR_CHANNEL) );

		// If we expected it to be present, but it failed
		if ( !_sensors.mag.status.present )
			mavlink_queue_broadcast_error(
				"[SENSOR] Unable to configure mag, disabling!" );
	} else {
		sensor_status_init( &_sensors.mag.status, false );
	}

	// Baro
	if ( get_param_fix16( PARAM_SENSOR_BARO_UPDATE_RATE ) > 0 ) {
		sensor_status_init( &_sensors.baro.status, drv_sensors_baro_init(NAZE32_I2C_SENSOR_CHANNEL) );

		// If we expected it to be present, but it failed
		if ( !_sensors.baro.status.present )
			mavlink_queue_broadcast_error(
				"[SENSOR] Unable to configure baro, disabling!" );
	} else {
		sensor_status_init( &_sensors.baro.status, false );
	}

	return true;
}

bool drv_sensors_i2c_job_queued( void ) {
	bool a_done = ( accel_status == I2C_JOB_DEFAULT ) || ( accel_status == I2C_JOB_COMPLETE );
	bool g_done = ( gyro_status == I2C_JOB_DEFAULT ) || ( gyro_status == I2C_JOB_COMPLETE );
	bool t_done = ( temp_status == I2C_JOB_DEFAULT ) || ( temp_status == I2C_JOB_COMPLETE );
	bool m_done = ( mag_status == I2C_JOB_DEFAULT ) || ( mag_status == I2C_JOB_COMPLETE );
	bool b_done = ( baro_status == I2C_JOB_DEFAULT ) || ( baro_status == I2C_JOB_COMPLETE );
	bool s_done = ( sonar_status == I2C_JOB_DEFAULT ) || ( sonar_status == I2C_JOB_COMPLETE );

	return !a_done || !g_done || !t_done || !m_done || !b_done || !s_done;
}

void drv_sensors_i2c_clear( void ) {
	while ( drv_sensors_i2c_job_queued() )
		__asm__("nop");
}

static void drv_sensors_i2c_poll( uint32_t time_us ) {
	//==-- Update IMU
	if ( _sensors.imu.status.present ) {
		// Update the imu sensor if we've recieved a new interrupt
		// and we're not collecting data already
		if( ( imu_time_ready_ > imu_time_last_ ) &&
			( accel_status == I2C_JOB_DEFAULT ) &&
			( gyro_status == I2C_JOB_DEFAULT ) &&
			( temp_status == I2C_JOB_DEFAULT ) ) {

			mpu_request_async_accel_read( NAZE32_I2C_SENSOR_CHANNEL, read_accel_raw, &accel_status );
			mpu_request_async_gyro_read( NAZE32_I2C_SENSOR_CHANNEL, read_gyro_raw, &gyro_status );
			mpu_request_async_temp_read( NAZE32_I2C_SENSOR_CHANNEL, &read_temp_raw, &temp_status );

			imu_time_last_ = imu_time_ready_;
		}
	}

	// Only allow a single additional i2c device to be queued each sensor cycle
	bool aux_sensor_req = ( mag_status != I2C_JOB_DEFAULT ) && ( baro_status != I2C_JOB_DEFAULT ) && ( sonar_status != I2C_JOB_DEFAULT );

	//==-- Update Mag
	if ( _sensors.mag.status.present && !aux_sensor_req ) {
		// Update the sensor if it's time (and it's not currently reading)
		if ( ( ( time_us - _sensors.mag.status.time_read ) > _sensors.mag.period_update ) && ( mag_status == I2C_JOB_DEFAULT ) ) {
			hmc5883l_request_async_read( NAZE32_I2C_SENSOR_CHANNEL, read_mag_raw, &mag_status );
			aux_sensor_req = true;
		}
	}

	//==-- Update Baro
	if ( _sensors.baro.status.present && !aux_sensor_req ) {
		// Update the sensor if it's time (and it's not currently reading)
		if ( ( ( time_us - _sensors.baro.status.time_read ) > _sensors.baro.period_update ) && ( baro_status == I2C_JOB_DEFAULT ) ) {
			bmp280_request_async_read( NAZE32_I2C_SENSOR_CHANNEL, read_baro_raw, &baro_status );
			aux_sensor_req = true;
		}
	}

	//==-- Update Sonar
	if ( _sensors.sonar.status.present && !aux_sensor_req ) {
		// Update the sensor if it's time (and it's not currently reading)
		if ( ( ( time_us - _sensors.sonar.status.time_read ) > _sensors.sonar.period_update ) && ( sonar_status == I2C_JOB_DEFAULT ) ) {
			// sonar_request_async_read(read_baro_raw, &baro_status);
			aux_sensor_req = true;
		}
	}
}

bool drv_sensors_i2c_read( uint32_t time_us ) {
	drv_sensors_i2c_poll( time_us );

	bool imu_job_complete = false;
	bool mag_job_complete = false;
	bool baro_job_complete = false;
	bool sonar_job_complete = false;


	//TODO: Handle errors!
	if( accel_status == I2C_JOB_ERROR )
		accel_status = I2C_JOB_DEFAULT;
	if( gyro_status == I2C_JOB_ERROR )
		gyro_status = I2C_JOB_DEFAULT;
	if( temp_status == I2C_JOB_ERROR )
		temp_status = I2C_JOB_DEFAULT;

	// Check IMU status
	if ( ( accel_status == I2C_JOB_COMPLETE ) && ( gyro_status == I2C_JOB_COMPLETE ) && ( temp_status == I2C_JOB_COMPLETE ) ) {
		imu_job_complete = true;
		accel_status = I2C_JOB_DEFAULT;
		gyro_status = I2C_JOB_DEFAULT;
		temp_status = I2C_JOB_DEFAULT;

		//==-- Save raw data
		// XXX: Some values need to be inversed to be in the NED frame
		_sensors.imu.accel_raw.x = read_accel_raw[0];
		_sensors.imu.accel_raw.y = -read_accel_raw[1];
		_sensors.imu.accel_raw.z = -read_accel_raw[2];

		_sensors.imu.gyro_raw.x = read_gyro_raw[0];
		_sensors.imu.gyro_raw.y = -read_gyro_raw[1];
		_sensors.imu.gyro_raw.z = -read_gyro_raw[2];

		_sensors.imu.temp_raw = read_temp_raw;

		//==-- Handle raw data
		// Convert temperature SI units (degC, m/s^2, rad/s)
		// value = (_sensors.imu.temp_raw/temp_scale) + temp_shift
		_sensors.imu.temperature = fix16_add( fix16_div( fix16_from_int( _sensors.imu.temp_raw ),
														 _calibrations.data.accel.temp_scale ),
											  _calibrations.data.accel.temp_shift );

		// Accel
		// TODO: value = (raw - BIAS - (EMP_COMP * TEMP)) * scale
		// value = (raw - BIAS) * scale

		// Correct for measurement biases
		fix16_t accel_x_tmp = fix16_mul( fix16_from_int( _sensors.imu.accel_raw.x - get_param_int( PARAM_ACC_X_BIAS ) ),
										 _sensors.imu.accel_scale );
		fix16_t accel_y_tmp = fix16_mul( fix16_from_int( _sensors.imu.accel_raw.y - get_param_int( PARAM_ACC_Y_BIAS ) ),
										 _sensors.imu.accel_scale );
		fix16_t accel_z_tmp = fix16_mul( fix16_from_int( _sensors.imu.accel_raw.z - get_param_int( PARAM_ACC_Z_BIAS ) ),
										 _sensors.imu.accel_scale );

		// Scale the accelerometer to match 1G
		_sensors.imu.accel.x = fix16_mul( accel_x_tmp, ( accel_x_tmp > 0 ) ? get_param_fix16( PARAM_ACC_X_SCALE_POS ) : get_param_fix16( PARAM_ACC_X_SCALE_NEG ) );
		_sensors.imu.accel.y = fix16_mul( accel_y_tmp, ( accel_y_tmp > 0 ) ? get_param_fix16( PARAM_ACC_Y_SCALE_POS ) : get_param_fix16( PARAM_ACC_Y_SCALE_NEG ) );
		_sensors.imu.accel.z = fix16_mul( accel_z_tmp, ( accel_z_tmp > 0 ) ? get_param_fix16( PARAM_ACC_Z_SCALE_POS ) : get_param_fix16( PARAM_ACC_Z_SCALE_NEG ) );

		// Gyro
		// value = (raw - BIAS) * scale
		_sensors.imu.gyro.x = fix16_mul( fix16_from_int( _sensors.imu.gyro_raw.x - get_param_int( PARAM_GYRO_X_BIAS ) ),
										 _sensors.imu.gyro_scale );
		_sensors.imu.gyro.y = fix16_mul( fix16_from_int( _sensors.imu.gyro_raw.y - get_param_int( PARAM_GYRO_Y_BIAS ) ),
										 _sensors.imu.gyro_scale );
		_sensors.imu.gyro.z = fix16_mul( fix16_from_int( _sensors.imu.gyro_raw.z - get_param_int( PARAM_GYRO_Z_BIAS ) ),
										 _sensors.imu.gyro_scale );

		// Other IMU updates
		_sensors.imu.status.time_read = imu_time_ready_;
		_sensors.imu.status.new_data = true;
		safety_update_sensor( &_system_status.sensors.imu );
	}


	if ( mag_status == I2C_JOB_DEFAULT ) {
		mag_job_complete = true;
	} else if ( mag_status == I2C_JOB_ERROR ) {
		mag_job_complete = true;
		mag_status = I2C_JOB_DEFAULT;
	} else if ( mag_status == I2C_JOB_COMPLETE ) {
		mag_job_complete = true;
		mag_status = I2C_JOB_DEFAULT;

		// Handle raw values
		// XXX: Some values need to be switched to be in the NED frame
		_sensors.mag.raw.x = -read_mag_raw[0];
		_sensors.mag.raw.y = read_mag_raw[1];
		_sensors.mag.raw.z = read_mag_raw[2];

		_sensors.mag.mag.x = fix16_div( fix16_from_int( _sensors.mag.raw.x ),
										fix16_from_int( HMC5883L_GAIN_FACTOR ) );
		_sensors.mag.mag.y = fix16_div( fix16_from_int( _sensors.mag.raw.y ),
										fix16_from_int( HMC5883L_GAIN_FACTOR ) );
		_sensors.mag.mag.z = fix16_div( fix16_from_int( _sensors.mag.raw.z ),
										fix16_from_int( HMC5883L_GAIN_FACTOR ) );

		// TODO: Do remaining mag scaling / calibration steps

		/*
		// Get a north estimate
		// Build a rotation matrix
		v3d mag_body_x;
		v3d_normalize( &mag_body_x, &_sensors.mag.scaled );
		v3d mag_body_y;
		v3d mag_body_z = _sensors.imu.accel;

		v3d_cross( &mag_body_y, &mag_body_z, &mag_body_x );
		v3d_normalize( &mag_body_y, &mag_body_y );

		mf16 mag_body;
		dcm_from_basis( &mag_body, &mag_body_x, &mag_body_y, &mag_body_z );

		qf16 mag_q_body;
		matrix_to_qf16( &mag_q_body, &mag_body );
		qf16_normalize_to_unit( &mag_q_body, &mag_q_body );

		// De-rotate from the body back to the world
		qf16 q_tmp;
		qf16_inverse( &q_tmp, &mag_q_body );
		qf16_normalize_to_unit( &_sensors.mag.q, &q_tmp );
		*/

		// Other Mag updates
		_sensors.mag.status.time_read = system_micros();
		_sensors.mag.status.new_data = true;
		safety_update_sensor( &_system_status.sensors.mag );
	}


	if ( baro_status == I2C_JOB_DEFAULT ) {
		baro_job_complete = true;
	} else if ( baro_status == I2C_JOB_ERROR ) {
		baro_job_complete = true;
		baro_status = I2C_JOB_DEFAULT;
	} else if ( baro_status == I2C_JOB_COMPLETE ) {
		baro_job_complete = true;
		baro_status = I2C_JOB_DEFAULT;

		// Handle raw values
		_sensors.baro.raw_press = read_baro_raw[0];
		_sensors.baro.raw_temp = read_baro_raw[1];

		// Other Baro updates
		_sensors.baro.status.time_read = system_micros();
		_sensors.baro.status.new_data = true;
		safety_update_sensor( &_system_status.sensors.baro );
	}

	if ( sonar_status == I2C_JOB_DEFAULT ) {
		sonar_job_complete = true;
	} else if ( sonar_status == I2C_JOB_ERROR ) {
		sonar_job_complete = true;
		sonar_status = I2C_JOB_DEFAULT;
	} else if ( sonar_status == I2C_JOB_COMPLETE ) {
		sonar_job_complete = true;
		sonar_status = I2C_JOB_DEFAULT;

		//Handle raw values
		//_sensors.baro.raw_press = read_baro_raw[0];
		//_sensors.baro.raw_temp = read_baro_raw[1];

		//Other Baro updates
		//_sensors.baro.status.time_read = micros();
		//_sensors.baro.status.new_data = true;
		//safety_update_sensor(&_system_status.sensors.baro);

	}

	// TODO: Check other status
	// TODO: May need to offset these so they don't all check at once(?)

	return imu_job_complete && mag_job_complete && baro_job_complete && sonar_job_complete;
}
