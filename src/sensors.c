#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "calibration.h"
#include "drivers/drv_sensors.h"
#include "drivers/drv_system.h"
#include "drivers/drv_ppm.h"
#include "estimator.h"
#include "mavlink_system.h"
#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"

#include "fix16.h"
#include "fixextra.h"
#include "fixquat.h"
#include "fixvector3d.h"


//==-- Global Variables
int8_t _actuator_apply_g1_map[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];

sensor_readings_t _sensors;
calibration_data_t _calibrations;

system_status_t _system_status;
command_input_t _cmd_rc_input;


//==-- Local Variables
static uint16_t rc_cal_[DRV_PPM_MAX_INPUTS][3];
static fix16_t rc_rev_[DRV_PPM_MAX_INPUTS];
static fix16_t rc_dz_[DRV_PPM_MAX_INPUTS];
static bool attempted_rc_default_mode_change_;

//==-- Functions
void sensor_status_init( sensor_status_t* status, bool sensor_present ) {
	status->present = sensor_present;
	status->new_data = false;
	status->time_read = 0;
}

static void sensors_init_hil( void ) {
	sensor_status_init( &_sensors.hil.status,
						get_param_uint( PARAM_SENSOR_HIL_CBRK ) );

	if ( _sensors.hil.status.present )
		mavlink_queue_broadcast_error( "[SENSOR] HARDWARE IN THE LOOP ENABLED!" );

	_sensors.hil.accel.x = 0;
	_sensors.hil.accel.y = 0;
	_sensors.hil.accel.z = 0;
	_sensors.hil.gyro.x = 0;
	_sensors.hil.gyro.y = 0;
	_sensors.hil.gyro.z = 0;
	_sensors.hil.mag.x = 0;
	_sensors.hil.mag.y = 0;
	_sensors.hil.mag.z = 0;

	_sensors.hil.pressure_abs = 0;
	_sensors.hil.pressure_diff = 0;
	_sensors.hil.pressure_alt = 0;

	_sensors.hil.temperature = 0;
}

void sensors_init( void ) {
	_sensors.fresh_sensor_data = false;

	//==-- Hardware In The Loop
	sensors_init_hil();

	if ( !_sensors.hil.status.present ) {
		if ( !drv_sensors_i2c_init() ) {
			sensor_status_init( &_sensors.imu.status, false );
			sensor_status_init( &_sensors.mag.status, false );
			sensor_status_init( &_sensors.baro.status, false );

			mavlink_queue_broadcast_error(
				"[SENSOR] Unable to configure i2c, disabling board sensors!" );
		}
	} else {
		sensor_status_init( &_sensors.imu.status, false );
		sensor_status_init( &_sensors.mag.status, false );
		sensor_status_init( &_sensors.baro.status, false );

		mavlink_queue_broadcast_notice( "[SENSOR] Using HIL, some sensors disabled" );
	}

	_sensors.mag.period_update = 1000 * fix16_to_int( fix16_div(
											_fc_1000, get_param_fix16( PARAM_SENSOR_MAG_UPDATE_RATE ) ) );
	_sensors.mag.raw.x = 0;
	_sensors.mag.raw.y = 0;
	_sensors.mag.raw.z = 0;
	_sensors.mag.mag.x = 0;
	_sensors.mag.mag.y = 0;
	_sensors.mag.mag.z = 0;

	_sensors.baro.period_update = 1000 * fix16_to_int( fix16_div(
											 _fc_1000, get_param_fix16( PARAM_SENSOR_BARO_UPDATE_RATE ) ) );
	_sensors.baro.raw_press = 0;
	_sensors.baro.raw_temp = 0;

	//==-- Sonar
	if ( get_param_fix16( PARAM_SENSOR_SONAR_UPDATE_RATE ) > 0 ) {
		mavlink_queue_broadcast_error(
			"[SENSOR] Sonar feature is not developed yet!" );

		// XXX:sensor_status_init( &_sensors.sonar.status, sonarInit() );
		sensor_status_init( &_sensors.sonar.status, false );

		// If we expected it to be present, but it failed
		if ( !_sensors.sonar.status.present )
			mavlink_queue_broadcast_error(
				"[SENSOR] Unable to configure sonar, disabling!" );
	} else {
		sensor_status_init( &_sensors.sonar.status, false );
	}

	//==-- External Pose
	sensor_status_init( &_sensors.ext_pose.status, false );

	//==-- RC Input
	sensor_status_init( &_sensors.rc_input.status, drv_sensors_rc_input_init() );

	for ( int i = 0; i < 8; i++ ) {
		_sensors.rc_input.raw[i] = 0;
	}

	_sensors.rc_input.mapping_set = false;
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
	_sensors.rc_input.r_m = MAIN_MODE_UNSET;
	sensors_update_rc_cal();

	attempted_rc_default_mode_change_ = false;

	//RC RSSI
	sensor_status_init( &_sensors.rc_rssi.status, false );	//Init by RC input setup (assmue not present by default)
	_sensors.rc_rssi.raw = 0;
	_sensors.rc_rssi.normalized = 0;

	// RC Safety toggle
	sensor_status_init( &_sensors.rc_arm_toggle.status,
						get_param_uint( PARAM_SENSOR_RC_SAFETY_CBRK ) );
	_sensors.rc_arm_toggle.arm_req_made = false;
	_sensors.rc_arm_toggle.timer_start_us = 0;

	//==-- Safety button
	sensor_status_init( &_sensors.safety_button.status,
						get_param_uint( PARAM_SENSOR_SAFETY_CBRK ) );
	if ( _sensors.safety_button.status.present ) {
		if ( !drv_sensors_safety_button_init() ) {
			mavlink_queue_broadcast_error(
				"[SENSOR] Could not initialize safety button!" );
		}
	}

	_sensors.safety_button.state = false;
	_sensors.safety_button.period_us = 100000; // 100ms update rate
	_sensors.safety_button.time_db_read = 0;
	_sensors.safety_button.period_db_us = 50000; // 50ms debounce period
	_sensors.safety_button.state_db = false;

	//==-- Voltage Monitor
	sensor_status_init( &_sensors.voltage_monitor.status,
						drv_sensors_battery_monitor_init() );

	_sensors.voltage_monitor.state_raw = 0;
	_sensors.voltage_monitor.state_calc = 0;
	_sensors.voltage_monitor.state_filtered = 0;
}

/*
//==-- Low Pass Filter for time offsets
int64_t sensors_clock_smooth_time_skew( int64_t tc, int64_t tn ) {
	// The closer alpha is to 1.0, the faster the moving
	// average updates in response to new offset samples.

	// Do this in floating point as fix16_t does not have an easy interface for
	// uint64_t
	float alpha = fix16_to_float( get_param_fix16( PARAM_TIMESYNC_ALPHA ) );
	return ( int64_t )( alpha * tn ) + ( int64_t )( ( 1.0f - alpha ) * tc );
}

float sensors_clock_smooth_time_drift( float tc, float tn ) {
	// The closer alpha is to 1.0, the faster the moving
	// average updates in response to new offset samples.

	// Do this in floating point as fix16_t does not have an easy interface for
	// uint64_t
	float alpha = fix16_to_float( get_param_fix16( PARAM_TIMESYNC_ALPHA ) );
	return ( int64_t )( alpha * tn ) + ( int64_t )( ( 1.0f - alpha ) * tc );
}

uint64_t sensors_clock_rt_get( void ) {
	return ( uint64_t )( system_micros() * 1000LL ) + _sensors.clock.rt_offset_ns;
}

uint32_t sensors_clock_imu_int_get( void ) {
	return _sensors.clock.imu_time_ready;
}
*/

static fix16_t dual_normalized_input( uint16_t pwm, uint16_t min, uint16_t mid, uint16_t max ) {
	fix16_t pwmn = 0;

	if ( pwm > mid ) {
		pwmn = fix16_div( fix16_from_int( pwm - mid ), fix16_from_int( max - mid ) );
	} else if ( pwm < mid ) {
		pwmn = -fix16_div( fix16_from_int( mid - pwm ), fix16_from_int( mid - min ) );
	} else {
		// Stick is perfectly centered
		pwmn = 0;
	}

	// Sanity check our dual normalize incase we have bad min/max params
	// Dual normalized (-1 -> 1)
	return fix16_constrain(pwmn, -_fc_1, _fc_1);
}

void sensors_update_rc_cal( void ) {
	rc_cal_[0][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC1_MIN );
	rc_cal_[0][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC1_MID );
	rc_cal_[0][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC1_MAX );
	rc_cal_[1][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC2_MIN );
	rc_cal_[1][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC2_MID );
	rc_cal_[1][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC2_MAX );
	rc_cal_[2][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC3_MIN );
	rc_cal_[2][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC3_MID );
	rc_cal_[2][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC3_MAX );
	rc_cal_[3][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC4_MIN );
	rc_cal_[3][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC4_MID );
	rc_cal_[3][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC4_MAX );
	rc_cal_[4][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC5_MIN );
	rc_cal_[4][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC5_MID );
	rc_cal_[4][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC5_MAX );
	rc_cal_[5][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC6_MIN );
	rc_cal_[5][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC6_MID );
	rc_cal_[5][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC6_MAX );
	rc_cal_[6][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC7_MIN );
	rc_cal_[6][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC7_MID );
	rc_cal_[6][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC7_MAX );
	rc_cal_[7][SENSOR_RC_CAL_MIN] = get_param_uint( PARAM_RC8_MIN );
	rc_cal_[7][SENSOR_RC_CAL_MID] = get_param_uint( PARAM_RC8_MID );
	rc_cal_[7][SENSOR_RC_CAL_MAX] = get_param_uint( PARAM_RC8_MAX );

	rc_rev_[0] = get_param_fix16( PARAM_RC1_REV );
	rc_rev_[1] = get_param_fix16( PARAM_RC2_REV );
	rc_rev_[2] = get_param_fix16( PARAM_RC3_REV );
	rc_rev_[3] = get_param_fix16( PARAM_RC4_REV );
	rc_rev_[4] = get_param_fix16( PARAM_RC5_REV );
	rc_rev_[5] = get_param_fix16( PARAM_RC6_REV );
	rc_rev_[6] = get_param_fix16( PARAM_RC7_REV );
	rc_rev_[7] = get_param_fix16( PARAM_RC8_REV );

	rc_dz_[0] = get_param_fix16( PARAM_RC1_DZ );
	rc_dz_[1] = get_param_fix16( PARAM_RC2_DZ );
	rc_dz_[2] = get_param_fix16( PARAM_RC3_DZ );
	rc_dz_[3] = get_param_fix16( PARAM_RC4_DZ );
	rc_dz_[4] = get_param_fix16( PARAM_RC5_DZ );
	rc_dz_[5] = get_param_fix16( PARAM_RC6_DZ );
	rc_dz_[6] = get_param_fix16( PARAM_RC7_DZ );
	rc_dz_[7] = get_param_fix16( PARAM_RC8_DZ );
}

static bool sensors_update( uint32_t time_us ) {
	// bool update_success = false;
	// TODO: Remember not to expect all sensors to be ready

	//==-- Update IMU
	// XXX: Nothing to do, but could do a present check for posix if we go there

	//==-- Update Magnetometer
	if ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) {
		// Anything?
	}

	//==-- Update Barometer
	if ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
		/*
if(_sensors.baro.status.new_data) {
_sensors.baro.status.new_data = false;

mavlink_message_t baro_msg_out;
mavlink_prepare_scaled_pressure(&baro_msg_out);
lpq_queue_broadcast_msg(&baro_msg_out);
}
*/
	}

	//==-- Update Sonar
	if ( _system_status.sensors.sonar.health == SYSTEM_HEALTH_OK ) {
		// Anything?
	}

	//==-- RC Input, RC Mapping, & Saftety Toggle
	if ( drv_sensors_rc_rssi_read( &_sensors.rc_rssi.raw ) ) {
		_sensors.rc_rssi.status.time_read = time_us;
		_sensors.rc_rssi.status.new_data = true;
		safety_update_sensor( &_system_status.sensors.rc_rssi );

		//Normalize to typical values (cant calibrate RSSI)
		fix16_t rssi_rn = dual_normalized_input( _sensors.rc_rssi.raw, 1000, 1500, 2000 );
		_sensors.rc_rssi.normalized = fix16_normalize_clamp( rssi_rn, -_fc_1, _fc_1);
	}

	if ( drv_sensors_rc_input_read( _sensors.rc_input.raw ) ) {
		_sensors.rc_input.status.time_read = time_us;
		_sensors.rc_input.status.new_data = true;
		safety_update_sensor( &_system_status.sensors.rc_input );

		//XXX:	This is limiting in the sense that you need a 4Ch. controller
		//		for the RC input to be accepted. Probably won't be much of a
		//		problem with modern equipment though.
		//		Only perform this check when we actually recieve new data.
		_sensors.rc_input.mapping_set = get_param_uint( PARAM_RC_MAP_ROLL ) &&
										get_param_uint( PARAM_RC_MAP_PITCH ) &&
										get_param_uint( PARAM_RC_MAP_YAW ) &&
										get_param_uint( PARAM_RC_MAP_THROTTLE );
	}

	// Check that the following is OK before doing any other RC checks:
	//  RC input is healthy
	//	There is actually new data
	//  Not currently calibrating
	if( ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK ) &&
		( _sensors.rc_input.status.new_data ) &&
		( _system_status.state != MAV_STATE_CALIBRATING ) ) {

		// Handle the normalization steps
		fix16_t rc_ncd[DRV_PPM_MAX_INPUTS];

		for(int i=0; i<DRV_PPM_MAX_INPUTS; i++) {
			// Raw normalized
			fix16_t rn = dual_normalized_input( _sensors.rc_input.raw[i],
												rc_cal_[i][SENSOR_RC_CAL_MIN],
												rc_cal_[i][SENSOR_RC_CAL_MID],
												rc_cal_[i][SENSOR_RC_CAL_MAX] );

			// Normalized, corrected, and deadzoned
			rc_ncd[i] = fix16_mul( rc_rev_[i],
								   ( ( fix16_abs( rn ) < rc_dz_[i] ) ? 0 : rn ) );
		}

		// Group mixers
		for ( int i = 0; i < MIXER_NUM_MOTORS; i++ ) {
			_actuator_control_g2[i] = rc_ncd[i];
			_actuator_control_g3[i] = rc_ncd[i];
		}

		// RC input control mapping, and RC arming
		if ( _sensors.rc_input.mapping_set ) {
			uint8_t chan_roll = get_param_uint( PARAM_RC_MAP_ROLL ) - 1;
			uint8_t chan_pitch = get_param_uint( PARAM_RC_MAP_PITCH ) - 1;
			uint8_t chan_yaw = get_param_uint( PARAM_RC_MAP_YAW ) - 1;
			uint8_t chan_throttle = get_param_uint( PARAM_RC_MAP_THROTTLE ) - 1;

			//Store the reading for debug, should be already sanitized by this point
			_sensors.rc_input.p_r = _sensors.rc_input.raw[chan_roll];
			_sensors.rc_input.p_p = _sensors.rc_input.raw[chan_pitch];
			_sensors.rc_input.p_y = _sensors.rc_input.raw[chan_yaw];
			_sensors.rc_input.p_T = _sensors.rc_input.raw[chan_throttle];

			// Calculate deadzones and save outputs
			_sensors.rc_input.c_r = rc_ncd[chan_roll];
			_sensors.rc_input.c_p = rc_ncd[chan_pitch];
			_sensors.rc_input.c_y = rc_ncd[chan_yaw];
			// See if we should allow full range thrust
			if( get_param_uint(PARAM_RC_FULL_RANGE_THRUST) ) {
				//If so, simply use the dual range calculation
				_sensors.rc_input.c_T = rc_ncd[chan_throttle];
			} else {
				//Otherwise, we have to re-normalize to 0->1
				_sensors.rc_input.c_T = fix16_normalize_clamp( rc_ncd[chan_throttle], -_fc_1, _fc_1);
			}

			// Handle the logic for arming toggle
			_sensors.rc_arm_toggle.status.time_read = time_us;
			_sensors.rc_arm_toggle.status.new_data = true;

			if ( ( fix16_abs(_sensors.rc_input.c_T) < _fc_0_05 ) && ( fix16_abs( _sensors.rc_input.c_y ) > _fc_0_95 ) ) {

				if ( _sensors.rc_arm_toggle.timer_start_us == 0 )
					_sensors.rc_arm_toggle.timer_start_us = time_us;

				if ( time_us > ( _sensors.rc_arm_toggle.timer_start_us + get_param_uint( PARAM_RC_ARM_TIMER ) ) ) {
					if ( !_sensors.rc_arm_toggle.arm_req_made ) {
						if ( _sensors.rc_input.c_y < 0 ) {
							safety_request_disarm();
						} else {
							safety_request_arm();
						}

						_sensors.rc_arm_toggle.arm_req_made = true;
					}
				}
			} else {
				_sensors.rc_arm_toggle.arm_req_made = false;
				_sensors.rc_arm_toggle.timer_start_us = 0;
			}
		}

		// RC mode switch functions
		if ( get_param_uint( PARAM_RC_MAP_MODE_SW ) ) {
			uint32_t chan_mode = get_param_uint( PARAM_RC_MAP_MODE_SW ) - 1;
			_sensors.rc_input.p_m = _sensors.rc_input.raw[chan_mode];

			//Get the normalized value (0->1)
			_sensors.rc_input.c_m = fix16_normalize_clamp( rc_ncd[chan_mode], -_fc_1, _fc_1);;
			compat_px4_main_mode_t r_m_last = _sensors.rc_input.r_m;

			if ( ( get_param_fix16( PARAM_RC_MODE_STAB ) > 0 ) &&
				 ( fix16_abs( fix16_sub( get_param_fix16( PARAM_RC_MODE_STAB ), _sensors.rc_input.c_m ) ) < get_param_fix16( PARAM_RC_MODE_RANGE ) ) ) {
				_sensors.rc_input.r_m = MAIN_MODE_STABILIZED;
			} else if ( ( get_param_fix16( PARAM_RC_MODE_ACRO ) > 0 ) &&
						( fix16_abs( fix16_sub( get_param_fix16( PARAM_RC_MODE_ACRO ), _sensors.rc_input.c_m ) ) < get_param_fix16( PARAM_RC_MODE_RANGE ) ) ) {
				_sensors.rc_input.r_m = MAIN_MODE_ACRO;
			} else if ( ( get_param_fix16( PARAM_RC_MODE_OFFBOARD ) > 0 ) &&
						( fix16_abs( fix16_sub( get_param_fix16( PARAM_RC_MODE_OFFBOARD ), _sensors.rc_input.c_m ) ) < get_param_fix16( PARAM_RC_MODE_RANGE ) ) ) {
				_sensors.rc_input.r_m = MAIN_MODE_OFFBOARD;
			} // Else no mode select match, don't change mode

			// If a new mode is requested
			if ( _sensors.rc_input.r_m != r_m_last ) {
				if ( !safety_request_control_mode( _sensors.rc_input.r_m ) ) {
					mavlink_queue_broadcast_error( "[SENSOR] RC mode switch rejected" );
				}
			}
		} else if ( !attempted_rc_default_mode_change_ &&
					( _system_status.control_mode == MAIN_MODE_UNSET ) &&
					( get_param_uint( PARAM_RC_MODE_DEFAULT ) != MAIN_MODE_UNSET ) ) {
			// RC connection should trigger a default mode change
			// We only allow this to occur once after boot
			// in case RC drops and reconnects mid-flight
			// This also only occurs if the mode is unset,
			// and the default is valid, for the same reason

			if ( !safety_request_control_mode(
					 get_param_uint( PARAM_RC_MODE_DEFAULT ) ) ) {
				mavlink_queue_broadcast_error(
					"[SENSOR] Error setting RC default mode" );
			}

			attempted_rc_default_mode_change_ = true;
		}
	} else if ( ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_TIMEOUT ) &&
				( _sensors.rc_input.raw[0] > 0 ) ) {
		// Else if we have a timeout and there is some stray data,
		// clear out the data (so mavlink, etc., also shows timeout)
		for(int i=0; i<DRV_PPM_MAX_INPUTS; i++)
			_sensors.rc_input.raw[i] = 0;
	}

	//==-- Safety Button
	bool safety_button_reading = false;

	safety_button_reading = drv_sensors_safety_button_read();

	if ( safety_button_reading != _sensors.safety_button.state_db )
		_sensors.safety_button.time_db_read = time_us;

	if ( ( time_us - _sensors.safety_button.time_db_read ) > _sensors.safety_button.period_db_us ) {
		if ( safety_button_reading != _sensors.safety_button.state ) { // The reading has changed
			_sensors.safety_button.state = safety_button_reading;

			_sensors.safety_button.status.time_read = time_us;
			_sensors.safety_button.status.new_data = true;
		}
	}

	_sensors.safety_button.state_db = safety_button_reading;

	//==-- Voltage Monitor
	// Continue as long as the number of cells has been set,
	// and we are ready for a new sample
	if( ( get_param_uint( PARAM_BATTERY_CELL_NUM ) > 0 )  &&
		( (time_us - _sensors.voltage_monitor.status.time_read) > get_param_uint(PARAM_BATTERY_PERIOD) ) ) {
		//_sensors.voltage_monitor.state_raw =
		// digitalIn(_sensors.voltage_monitor.gpio_p, _sensors.voltage_monitor.pin);
		_sensors.voltage_monitor.state_raw = drv_sensors_battery_monitor_read();

		fix16_t voltage_res = fix16_div( fix16_from_int( 0xFFF ),
										 _fc_3_3 ); // XXX: 0xFFF is from 12Bit adc
		// XXX: TODO: Should lookup board rev properly
		fix16_t voltage_div = ( get_param_fix16( PARAM_BATTERY_DIVIDER ) == -_fc_1 ) ? SENSOR_VMON_DIVIDER_NAZE32 : get_param_fix16( PARAM_BATTERY_DIVIDER );

		_sensors.voltage_monitor.state_calc = fix16_mul( fix16_div( fix16_from_int( _sensors.voltage_monitor.state_raw ),
																	voltage_res ),
														 voltage_div );
		// Filter reading: value_lpf = ((1 - alpha) * value) + (alpha * value_lpf);
		fix16_t voltage_alpha = get_param_fix16( PARAM_BATTERY_READING_FILTER );
		_sensors.voltage_monitor.state_filtered = fix16_sadd(
			fix16_smul( fix16_ssub( _fc_1, voltage_alpha ),
						_sensors.voltage_monitor.state_calc ),
			fix16_smul( voltage_alpha, _sensors.voltage_monitor.state_filtered ) );
		// Calculate percentage left
		fix16_t voltage_min = fix16_mul( fix16_from_int( get_param_uint( PARAM_BATTERY_CELL_NUM ) ),
										 get_param_fix16( PARAM_BATTERY_CELL_MIN ) );
		fix16_t voltage_max = fix16_mul( fix16_from_int( get_param_uint( PARAM_BATTERY_CELL_NUM ) ),
										 get_param_fix16( PARAM_BATTERY_CELL_MAX ) );
		fix16_t voltage_range = fix16_sub( voltage_max, voltage_min );
		// Only calc percentage if in range
		if ( ( _sensors.voltage_monitor.state_filtered > voltage_min ) && ( _sensors.voltage_monitor.state_filtered < voltage_max ) ) {
			_sensors.voltage_monitor.precentage = fix16_div(
				fix16_sub( _sensors.voltage_monitor.state_filtered, voltage_min ),
				voltage_range );
		} else {
			_sensors.voltage_monitor.precentage = 0;
		}

		_sensors.voltage_monitor.status.time_read = time_us;
		_sensors.voltage_monitor.status.new_data = true;
	}

	// TODO: This should be aware of failures
	return true;
}

void sensors_read( uint32_t time_us ) {
	// Return the results
	//TODO: Does this really need to be structured like this?
	//		Consider relying on just the internal sensor data
	//		ready vars elsewhere in the system
	if ( drv_sensors_i2c_read( time_us ) ) {
		_sensors.fresh_sensor_data = sensors_update( time_us );
	}
}

#ifdef __cplusplus
}
#endif
