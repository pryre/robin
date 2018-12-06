#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sensors.h"
#include "calibration.h"
#include "estimator.h"
#include "safety.h"
#include "params.h"
#include "mixer.h"
#include "mavlink_system.h"
#include "drivers/drv_sensors.h"
#include "drivers/drv_system.h"

//#include "breezystm32.h"
//#include "pwm.h"
//#include "adc.h"
//#include "gpio.h"

#include "fix16.h"
#include "fixvector3d.h"
#include "fixquat.h"
#include "fixextra.h"

//==-- Local Variables
static uint16_t rc_cal[8][3];
static bool rc_rev[8];
static fix16_t rc_dz[8];

//==-- Global Variables
int8_t _actuator_apply_g1_map[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];

sensor_readings_t _sensors;
calibration_data_t _calibrations;

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

static void sensor_status_init(sensor_status_t *status, bool sensor_present) {
	status->present = sensor_present;
	status->new_data = false;
	status->time_read = 0;
}

static void sensors_init_hil(void) {
	sensor_status_init(&_sensors.hil.status, get_param_uint(PARAM_SENSOR_HIL_CBRK));

	if(_sensors.hil.status.present)
		mavlink_queue_broadcast_error("[SENSOR] HARDWARE IN THE LOOP ENABLED!");

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

void sensors_init(void) {
	//==-- Hardware In The Loop
	sensors_init_hil();

	if( !_sensors.hil.status.present ) {
		if( drv_sensors_i2c_init() ) {
			/*
			// XXX: I2C Sniffer
			uint8_t addr;
			for (addr=0; addr<128; ++addr) {
				delay(50);
				if (i2cWriteRegister(addr, 0x00, 0x00)) {

					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					snprintf(text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, "[PARAM] I2C sniff found: 0x%x", addr);
					mavlink_queue_broadcast_notice(text);
				}
			}
			*/

			//==-- IMU
			sensor_status_init(&_sensors.imu.status, drv_sensors_imu_init( &_sensors.imu.accel_scale, &_sensors.imu.gyro_scale ) );

			//==-- Mag
			if( get_param_fix16( PARAM_SENSOR_MAG_UPDATE_RATE ) > 0 ) {
				mavlink_queue_broadcast_error("[SENSOR] Mag support is not stable!");

				sensor_status_init( &_sensors.mag.status, drv_sensors_mag_init() );

				//If we expected it to be present, but it failed
				if(!_sensors.mag.status.present)
					mavlink_queue_broadcast_error("[SENSOR] Unable to configure mag, disabling!");
			} else {
				sensor_status_init( &_sensors.mag.status, false );
			}

			//==-- Baro
			if( get_param_fix16( PARAM_SENSOR_BARO_UPDATE_RATE ) > 0 ) {
				sensor_status_init( &_sensors.baro.status, drv_sensors_baro_init() );

				//If we expected it to be present, but it failed
				if(!_sensors.baro.status.present)
					mavlink_queue_broadcast_error("[SENSOR] Unable to configure baro, disabling!");
			} else {
				sensor_status_init( &_sensors.baro.status, false );
			}
		} else {

			sensor_status_init( &_sensors.imu.status, false );
			sensor_status_init( &_sensors.mag.status, false );
			sensor_status_init( &_sensors.baro.status, false );

			mavlink_queue_broadcast_error("[SENSOR] Unable to configure i2c, disabling board sensors!");
		}
	} else {
		sensor_status_init( &_sensors.imu.status, false );
		sensor_status_init( &_sensors.mag.status, false );
		sensor_status_init( &_sensors.baro.status, false );

		mavlink_queue_broadcast_notice("[SENSOR] Using HIL, some sensors disabled");
	}

	//==-- Timer
	clock_init();

	_sensors.mag.period_update = 1000*fix16_to_int( fix16_div(_fc_1000, get_param_fix16(PARAM_SENSOR_MAG_UPDATE_RATE)));
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

	_sensors.baro.period_update = 1000*fix16_to_int( fix16_div(_fc_1000, get_param_fix16(PARAM_SENSOR_BARO_UPDATE_RATE)));
	_sensors.baro.raw_press = 0;
	_sensors.baro.raw_temp = 0;

	//==-- Sonar
	if( get_param_fix16( PARAM_SENSOR_SONAR_UPDATE_RATE ) > 0 ) {
		mavlink_queue_broadcast_error("[SENSOR] Sonar feature is not developed yet!");

		//XXX:sensor_status_init( &_sensors.sonar.status, sonarInit() );
		sensor_status_init( &_sensors.sonar.status, false );

		//If we expected it to be present, but it failed
		if(!_sensors.sonar.status.present)
			mavlink_queue_broadcast_error("[SENSOR] Unable to configure sonar, disabling!");
	} else {
		sensor_status_init( &_sensors.sonar.status, false );
	}

	//==-- External Pose
	sensor_status_init( &_sensors.ext_pose.status, false );

	//==-- RC Input
	sensor_status_init( &_sensors.rc_input.status, drv_sensors_rc_input_init() );

	for(int i=0; i<8; i++) {
		_sensors.rc_input.raw[i] = 0;
	}

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
	sensor_status_init( &_sensors.rc_safety_toggle.status, get_param_uint(PARAM_SENSOR_RC_SAFETY_CBRK) );
	_sensors.rc_safety_toggle.arm_req_made = false;
	_sensors.rc_safety_toggle.timer_start_us = 0;

	//==-- Safety button
	sensor_status_init(&_sensors.safety_button.status, get_param_uint(PARAM_SENSOR_SAFETY_CBRK));
	if(_sensors.safety_button.status.present) {
		if( !drv_sensors_safety_button_init() ) {
			mavlink_queue_broadcast_error("[SENSOR] Could not initialize safety button!");
		}
	}

	_sensors.safety_button.state = false;
	_sensors.safety_button.period_us = 100000;		//100ms update rate
	_sensors.safety_button.time_db_read = 0;
	_sensors.safety_button.period_db_us = 50000;	//50ms debounce period
	_sensors.safety_button.state_db = false;

	//==-- Voltage Monitor
	sensor_status_init(&_sensors.voltage_monitor.status, drv_sensors_battery_monitor_init());

	_sensors.voltage_monitor.state_raw = 0;
	_sensors.voltage_monitor.state_calc = 0;
	_sensors.voltage_monitor.state_filtered = 0;
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
	return (uint64_t)( system_micros() * 1000LL ) + _sensors.clock.rt_offset_ns;
}

uint32_t sensors_clock_imu_int_get(void) {
	return _sensors.clock.imu_time_read;
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

static bool sensors_update(uint32_t time_us) {
	//bool update_success = false;
	//TODO: Remember not to expect all sensors to be ready

	//==-- Update IMU
	//XXX: Nothing to do, but could do a present check for posix if we go there

	//==-- Update Magnetometer
	if( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) {
		//Anything?
	}

	//==-- Update Barometer
	if( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
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
	if( _system_status.sensors.sonar.health == SYSTEM_HEALTH_OK ) {
		//Anything?
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

		drv_sensors_rc_input_read( _sensors.rc_input.raw );

		_sensors.rc_input.p_r = _sensors.rc_input.raw[chan_roll];
		_sensors.rc_input.p_p = _sensors.rc_input.raw[chan_pitch];
		_sensors.rc_input.p_y = _sensors.rc_input.raw[chan_yaw];
		_sensors.rc_input.p_T = _sensors.rc_input.raw[chan_throttle];

		if( (_sensors.rc_input.p_r > 800) && (_sensors.rc_input.p_r < 2200 ) &&
			(_sensors.rc_input.p_p > 800) && (_sensors.rc_input.p_p < 2200 ) &&
			(_sensors.rc_input.p_y > 800) && (_sensors.rc_input.p_y < 2200 ) &&
			(_sensors.rc_input.p_T > 800) && (_sensors.rc_input.p_T < 2200 ) ) {
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

			//Handle actuator control mapping
			for(int i=0; i<MIXER_NUM_MOTORS; i++) {
				_actuator_control_g2[i] = dual_normalized_input(_sensors.rc_input.raw[i],
																rc_cal[i][SENSOR_RC_CAL_MIN],
																rc_cal[i][SENSOR_RC_CAL_MID],
																rc_cal[i][SENSOR_RC_CAL_MAX]);
				_actuator_control_g3[i] = _actuator_control_g2[i];
			}

			//If we're using a mode switch
			if( get_param_uint(PARAM_RC_MAP_MODE_SW) ) {
				_sensors.rc_input.p_m = _sensors.rc_input.raw[get_param_uint(PARAM_RC_MAP_MODE_SW) - 1];

				if( ( _sensors.rc_input.p_m > 800 ) && ( _sensors.rc_input.p_m < 2200 ) ) {
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

	safety_button_reading = drv_sensors_safety_button_read();

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
		_sensors.voltage_monitor.state_raw = drv_sensors_battery_monitor_read();

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

	//TODO: This should be aware of failures
	return true;
}

bool sensors_read(uint32_t time_us) {
	bool new_data_read = false;

	//Return the results
	if( drv_sensors_i2c_read( time_us ) ) {
		new_data_read = sensors_update( time_us );
	}

	return new_data_read;
}

#ifdef __cplusplus
}
#endif
