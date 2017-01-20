#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BOARD_REV 2

// global variable declarations
//extern vector_t _accel;
//extern vector_t _gyro;
//extern float _imu_temperature;
//extern uint32_t _imu_time;
//extern bool _imu_ready;

//extern bool _diff_pressure_present;
//extern int16_t _diff_pressure;
//extern int16_t _temperature;

//extern bool _baro_present;
//extern int16_t _baro_pressure;
//extern int16_t _baro_temperature;

//extern bool _sonar_present;
//extern int16_t _sonar_range;
//extern uint32_t _sonar_time;

typedef struct {
	uint32_t dt;
	uint32_t counter;
	uint32_t start;
	uint32_t end;
	uint32_t average_time;
	uint32_t max;
	uint32_t min;
} sensor_readings_time_t;

extern sensor_readings_time_t sensor_time;

// function declarations
void init_sensors(void);
bool update_sensors(uint32_t time_us);

//bool calibrate_acc(void);
//bool calibrate_gyro(void);
