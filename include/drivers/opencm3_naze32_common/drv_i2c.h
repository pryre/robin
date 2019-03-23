/*
   i2c.h :  I^2C support for STM32F103CB

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/i2c.h

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define I2C_BUFFER_SIZE 32
#define I2C_DEFAULT_TIMEOUT 30000

typedef enum {
    I2C_JOB_TYPE_READ,
	I2C_JOB_TYPE_WRITE
} drv_i2c_job_type_t;

typedef enum {
    I2C_JOB_DEFAULT,
    I2C_JOB_QUEUED,
    I2C_JOB_BUSY,
    I2C_JOB_COMPLETE,
    I2C_JOB_ERROR
} drv_i2c_job_status_t;

typedef enum {
    I2C_JOB_ERROR_NONE,
    I2C_JOB_ERROR_ACK,
    I2C_JOB_ERROR_BUS,
    I2C_JOB_ERROR_ARBITRATION,
    I2C_JOB_ERROR_GENERIC
} drv_i2c_job_error_t;

typedef struct {
    uint32_t i2c;
    drv_i2c_job_type_t type;
    uint8_t addr;
    uint8_t reg;
    volatile uint8_t* data;
    uint8_t length;
	struct drv_i2c_job_t* next_job;
    volatile drv_i2c_job_status_t* status;
    void (*CB)(void);

	bool busy;
	drv_i2c_job_error_t error;
} drv_i2c_job_t;

typedef struct {
	drv_i2c_job_t buffer[I2C_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
} drv_i2c_jb_t;

void     drv_i2c_init(uint32_t i2c);
uint32_t drv_i2c_get_error_count(uint32_t i2c);

// Blocking I2C functions (returns value success or failure)
bool drv_i2c_read_buffer(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
bool drv_i2c_write_buffer(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
bool drv_i2c_read_register(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t *data);
bool drv_i2c_write_register(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data);

// ===================================================================
// Asynchronous I2C handler
// To use this, queue up a job, and create a callback you want called when the job is finished
// You can track progress of the job using the status pointer.  Otherwise, functions the same
// as the blocking versions.
//
// This uses a circular buffer to stage jobs for the I2C peripheral.  The buffer is, by default, 64 jobs
// long (I2C_BUFFER_SIZE), with a maximum size of 256. I hope you never queue up that many jobs, because
// that will take a long time to process However, if you were to reach the limit, it would then start
// ignoring new jobs until there was space on the buffer.
//
// For an example of how to use, check out mpu6050_request_read_temp - the non-blocking way to read
// the accelerometer
void drv_i2c_queue_job(uint32_t i2c, drv_i2c_job_type_t type, uint8_t addr, uint8_t reg,
        volatile uint8_t *data, uint8_t length, volatile drv_i2c_job_status_t *status, void (*CB)(void));
