/*
   mpu.c : driver for Invensense MPU devices (currently just MPU6050)

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_mpu.c

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


#include <breezystm32.h>
#include <drivers/naze32_common/drv_mpu.h>

#include <math.h>

void (*mpuInterruptCallbackPtr)(void) = NULL;

bool mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length)
{
    return i2cReadBuffer(MPU_ADDRESS, reg, length, data);
}

bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    return i2cWriteRegister(MPU_ADDRESS, reg, data);
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        if(mpuInterruptCallbackPtr != NULL)
        {
            mpuInterruptCallbackPtr();
        }
    }
    EXTI_ClearITPendingBit(EXTI_Line13);
}

void mpu_read_accel(int16_t *accData)
{
    uint8_t buf[6];

    mpuReadRegisterI2C(MPU_RA_ACCEL_XOUT_H, buf, 6);

    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}


void mpu_read_gyro(int16_t *gyroData)
{
    uint8_t buf[6];

    mpuReadRegisterI2C(MPU_RA_GYRO_XOUT_H, buf, 6);

    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

void mpu_read_temperature(int16_t *tempData)
{
    uint8_t buf[2];

    mpuReadRegisterI2C(MPU_RA_TEMP_OUT_A, buf, 2);

    *tempData = (int16_t)((buf[0] << 8) | buf[1]) / 4;
}

// XXX we should figure out how to make interrupts with with F3 as well
#ifdef STM32F10X_MD
void mpu_exti_init(int boardVersion)
{
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configure EXTI
    EXTI_ClearITPendingBit(EXTI_Line13);
    EXTI_InitTypeDef EXTI_InitStrutcure;
    // GPIO Structure Used To initialize external interrupt pin
    // This assumes that the interrupt pin is attached to pin 26 (PB13)
    // Which is not be the case for all boards. The naze32 rev5+ has it's
    // interrupt on PC13, while rev4- and the flip32 devices use PB13.
    // see src/main/sensors/initializiation.c:85 in the cleanflight source code
    // for their version handling.
    if (boardVersion > 4) {
        gpioExtiLineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
    } else {
        gpioExtiLineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    }

    // Configure EXTI Line13
    EXTI_InitStrutcure.EXTI_Line = EXTI_Line13;
    EXTI_InitStrutcure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStrutcure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStrutcure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStrutcure);

    // Disable AFIO Clock - we don't need it anymore
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, DISABLE);

    // Configure NVIC (Nested Vector Interrupt Controller)
    NVIC_InitTypeDef NVIC_InitStructure;
    // Select NVIC Channel to configure
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    // Set priority to lowest
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    // Set subpriority to lowest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // Update NVIC registers
    NVIC_Init(&NVIC_InitStructure);
}
#endif

/*=======================================================
 * Asynchronous I2C Read Functions:
 * These methods use the asynchronous I2C
 * read capability on the naze32.
 */

// Allocate storage for asynchronous I2C communcation
static uint8_t accel_buffer[6];
static volatile int16_t* accel_data;

// This function is called when the I2C job is finished
void accel_read_CB(void)
{
    accel_data[0] = (int16_t)((accel_buffer[0] << 8) | accel_buffer[1]);
    accel_data[1] = (int16_t)((accel_buffer[2] << 8) | accel_buffer[3]);
    accel_data[2] = (int16_t)((accel_buffer[4] << 8) | accel_buffer[5]);
}

#ifdef STM32F10X_MD
void mpu_request_async_accel_read(int16_t *accData, volatile uint8_t *status)
{
    accel_data = accData;
    // Adds a new i2c job to the I2C job queue.
    // Current status of the job can be read by polling the
    // status variable, and the callback will be called when the function
    // is finished
    i2c_queue_job(READ,
                  MPU_ADDRESS,
                  MPU_RA_ACCEL_XOUT_H,
                  accel_buffer,
                  6,
                  status,
                  &accel_read_CB);
}
#endif

static uint8_t gyro_buffer[6];
static volatile int16_t* gyro_data;
void gyro_read_CB(void)
{
    gyro_data[0] = (int16_t)((gyro_buffer[0] << 8) | gyro_buffer[1]);
    gyro_data[1] = (int16_t)((gyro_buffer[2] << 8) | gyro_buffer[3]);
    gyro_data[2] = (int16_t)((gyro_buffer[4] << 8) | gyro_buffer[5]);
}

void mpu_request_async_gyro_read(int16_t *gyroData, volatile uint8_t *status)
{
    gyro_data = gyroData;
    i2c_queue_job(READ,
                  MPU_ADDRESS,
                  MPU_RA_GYRO_XOUT_H,
                  gyro_buffer,
                  6,
                  status,
                  &gyro_read_CB);
}

static uint8_t temp_buffer[2];
static volatile int16_t* temp_data;
void temp_read_CB(void)
{
    //LED0_ON;
    (*temp_data) = (int16_t)((temp_buffer[0] << 8)| temp_buffer[1])/4;
    //LED0_OFF;
}

void mpu_request_async_temp_read(volatile int16_t *tempData, volatile uint8_t *status)
{
    temp_data = tempData;
    i2c_queue_job(READ,
                  MPU_ADDRESS,
                  MPU_RA_TEMP_OUT_A,
                  temp_buffer,
                  2,
                  status,
                  &temp_read_CB);
}


/*=======================================================
 * Custom ISR Registration
 * This method registers a custom interrpt to be
 * run upon the interrupt pin on the MPU6050 going high
 */
void mpu_register_interrupt_cb(void (*functionPtr)(void), int boardVersion)
{
    gpio_config_t gpio;
    gpio.pin = Pin_13;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    if (boardVersion > 4){
        gpioInit(GPIOC, &gpio);
    } else {
        gpioInit(GPIOB, &gpio);
    }
    mpu_exti_init(boardVersion);

    mpuInterruptCallbackPtr = functionPtr;
}
