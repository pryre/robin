#pragma once

bool bmp280Init( uint32_t i2c, int boardVersion );

// Blocking I2C Read Method
void bmp280Read(uint32_t i2c, volatile int16_t *baroData);

// Asynchronous I2C method
void bmp280_request_async_read(uint32_t i2c, volatile int32_t* baroData, volatile uint8_t* status );
