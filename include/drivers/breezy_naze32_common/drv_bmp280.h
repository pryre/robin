#pragma once

bool bmp280Init( int boardVersion );

// Blocking I2C Read Method
// void bmp280Read(int32_t *baroData);

// Asynchronous I2C method
void bmp280_request_async_read( int32_t* magData, volatile uint8_t* status );
