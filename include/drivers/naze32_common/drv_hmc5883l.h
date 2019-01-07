#pragma once

#define HMC5883L_GAIN_FACTOR 1090

bool hmc5883lInit( int boardVersion );

// Blocking I2C Read Method
void hmc5883lRead( int16_t* magData );

// Asynchronous I2C method
void hmc5883l_request_async_read( int16_t* magData, volatile uint8_t* status );
// void hmc5883l_request_async_update();
// void hmc5883l_read_magnetometer(int16_t *magData);
