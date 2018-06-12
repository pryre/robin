/*
   bmp280.c :  Support for BMP280 Barometer


*/


#include <breezystm32.h>
#include <drv_bmp280.h>

#include <math.h>

// HMC5883L, default address 0x1E
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  ±0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  ±1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  ±1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  ±2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  ±4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  ±4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  ±5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  ±8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */

#define BARO_ADDRESS 0x76 //might be: 0xEC or 0xEE
#define BARO_REG_ID 0xD0
#define BARO_REG_CTRL_MEAS 0xF4
#define BARO_REG_CONFIG 0xF5
#define BARO_REG_DATA 0xF7 //6 bytes of info: press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb

#define BARO_ID 0x58
#define BARO_OVERSAMPLE_TEMP_X2 0x02
#define BARO_OVERSAMPLE_PRESS_X16 0x05
#define BARO_IIR_FILTER_MAX 0x05
#define BARO_STANDBY_MIN 0x00 //Allows for ~25Hz update rate
#define BARO_MODE_NORMAL 0x03

bool bmp280Init(int boardVersion) {
	bool success = false;


    bool ack = false;
    uint8_t sig = 0;

	//Check we have the right device
    ack = i2cReadBuffer(BARO_ADDRESS, BARO_REG_ID, 1, &sig);	//Check the identification register to make sure it is the correct device
    if(ack && (sig == BARO_ID) ) {
		//Set up the oversampling and operation mode
		uint8_t ctrl_meas = (BARO_OVERSAMPLE_TEMP_X2 << 5) | (BARO_OVERSAMPLE_PRESS_X16 << 2) | (BARO_MODE_NORMAL);
		i2cWriteRegister(BARO_ADDRESS, BARO_REG_CTRL_MEAS, ctrl_meas);

		//Set up the standby timer and measurement filter (SPI select is left as 0)
		uint8_t config = (BARO_STANDBY_MIN << 5) | (BARO_IIR_FILTER_MAX << 2);
		i2cWriteRegister(BARO_ADDRESS, BARO_REG_CONFIG, config);

		success = true;
	}

    return success;
}

/*
//XXX: Old mag function
void bmpRead(int16_t *baroData)
{
    uint8_t buf[6];

    i2cReadBuffer(BARO_ADDRESS, BARO_REG_DATA, 6, buf);
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    magData[0] = (int16_t)(buf[0] << 8 | buf[1]) * magGain[0];
    magData[2] = (int16_t)(buf[2] << 8 | buf[3]) * magGain[1];
    magData[1] = (int16_t)(buf[4] << 8 | buf[5]) * magGain[2];
}
*/

/* =================================================================
 * Asynchronous Method
 */
static uint8_t baro_buffer[6];
static int32_t *baro_data;

void baro_read_CB(void) {
	baro_data[0] = (baro_buffer[0] << 16 | baro_buffer[1] << 8 | baro_buffer[2]);		//Pressure Data
    baro_data[1] = (baro_buffer[3] << 16 | baro_buffer[4] << 8 | baro_buffer[5]);		//Pressure Data
}


//XXX: Maximum 25 Hz update rate (datasheet)
void bmp280_request_async_read(int32_t *baroData, volatile uint8_t *status) {
	baro_data = baroData;

	i2c_queue_job(READ,
					BARO_ADDRESS,
					BARO_REG_DATA,
					baro_buffer,
					6,
					status,
					&baro_read_CB);
}
