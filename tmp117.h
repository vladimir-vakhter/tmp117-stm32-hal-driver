/*
 ******************************************************************************
 * @file:			tmp117.h
 * @author: 	Vladimir Vakhter
 * @version:	0.1a
 * @date: 		Apr 26, 2022
 * @brief:		STM32 HAL-based library for the TMP117 temperature sensor.
 * 						Datasheet: http://www.ti.com/lit/ds/symlink/tmp117.pdf
 *
 * @acknowledgements:
 * 						- Nils Minor (https://github.com/NilsMinor/TMP117-Arduino)
 * 						- Shivam Gupta (https://github.com/shiivamgupta/TMP117-High-Precision-Digital-Temperature-Sensor)
 ******************************************************************************     
 */

#ifndef INC_ICAS_TMP117_H_
#define INC_ICAS_TMP117_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <i2c.h>
#include <math.h>

/* user defines ------------------------- */

/* resolution ----- */
#define TMP117_RESOLUTION       (double)0.0078125																// deg C per 1 LSB

/* pointer registers ----- */
// name			 										address		description														type (read/write)		reset
#define TMP117_CONFIG_REG				0x01			// configuration register							R/W									0x0220
#define TMP117_DEVICE_ID_REG		0x0F			// device ID register									R                   0x0117

#define TMP117_TEMP_RES_REG 		0x00			// temperature result register				R										0x8000
#define TMP117_THIGH_LIM_REG		0x02			// temperature high limit register		R/W                 0x6000
#define TMP117_TLOW_LIM_REG			0x03			// temperature low limit register			R/W                 0x8000
#define TMP117_TEMP_OFFSET_REG	0x07			// temperature offset register				R/W                 0x0000

#define TMP117_EEPROM_UL_REG		0x04			// EEPROM unlock register							R/W                 0x0000
#define TMP117_EEPROM1_REG			0x05			// EEPROM1 register										R/W                 0xxxxx
#define TMP117_EEPROM2_REG			0x06			// EEPROM2 register										R/W                 0xxxxx
#define TMP117_EEPROM3_REG			0x08			// EEPROM3 register										R/W                 0xxxxx

/* I2C addresses ----- */
// ADD0 = GND (default in this library)
#define TMP117_I2C_R_ADDR_GND 	0x91			// 1001 0001			read
#define TMP117_I2C_W_ADDR_GND		0x90			// 1001 0000			write

// ADD0 = V+
#define TMP117_I2C_R_ADDR_VP		0x93			// 1001 0011			read
#define TMP117_I2C_W_ADDR_VP		0x92			// 1001 0010			write

// ADD0 = SDA
#define TMP117_I2C_R_ADDR_SDA		0x95			// 1001 0101			read
#define TMP117_I2C_W_ADDR_SDA		0x94      // 1001 0100			write

// ADD0 = SCL
#define TMP117_I2C_R_ADDR_SCL		0x97			// 1001 0111			read
#define TMP117_I2C_W_ADDR_SCL		0x96      // 1001 0110			write

/* data types ------------------------- */
typedef void (*allert_callback)(void);		// "allert_callback f;" compiles equally as "void (*f)();"

/* Conversion averaging modes:
 * 		- NO_AVG - no averaging; AVG_8, AVG_32, and AVG_64 are 8, 32, and 64 averaged conversions accordingly
 * 		- determines the number of conversions taken before the temperature register is updated
 * 		- the average is an accumulated and not a running one (reported once after all conversions were averaged)
 * 		- averaging reduces the noise, but increases the active power consumption
 * 		- default (factory EEPROM) mode: AVG8 with a conversion time of 1 second
 */
typedef enum {
	TMP117_NO_AVG = 0,	TMP117_AVG_8, TMP117_AVG_32, TMP117_AVG_64
} TMP117_AVG_MODE;

/*
 * Conversion modes:
 * 		- CC - continuous conversion, SD - shutdown, and OS - one-shot conversion
 * 		- the sensor cannot start in OS, by it can start either in CC or in SD
 */
typedef enum {
	TMP117_CC_MODE = 0,	TMP117_SD_MODE = 1, TMP117_OS_MODE = 3
} TMP117_CONV_MODE;

/*
 * Therm/alert/data mode select:
 * 		- ALERT - alert mode (window-limit detector):
 * 				-- at the end of each conversion, the sensor compares the temperature with the low- and high-limit registers
 * 				-- if the temperature exceeds the high limit register value, the HIGH_Alert status flag is set in the config.register
 * 				-- if the temperature is lower than the low limit register value, the LOW_Alert status flag is set in the config.register
 * 				-- the user can run an I2C read from the configuration register to clear the status flags in alert mode
 * 				-- the user can also run an SMBus alert response command to deassert the ALERT pin
 * 		- THERM - therm mode (high-limit threshold detector):
 * 				-- at the end of each conversion, the sensor compares the temperature with the low- and high-limit registers
 * 				-- the sensor sets the HIGH_Alert flag if the temperature exceeds the high limit register value
 * 				-- the sensor clears the HIGH_Alert flag if the temperature goes below the low limit register value
 * 				-- the LOW_Alert flag is disabled and always reads 0; I2C reads of the config.register do not affect the status bits
 * 				-- the ALERT pin can't be cleared by an I2C read of the config.register or by an SMBus alert response command
 * 		- DATA - "thermometer" mode (not named this way in the datasheet)
 * 				-- the ALERT pin reflects the status of the data ready flag
 */
typedef enum {
	TMP117_ALERT_MODE = 0, TMP117_THERM_MODE, TMP117_DATA_MODE
} TMP117_TnA_MODE;

/*
 * Alert pin polarity:
 * 		- TMP117_POL_L - active low, TMP117_POL_H - active high
 */
typedef enum {
	TMP117_POL_L = 0, TMP117_POL_H
} TMP117_ALERT_PIN_POL;

/*  Conversion Cycle Time in CC Mode
*    ------------------------------------------------------------------
*   |CONV | AVG0(0)  | AVG1(8) |  AVG2(32) |  AVG3(64) | constant name |
*    ------------------------------------------------------------------
*   |  0  | 15.5ms   | 125ms   |  500ms    |  1s       | TMP117_C15mS5 |
*   |  1  | 125ms    | 125ms   |  500ms    |  1s       | TMP117_C125mS |
*   |  2  | 250ms    | 250ms   |  500ms    |  1s       | TMP117_C250mS |
*   |  3  | 500ms    | 500ms   |  500ms    |  1s       | TMP117_C500mS |
*   |  4  | 1s       | 1s      |  1s       |  1s       | TMP117_C1S    |
*   |  5  | 4s       | 4s      |  4s       |  4s       | TMP117_C4S    |
*   |  6  | 8s       | 8s      |  8s       |  8s       | TMP117_C8S    |
*   |  7  | 16s      | 16s     |  16s      |  16s      | TMP117_C16S   |
*    ------------------------------------------------------------------
*/
typedef enum {
	TMP117_C15mS5 = 0, TMP117_C125mS, TMP117_C250mS, TMP117_C500mS,
	TMP117_C1S, TMP117_C4S, TMP117_C8S, TMP117_C16S
} TMP117_CONV_TIME;

/* public functions (API) ----------------------- */

/* settings --------------- */
bool init(I2C_HandleTypeDef* i2c, uint8_t buffer[]); 																							// initialize the sensor
bool setAveraging(I2C_HandleTypeDef* i2c, uint8_t buffer[], TMP117_AVG_MODE avg);									// set averaging
bool setAlertMode(I2C_HandleTypeDef* i2c, uint8_t buffer[], TMP117_TnA_MODE mode);								// set alert mode
bool setAlertPolarity(I2C_HandleTypeDef* i2c, uint8_t buffer[], TMP117_ALERT_PIN_POL polarity);		// set alert polarity
bool setConversionMode(I2C_HandleTypeDef* i2c, uint8_t buffer[], TMP117_CONV_MODE mode);					// set conversion mode
bool setConversionTime(I2C_HandleTypeDef* i2c, uint8_t buffer[], TMP117_CONV_TIME conv_time);			// set conversion time
bool softwareReset(I2C_HandleTypeDef* i2c, uint8_t buffer[]);																			// software reset

bool getConfig(I2C_HandleTypeDef* i2c, uint8_t buffer[], uint16_t* config);												// get configuration register value
bool getDeviceID(I2C_HandleTypeDef* i2c, uint8_t buffer[], uint16_t* id);													// get device ID

/* temperature --------------- */
bool setHighLimitTemperature(I2C_HandleTypeDef* i2c, uint8_t buffer[], double temp);							// set high limit temperature
bool setLowLimitTemperature(I2C_HandleTypeDef* i2c, uint8_t buffer[], double temp);               // set low limit temperature

bool getResultTemperature(I2C_HandleTypeDef* i2c, uint8_t buffer[], double* temp_c); 							// get result temperature
bool getOffsetTemperature(I2C_HandleTypeDef* i2c, uint8_t buffer[], double* offset);							// get offset temperature
bool getHighLimitTemperature(I2C_HandleTypeDef* i2c, uint8_t buffer[], double* temp);							// get high limit temperature
bool getLowLimitTemperature(I2C_HandleTypeDef* i2c, uint8_t buffer[], double* temp);							// get low limit temperature

/* calibration --------------- */
bool calibrate(I2C_HandleTypeDef* i2c, uint8_t buffer[], double target_temp);											// calibrate sensor

/* EEPROM --------------- */
bool readEEPROM (I2C_HandleTypeDef* i2c, uint8_t buffer[], uint8_t eeprom_num, uint16_t* data);		// read EEPROM
bool writeEEPROM (I2C_HandleTypeDef* i2c, uint8_t buffer[], uint16_t data, uint8_t eeprom_num);		// write EEPROM

#endif /* INC_ICAS_TMP117_H_ */
