#ifndef I2C_H
#define I2C_H
#include <stdint.h>
#include "stm32f4xx.h"

void I2C_configure(void);



typedef struct _I2C_Dev
{
	I2C_TypeDef *I2Cx;
	char error;
	int timeout_counter;
} I2C_Bus;

extern I2C_Bus I2C1_Bus;
extern I2C_Bus I2C2_Bus;

/**
 * \brief  Устанавливает значение регистра по I2C
 *
 * \param deviceAddr	адрес I2C устройства
 * \param reg			адрес регистра
 * \param value			значение регистра
 */
int I2C_RegWrite(I2C_Bus *I2Cx, uint8_t deviceAddr, uint8_t reg, uint8_t value);

/**
 * \brief Считывает значение регистра по I2C
 *
 * \param deviceAddr	адрес I2C устройства
 * \param reg			адрес регистра
 * \rerurn	Значение регистра
 */
uint8_t I2C_RegRead(I2C_Bus *I2Cx, uint8_t deviceAddr, uint8_t reg);

/**
 * \brief Считывает значение 16 битного регистра по I2C
 *
 * \param deviceAddr	адрес I2C устройства
 * \param reg			адрес регистра
 * \rerurn	Значение регистра
 */
uint16_t I2C_RegRead16(I2C_Bus *I2Cx, uint8_t deviceAddr, uint8_t addr);

int I2C_RegWrite16(I2C_Bus *i2cDev, uint8_t deviceAddr, uint8_t addr, uint16_t value);

int TMP116_generalCallResetCommand(I2C_Bus *i2cDev); //

int I2C_MultipleRegRead(I2C_Bus *I2Cx, uint8_t deviceAddr, int count, uint8_t *regAddr, uint8_t *values);

void I2C_configure(void);

uint16_t readID(void);

uint16_t readLowLimit(void);

uint16_t readHighLimit(void);

void TMP116_enterShutdownMode(void);

void TMP116_enterContinuousConversionMode(void);

float TMP116_measureTemperatureFromShutDownMode(void);

float TMP116_getTemperature(void);

int TMP116_writeRegister( I2C_Bus *i2cDev, uint8_t addr, uint16_t value); //common write sequence (dont use this)

int TMP116_writeConfiguration(uint16_t value); //use this 
uint16_t TMP116_readConfiguration(void);




#endif
