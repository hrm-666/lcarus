#ifndef _QMC5883L_H_
#define _QMC5883L_H_

#include "stm32f10x.h"

#define QMC5883L_MAG_ADDRESS 0x0D

// Registers
#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A
#define QMC5883L_REG_SET_AND_RESET	0x0B

// data output rates for 5883L
#define QMC5883L_ODR_10HZ  (0x00 << 2)
#define QMC5883L_ODR_50HZ  (0x01 << 2)
#define QMC5883L_ODR_100HZ (0x02 << 2)
#define QMC5883L_ODR_200HZ (0x03 << 2)

// Sensor operation modes
#define QMC5883L_MODE_STANDBY    0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128 (0x10 << 6)
#define QMC5883L_OSR_64  (0x11 << 6)

#define QMC5883L_RST 0x80

#define QMC5883L_REG_DATA_OUTPUT_X 0x00
#define QMC5883L_REG_DATA_UNLOCK 0x05

#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_STATUS_DRDY 0x01
#define QMC5883L_REG_STATUS_OVL  0x02
#define QMC5883L_REG_STATUS_DOR  0x04

#define QMC5883L_REG_ID 0x0D
#define QMC5883_ID_VAL 0xFF

//#define COMPASS_RAW_TO_GASS		0.00008333f		//±2G: 12000LSB/G
#define COMPASS_RAW_TO_GASS		0.00033333f		//±8G: 3000LSB/G

uint8_t Qmc5883lInit(void);
uint8_t GetQmc5883lInitStatus(void);
uint8_t GetQmc5883lData(int16_t *magX, int16_t *magY, int16_t *magZ);
void CompassDataTransToGauss(float *rawX, float *rawY, float *rawZ);
#endif
