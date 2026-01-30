#include "qmc5883l.h"
#include "i2c.h"
#include "log_lib.h"

static uint8_t Qmc5883lReadByte(uint8_t reg);
static void Qmc5883lRead(uint8_t reg, uint8_t *pBuf, uint8_t len);
static void Qmc5883lWriteByte(u8 reg,u8 data);

uint8_t Qmc5883lInit(void)
{
	uint8_t data = 0;

	Qmc5883lWriteByte(QMC5883L_REG_SET_AND_RESET, 0x01);
	data = Qmc5883lReadByte(QMC5883L_REG_SET_AND_RESET);
	if(data == 0x01){
		data = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | QMC5883L_OSR_512 | QMC5883L_RNG_8G;
		Qmc5883lWriteByte(QMC5883L_REG_CONF1, data);
		return 0;
	} else {
		LogError("qmc5883l init fail.");
		return 1;
	}

}

//读取一个字节
static uint8_t Qmc5883lReadByte(u8 reg)
{
	uint8_t rec_data;
	I2cStart();
	I2cSendByte(QMC5883L_MAG_ADDRESS << 1 | 0);
	I2cWaitAck();
	I2cSendByte(reg);
	I2cWaitAck();
	
	I2cStart();
	I2cSendByte(QMC5883L_MAG_ADDRESS << 1 | 1);
	I2cWaitAck();
	rec_data = I2cReadByte(0);	//不应答
	I2cStop();
	return rec_data;
}

//读取指定个字节
static void Qmc5883lRead(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t i = 0;
	
	for(i = 0; i < len; i++){
		pBuf[i] = Qmc5883lReadByte(reg + i);
	}
	
}

//写入一个字节
static void Qmc5883lWriteByte(u8 reg,u8 data)
{
	I2cStart();
	I2cSendByte(QMC5883L_MAG_ADDRESS << 1);
	I2cWaitAck();
	I2cSendByte(reg);
	I2cWaitAck();
	
	I2cSendByte(data);
	I2cWaitAck();
	I2cStop();
}

static enum {
		STATE_WAIT_DRDY,
		STATE_READ,
} state = STATE_WAIT_DRDY;

uint8_t GetQmc5883lData(int16_t *magX, int16_t *magY, int16_t *magZ)
{
	static uint8_t status = 0; // request status on first read
	static uint8_t receiveBuf[6] = {0};
	
	switch (state) {
		case STATE_WAIT_DRDY:
			if (status & QMC5883L_REG_STATUS_DRDY) {
				//数据可用
				Qmc5883lRead(QMC5883L_REG_DATA_OUTPUT_X, receiveBuf, 6);
				state = STATE_READ;
			}
			else if (status & QMC5883L_REG_STATUS_DOR) {
				//数据溢出(和数据未准备好)。数据寄存器可能被锁定，读取解锁寄存器
				Qmc5883lRead(QMC5883L_REG_DATA_UNLOCK, receiveBuf + sizeof(receiveBuf) - 1, 1);
				status = 0;   //强制下一步读取状态
			} else {
				//读取状态寄存器检查数据是否就绪——如果读取失败，状态将保持不变
				status = Qmc5883lReadByte(QMC5883L_REG_STATUS);
			}
			return 1;
		case STATE_READ:
			*magX = (int16_t)receiveBuf[1] << 8 | receiveBuf[0];
			*magY = (int16_t)receiveBuf[3] << 8 | receiveBuf[2];
			*magZ = (int16_t)receiveBuf[5] << 8 | receiveBuf[4];

			state = STATE_WAIT_DRDY;

			// Indicate that new data is required
			status = 0;

			return 0;
		default:
			break;
	}
	
	return 1;	
}

void CompassDataTransToGauss(float *rawX, float *rawY, float *rawZ)
{
	*rawX = (*rawX) * COMPASS_RAW_TO_GASS;
	*rawY = (*rawY) * COMPASS_RAW_TO_GASS;
	*rawZ = (*rawZ) * COMPASS_RAW_TO_GASS;
}
