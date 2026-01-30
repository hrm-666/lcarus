#include "bmp280.h"
#include "i2c.h"
#include "math.h"
#include "my_lib.h"
#include "log_lib.h"

typedef	long signed int			BMP280_S32_t;	//有符号 64位！
typedef	long unsigned int		BMP280_U32_t;	//无符号 32位！
typedef	long long signed int	BMP280_S64_t;

static Bmp280CalValue_T bmp280CalValue = {0};
static BMP280_S32_t t_fine = 0;

static uint8_t Bmp280ReadByte(uint8_t reg);
static void Bmp280WriteByte(uint8_t reg,uint8_t data);
static void Bmp280GetCalValue(void);
static void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE * Oversample_Mode);
static void BMP280_Set_Standby_FILTER(BMP_CONFIG * BMP_Config);

/*bmp280初始化
返回值	1，成功；0，失败*/
uint8_t Bmp280Init(void)
{
	uint8_t ret = 0;
	
	Bmp280GetCalValue();
	Bmp280WriteByte(BMP280_RESET_REG, 0xB6);		//复位
	
	ret = Bmp280ReadByte(BMP280_CHIPID_REG);		//检查ID是否正确
	if(ret == 0x58){
		BMP_OVERSAMPLE_MODE BMP_OVERSAMPLE_MODEStructure;
		BMP_OVERSAMPLE_MODEStructure.P_Osample = BMP280_P_MODE_5;
		BMP_OVERSAMPLE_MODEStructure.T_Osample = BMP280_T_MODE_2;
		BMP_OVERSAMPLE_MODEStructure.WORKMODE  = BMP280_NORMAL_MODE;
		BMP280_Set_TemOversamp(&BMP_OVERSAMPLE_MODEStructure);
		
		BMP_CONFIG BMP_CONFIGStructure;
		BMP_CONFIGStructure.T_SB = BMP280_T_SB1;
		BMP_CONFIGStructure.FILTER_COEFFICIENT = BMP280_FILTER_MODE_4;
		BMP_CONFIGStructure.SPI_EN = DISABLE;
		BMP280_Set_Standby_FILTER(&BMP_CONFIGStructure);
		
		return 0;
	} else{
		LogError("bmp280 init fail.");
		return 1;
	}

}

static void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE * Oversample_Mode)
{
	uint8_t Regtmp;
	Regtmp = ((Oversample_Mode->T_Osample)<<5)|
			 ((Oversample_Mode->P_Osample)<<2)|
			 ((Oversample_Mode)->WORKMODE);
	
	Bmp280WriteByte(BMP280_CTRLMEAS_REG,Regtmp);
}


static void BMP280_Set_Standby_FILTER(BMP_CONFIG * BMP_Config)
{
	uint8_t Regtmp;
	Regtmp = ((BMP_Config->T_SB)<<5)|
			 ((BMP_Config->FILTER_COEFFICIENT)<<2)|
			 ((BMP_Config->SPI_EN));
	
	Bmp280WriteByte(BMP280_CONFIG_REG,Regtmp);
}

//读取一个字节
static uint8_t Bmp280ReadByte(uint8_t reg)
{
	uint8_t rec_data;
	I2cStart();
	I2cSendByte(BMP280_ADDRESS<<1|0);
	I2cWaitAck();
	I2cSendByte(reg);
	I2cWaitAck();
	
	I2cStart();
	I2cSendByte(BMP280_ADDRESS<<1|1);
	I2cWaitAck();
	rec_data = I2cReadByte(0);	//不应答
	I2cStop();
	return rec_data;
}

//读取指定个字节
static void Bmp280Read(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t i = 0;
	
	for(i = 0; i < len; i++){
		pBuf[i] = Bmp280ReadByte(reg + i);
	}
	
}

//写入一个字节
static void Bmp280WriteByte(uint8_t reg,uint8_t data)
{
	I2cStart();
	I2cSendByte(BMP280_ADDRESS<<1);
	I2cWaitAck();
	I2cSendByte(reg);
	I2cWaitAck();
	
	I2cSendByte(data);
	I2cWaitAck();
	I2cStop();
}

//读取矫正值
static void Bmp280GetCalValue(void)
{
	uint8_t calArr[2] = {0};

	//温度传感器矫正值
	Bmp280Read(BMP280_DIG_T1_LSB_REG, calArr,2);
	bmp280CalValue.t1 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_T2_LSB_REG, calArr,2);
	bmp280CalValue.t2 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_T3_LSB_REG, calArr,2);
	bmp280CalValue.t3 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	//大气压传感器的矫正值
	Bmp280Read(BMP280_DIG_P1_LSB_REG, calArr,2);
	bmp280CalValue.p1 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P2_LSB_REG, calArr,2);
	bmp280CalValue.p2 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P3_LSB_REG, calArr,2);
	bmp280CalValue.p3 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P4_LSB_REG, calArr,2);
	bmp280CalValue.p4 = ((uint16_t)calArr[1] << 8) | calArr[0];

	Bmp280Read(BMP280_DIG_P5_LSB_REG, calArr,2);
	bmp280CalValue.p5 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P6_LSB_REG, calArr,2);
	bmp280CalValue.p6 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P7_LSB_REG, calArr,2);
	bmp280CalValue.p7 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P8_LSB_REG, calArr,2);
	bmp280CalValue.p8 = ((uint16_t)calArr[1] << 8) | calArr[0];
	
	Bmp280Read(BMP280_DIG_P9_LSB_REG, calArr,2);
	bmp280CalValue.p9 = ((uint16_t)calArr[1] << 8) | calArr[0];

}


//摘自bmp280数据手册
/**************************传感器值转定点值*************************************/
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bmp280_compensate_T_double(BMP280_S32_t  adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)bmp280CalValue.t1)/1024.0) * ((double)bmp280CalValue.t2);
	var2 = ((((double)adc_T)/131072.0 - ((double)bmp280CalValue.t1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) bmp280CalValue.t1)/8192.0)) * ((double)bmp280CalValue.t3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(BMP280_S32_t  adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)bmp280CalValue.p6) / 32768.0;
	var2 = var2 + var1 * ((double)bmp280CalValue.p5) * 2.0;
	var2 = (var2/4.0)+(((double)bmp280CalValue.p4) * 65536.0);
	var1 = (((double)bmp280CalValue.p3) * var1 * var1 / 524288.0 + ((double)bmp280CalValue.p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)bmp280CalValue.p1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)bmp280CalValue.p9) * p * p / 2147483648.0;
	var2 = p * ((double)bmp280CalValue.p8) / 32768.0;
	p = p + (var1 + var2 + ((double)bmp280CalValue.p7)) / 16.0;
	return p;
}

//获取BMP当前状态
//status_flag = BMP280_MEASURING ||
//			 	BMP280_IM_UPDATE
uint8_t BMP280_GetStatus(uint8_t status_flag)
{
	uint8_t flag;
	flag = Bmp280ReadByte(BMP280_STATUS_REG);
	if(flag&status_flag)	return 1;
	else return 0;
}


/*读取大气压值
返回值：气压值*/
double Bmp280GetPressure(void)
{
	uint8_t msb,lsb,xlsb;
	BMP280_S32_t  rawPressure;
	double pressure;
	
	xlsb = Bmp280ReadByte(BMP280_PRESSURE_XLSB_REG);
	lsb	 = Bmp280ReadByte(BMP280_PRESSURE_LSB_REG);
	msb	 = Bmp280ReadByte(BMP280_PRESSURE_MSB_REG);
	rawPressure = ((long)(msb << 12))|((long)(lsb << 4))|(xlsb>>4);	//寄存器的值组合起来
	
	pressure = bmp280_compensate_P_double(rawPressure);
	return pressure;
}

/*读取温度值
返回值：温度值*/
double BMP280_Get_Temperature(void)
{
	uint8_t XLsb,Lsb, Msb;
	long signed Bit32;
	double temperature;
	XLsb = Bmp280ReadByte(BMP280_TEMPERATURE_XLSB_REG);
	Lsb	 = Bmp280ReadByte(BMP280_TEMPERATURE_LSB_REG);
	Msb	 = Bmp280ReadByte(BMP280_TEMPERATURE_MSB_REG);
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);	//寄存器的值,组成一个浮点数
	temperature = bmp280_compensate_T_double(Bit32);
	return temperature;
}

