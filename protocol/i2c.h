#ifndef _i2c_H
#define _i2c_H

#include "system.h"

/*  I2C_SCL时钟端口、引脚定义 */
#define I2C_SCL_PORT 			GPIOB   
#define I2C_SCL_PIN 			(GPIO_Pin_7)
#define I2C_SCL_PORT_RCC		RCC_APB2Periph_GPIOB

/*  I2C_SDA时钟端口、引脚定义 */
#define I2C_SDA_PORT 			GPIOB  
#define I2C_SDA_PIN 			(GPIO_Pin_6)
#define I2C_SDA_PORT_RCC		RCC_APB2Periph_GPIOB

//IO操作函数	 
#define I2C_SCL    PBout(7) //SCL
#define I2C_SDA    PBout(6) //SDA	 
#define READ_SDA   PBin(6)  //输入SDA

//I2C所有操作函数
void I2cInit(void);                //初始化I2C的IO口				 
void I2cStart(void);				//发送I2C开始信号
void I2cStop(void);	  			//发送I2C停止信号
void I2cSendByte(u8 txd);			//I2C发送一个字节
u8 I2cReadByte(u8 ack);			//I2C读取一个字节
u8 I2cWaitAck(void); 				//I2C等待ACK信号
void I2cAck(void);					//I2C发送ACK信号
void I2cNack(void);				//I2C不发送ACK信号


#endif
