#include "i2c.h"
#include "SysTick.h"

/*******************************************************************************
* 函 数 名         : I2cInit
* 函数功能		   : I2C初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void I2cInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(I2C_SCL_PORT_RCC | I2C_SDA_PORT_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(I2C_SCL_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA_PIN;
	GPIO_Init(I2C_SDA_PORT,&GPIO_InitStructure);
	
	I2C_SCL=1;
	I2C_SDA=1;	
}

/*******************************************************************************
* 函 数 名         : SDA_OUT
* 函数功能		   : SDA输出配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(I2C_SDA_PORT,&GPIO_InitStructure);
}

/*******************************************************************************
* 函 数 名         : SDA_IN
* 函数功能		   : SDA输入配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(I2C_SDA_PORT,&GPIO_InitStructure);
}

static void I2cDelay(void)
{
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
//    volatile int i = 1;	
//    while (i)
//        i--;
}

/*******************************************************************************
* 函 数 名         : I2cStart
* 函数功能		   : 产生I2C起始信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void I2cStart(void)
{
	SDA_OUT();     //sda线输出
	I2C_SDA=1;	  	  
	I2C_SCL=1;
//	delay_us(5);
	I2cDelay();
 	I2C_SDA=0;//START:when CLK is high,DATA change form high to low 
//	delay_us(6);
	I2cDelay();
	I2C_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	

/*******************************************************************************
* 函 数 名         : I2cStop
* 函数功能		   : 产生I2C停止信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void I2cStop(void)
{
	SDA_OUT();//sda线输出
	I2C_SCL=0;
	I2C_SDA=0;//STOP:when CLK is high DATA change form low to high
 	I2C_SCL=1; 
//	delay_us(6);
I2cDelay();	
	I2C_SDA=1;//发送I2C总线结束信号
//	delay_us(6);
I2cDelay();	
}

/*******************************************************************************
* 函 数 名         : I2cWaitAck
* 函数功能		   : 等待应答信号到来   
* 输    入         : 无
* 输    出         : 1，接收应答失败
        			 0，接收应答成功
*******************************************************************************/
u8 I2cWaitAck(void)
{
	u8 tempTime=0;
	
	I2C_SDA=1;
//	delay_us(1);
	I2cDelay();
	SDA_IN();      //SDA设置为输入  	   
	I2C_SCL=1;
//	delay_us(1);
	I2cDelay();
	while(READ_SDA)
	{
		tempTime++;
		if(tempTime>250)
		{
			I2cStop();
			return 1;
		}
	}
	I2C_SCL=0;//时钟输出0 	   
	return 0;  
} 

/*******************************************************************************
* 函 数 名         : I2cAck
* 函数功能		   : 产生ACK应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void I2cAck(void)
{
	I2C_SCL=0;
	SDA_OUT();
	I2C_SDA=0;
//	delay_us(2);
	I2cDelay();
	I2C_SCL=1;
//	delay_us(5);
	I2cDelay();
	I2C_SCL=0;
}

/*******************************************************************************
* 函 数 名         : I2cNack
* 函数功能		   : 产生NACK非应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/		    
void I2cNack(void)
{
	I2C_SCL=0;
	SDA_OUT();
	I2C_SDA=1;
//	delay_us(2);
	I2cDelay();
	I2C_SCL=1;
//	delay_us(5);
	I2cDelay();
	I2C_SCL=0;
}	

/*******************************************************************************
* 函 数 名         : I2cSendByte
* 函数功能		   : I2C发送一个字节 
* 输    入         : txd：发送一个字节
* 输    出         : 无
*******************************************************************************/		  
void I2cSendByte(u8 txd)
{      

//    unsigned char i = 8;
//    
//    SDA_OUT();
//    while (i--)
//    {
//        I2C_SCL=0;                //拉低时钟开始数据传输
//        I2cDelay();
//        if (txd & 0x80)
//            I2C_SDA=1;
//        else
//            I2C_SDA=0;
//        txd <<= 1;
//        I2cDelay();
//        I2C_SCL = 1;
//        I2cDelay();;
//    }
//    I2C_SCL=0;
	
    u8 t;   
	SDA_OUT(); 	    
    I2C_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>0) //0x80  1000 0000
			I2C_SDA=1;
		else
			I2C_SDA=0;
        txd<<=1; 	  
//		delay_us(2);   //对TEA5767这三个延时都是必须的
		I2cDelay();
		I2C_SCL=1;
//		delay_us(2); 
		I2cDelay();
		I2C_SCL=0;	
//		delay_us(2);
		I2cDelay();
    }	 
} 

/*******************************************************************************
* 函 数 名         : I2cReadByte
* 函数功能		   : I2C读一个字节 
* 输    入         : ack=1时，发送ACK，ack=0，发送nACK 
* 输    出         : 应答或非应答
*******************************************************************************/  
u8 I2cReadByte(u8 ack)
{
	
//	   unsigned char i;
//    unsigned char dat = 0;
//    SDA_IN();
//    for (i=0; i<8; i++)
//    {
//        dat <<= 1;
//        I2C_SCL = 1;
//        I2cDelay();
//        dat |= READ_SDA;
//        I2C_SCL = 0;
//        I2cDelay();
//    }
//    return dat;
	u8 i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        I2C_SCL=0; 
//        delay_us(2);
		I2cDelay();
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
//		delay_us(1);
I2cDelay();		
    }					 
    if (!ack)
        I2cNack();//发送nACK
    else
        I2cAck(); //发送ACK   
    return receive;
}
