#include "si24r1.h"
#include "spi.h"
#include "systick.h"
#include "led.h"
#include "math_lib.h"
#include "pair_freq.h"
#include "log_lib.h"

void Si24r1GpioPin(void);
uint8_t Si24r1_Check(void);

uint8_t Si24r1Init(void)
{ 	
	uint8_t ret = 0;

	Si24r1GpioPin();

	ret = Si24r1_Check();		//在位检测
	if(ret == 1){
		LogError("si24r1 init fail.");
		return 1;
	}

	Si24r1ReceiveMode();	//接收模式

	return 0;
}

void Si24r1GpioPin(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(SI24R1_CE_RCC_PORT | SI24R1_CSN_RCC_PORT, ENABLE);		
	RCC_APB2PeriphClockCmd(SI24R1_IRQ_RCC_PORT, ENABLE);	
	//CSN
	GPIO_InitStructure.GPIO_Pin = CSN_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(SPI_CSN_PORT, &GPIO_InitStructure);
	
	//CE
	GPIO_InitStructure.GPIO_Pin = SI24R1_CE_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(SI24R1_CE_PORT, &GPIO_InitStructure);
	
	//IRQ
	GPIO_InitStructure.GPIO_Pin  = SI24R1_IRQ_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SI24R1_IRQ_PORT, &GPIO_InitStructure);
   
	NRF_CE_L; 			
	SPI_CSN_H;	
}


//无线是否在位检测
uint8_t Si24r1_Check(void)
{
	uint8_t buf[5]={0X18,0X18,0X18,0X18,0X18};
	uint8_t i;
   	 
	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);
	SPI_Read_Buf(TX_ADDR,buf,5); 
	for(i=0;i<5;i++){
        if(buf[i]!=0X18)
            break;	
			}				
	if(i!=5)
        return 1;
	return 0;		 
}	 	 

//向寄存器写入值
uint8_t SPI_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
    
   	SPI_CSN_L;        
	
  	status = Spi_RW_Byte(reg);
  	Spi_RW_Byte(value);  
    
  	SPI_CSN_H;    
    
  	return(status);       			
}

//读取寄存器值
uint8_t SPI_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	  
    
 	SPI_CSN_L;  
	
  	Spi_RW_Byte(reg);   
  	reg_val = Spi_RW_Byte(0XFF);
	
  	SPI_CSN_H;   
    
  	return(reg_val);        
}	

//读出寄存器中连续len个字节长度的值
uint8_t SPI_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;	
    
  	SPI_CSN_L;         
	
  	status = Spi_RW_Byte(reg);	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
        pBuf[u8_ctr]=Spi_RW_Byte(0XFF);
	
  	SPI_CSN_H; 
    
  	return status;        
}

//向寄存器写入连续len个字节的值
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;
    
 	SPI_CSN_L;    
	
  	status = Spi_RW_Byte(reg);
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		Spi_RW_Byte(*pBuf++);
	
  	SPI_CSN_H;   
    
  	return status;         
}			

//接收模式	   
void Si24r1ReceiveMode(void)
{
	uint8_t addr[5] = {0x1F,0x2E,0x3D,0x4C,0x5B};
	
	NRF_CE_L;	

	SPI_Write_Reg(SETUP_AW, 0x03); // 设置地址宽度为 5bytes

	SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)addr, RX_ADR_WIDTH);//设置接收地址（RX）

	SPI_Write_Reg( NRF_WRITE_REG+FEATURE, 0x06 );//使能动态负载长度及ACK应答
	SPI_Write_Reg(NRF_WRITE_REG+DYNPD, 0x01); //使能接收管道0动态负载长度

	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);      //使能通道0的自动应答
	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);	 //使能通道0的接收地址
	SPI_Write_Reg(NRF_WRITE_REG+RF_CH, 5);	   //设置频点（RF通道）
//		SPI_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);				//设置接收数据通道0有效数据宽度为11
	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);									//设置射频数据率为1MHZ，发射功率为7dBm
	SPI_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);									//配置基本工作模式的参数;开启CRC，配置为接收模式,开启所有中断
	
	NRF_CE_H; 
}						 

 /**
  * @brief :读FIFO中数据宽度
  * @param :无
  * @note  :无
  * @retval:数据宽度
  */
uint8_t NRF24L01_Read_Top_Fifo_Width( void )
{
    uint8_t btmp;
	
    SPI_CSN_L;		//片选
	
    Spi_RW_Byte( R_RX_PL_WID );	//读FIFO中数据宽度命令
    btmp = Spi_RW_Byte( 0xFF );	//读数据
	
    SPI_CSN_H;		//取消片选
	
    return btmp;
}

//接收数据包
uint8_t Si24r1_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
	uint8_t width = 0;

	sta = SPI_Read_Reg(NRF_READ_REG+STATUS); 	 					    //状态标志位
	 	
	width = NRF24L01_Read_Top_Fifo_Width();
	
	if(sta&RX_OK){		//接收成功
		SPI_Read_Buf(RD_RX_PLOAD, rxbuf, width);
		SPI_Write_Reg(FLUSH_RX,0xff);
		SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta);
		return width; 
	}	   
	return 0;
}					    



//该函数初始化Si24r1到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void Si24r1_TX_Mode(void)
{														 
	uint8_t addr[5] = {0x1F,0x2E,0x3D,0x4C,0x5B};
		
	NRF_CE_L;		 
	
	SPI_Write_Reg(SETUP_AW, 0x03); // 设置地址宽度为 5bytes

	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)addr,TX_ADR_WIDTH);    //写TX节点地址 
	SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)addr,RX_ADR_WIDTH); //设置TX节点地址,主要为了接收ACK	  

	//Si24r1_Write_Reg(NRF_WRITE_REG+FEATURE, 0x02 );//使能动态负载长度及带负载的ACK应答
	//Si24r1_Write_Reg(NRF_WRITE_REG+DYNPD, 0x01); //使能接收管道0动态负载长度

	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);               //使能通道0的自动应答    
	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);           //使能通道0的接收地址  
	SPI_Write_Reg(NRF_WRITE_REG+RF_CH, 5);  //设置RF通道
	SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);          //设置自动重发间隔时间:500us;最大自动重发次数:10次
	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);						//设置射频数据率为1MHZ，发射功率为7dBm
	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);              //配置基本工作模式的参数;开启CRC，配置为发射模式,开启所有中断
     
	NRF_CE_H;                                          //CE为高,10us后启动发送
}		  


//启动Si24r1发送一次数据
//sendBuff:待发送数据首地址
//返回值:发送完成状况
uint8_t Si24r1_TxPacket(uint8_t *sendBuff)
{
  uint8_t state;   
	NRF_CE_L;
    
  //Si24r1_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)pair.addr,RX_ADR_WIDTH);
	SPI_Write_Buf(WR_TX_PLOAD,sendBuff,TX_PLOAD_WIDTH);
    
 	NRF_CE_H;		//启动发送
    
	while(NRF_IRQ!=0);		//等待发送完成
    
	state=SPI_Read_Reg(NRF_WRITE_REG+STATUS);		//读取状态寄存器的值	   
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,state);		//清除TX_DS或MAX_RT中断标志
    
	if(state&MAX_TX){		//达到最大重发次数	
    SPI_Write_Reg(FLUSH_TX,0xff);		//清除TX FIFO寄存器 	
		return MAX_TX; 
	}
	if(state&TX_OK){		//发送完成
		return TX_OK;
	}
	return 0xff;		//其他原因发送失败
}



