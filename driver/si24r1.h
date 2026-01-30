#ifndef _SI24R1_H_
#define _SI24R1_H_

#include "stm32f10x.h"

#define NRF_READ_REG    0x00  
#define NRF_WRITE_REG   0x20  
#define RD_RX_PLOAD     0x61  
#define WR_TX_PLOAD     0xA0  
#define FLUSH_TX        0xE1  
#define FLUSH_RX        0xE2  
#define REUSE_TX_PL     0xE3  
#define R_RX_PL_WID     0x60
#define NOP             0xFF  
#define W_ACK_PLOAD		0xA8
#define CONFIG          0x00  
                              
#define EN_AA           0x01  
#define EN_RXADDR       0x02  
#define SETUP_AW        0x03  
#define SETUP_RETR      0x04  
#define RF_CH           0x05  
#define RF_SETUP        0x06  
#define STATUS          0x07  
                              
#define MAX_TX  		0x10  
#define TX_OK   		0x20  
#define RX_OK   		0x40  

#define OBSERVE_TX      0x08  
#define CD              0x09  
#define RX_ADDR_P0      0x0A  
#define RX_ADDR_P1      0x0B  
#define RX_ADDR_P2      0x0C  
#define RX_ADDR_P3      0x0D  
#define RX_ADDR_P4      0x0E  
#define RX_ADDR_P5      0x0F  
#define TX_ADDR         0x10  
#define RX_PW_P0        0x11  
#define RX_PW_P1        0x12  
#define RX_PW_P2        0x13  
#define RX_PW_P3        0x14  
#define RX_PW_P4        0x15  
#define RX_PW_P5        0x16  
#define NRF_FIFO_STATUS 0x17  
                              
#define FEATURE			0x1D
#define DYNPD			0x1C

//CSN
#define SI24R1_CSN_RCC_PORT		RCC_APB2Periph_GPIOB
#define SPI_CSN_PORT					GPIOB
#define CSN_PIN								GPIO_Pin_0
#define SPI_CSN_L  						GPIO_ResetBits(SPI_CSN_PORT, CSN_PIN)
#define SPI_CSN_H  						GPIO_SetBits(SPI_CSN_PORT, CSN_PIN)

//CE
#define SI24R1_CE_RCC_PORT		RCC_APB2Periph_GPIOB
#define SI24R1_CE_PORT				GPIOB
#define SI24R1_CE_PIN					GPIO_Pin_1
#define NRF_CE_L   						GPIO_ResetBits(SI24R1_CE_PORT, SI24R1_CE_PIN)
#define NRF_CE_H   						GPIO_SetBits(SI24R1_CE_PORT, SI24R1_CE_PIN)

//IRQ
#define SI24R1_IRQ_RCC_PORT		RCC_APB2Periph_GPIOA
#define SI24R1_IRQ_PORT				GPIOA
#define SI24R1_IRQ_PIN				GPIO_Pin_4





#define NRF_IRQ    GPIO_ReadInputDataBit(SI24R1_IRQ_PORT, SI24R1_IRQ_PIN)


#define TX_ADR_WIDTH    5   
#define RX_ADR_WIDTH    5   
#define TX_PLOAD_WIDTH  11  	
#define RX_PLOAD_WIDTH  11  	
									   	   

u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);	
u8 SPI_Read_Reg(u8 reg);					
u8 SPI_Write_Reg(u8 reg, u8 value);		
				
void Si24r1ReceiveMode(void);					
void Si24r1_TX_Mode(void);	
u8 Si24r1_Check(void);						
u8 Si24r1_TxPacket(u8 *txbuf);				
uint8_t Si24r1_RxPacket(uint8_t *rxbuf);
uint8_t Si24r1Init(void);


#endif



