#include "led.h"
#include "systick.h"
#include "log_lib.h"

void Ws2812Led1Reset(void);

void Ws2812LedInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(WS2812_RCC_PORT ,ENABLE);
	RCC_APB2PeriphClockCmd(WS2812_2_RCC_PORT ,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = WS2812_PIN;
	GPIO_Init(WS2812_PORT, &GPIO_InitStructure);  

	GPIO_ResetBits(WS2812_PORT,WS2812_PIN);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = WS2812_2_PIN;
	GPIO_Init(WS2812_2_PORT, &GPIO_InitStructure);  

	GPIO_ResetBits(WS2812_2_PORT,WS2812_2_PIN);
	
	Ws2812Led1Reset();
}

void Ws2812Write0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->BSRR = GPIO_Pin;		//拉高
	//一个__nop()延时一个时钟周期，一个时钟周期约等于1/72000000，约13.9ns
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
	
	GPIOx->BRR = GPIO_Pin;		//拉低
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
}
 
void Ws2812Write1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->BSRR = GPIO_Pin;		//拉高
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
	
	GPIOx->BRR = GPIO_Pin;		//拉低
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
	
}
 
void Ws2812Led1Reset(void)
{
	RGB_LED_LOW;
//	delay_us(150);
	delay_ms(1);
}
 
void Ws2812Led1WriteByte(uint8_t byte)
{
	uint8_t i;
 
	for(i=0;i<8;i++)
		{
			if(byte&0x80)
				{
					Ws2812Write1(WS2812_PORT, WS2812_PIN);
			}
			else
				{
					Ws2812Write0(WS2812_PORT, WS2812_PIN);
			}
		byte <<= 1;
	}
}
 
void Ws2812Led1Write24Bits(uint8_t green,uint8_t red,uint8_t blue)
{
	Ws2812Led1WriteByte(green);
	Ws2812Led1WriteByte(red);
	Ws2812Led1WriteByte(blue);
}
 

void Ws2812Led1Red(void)
{
	 uint8_t i;
	for(i=0;i<2;i++)
		{
			Ws2812Led1Write24Bits(0, 0xff, 0);
	}
}
 
void Ws2812Led1Green(void)
{
	uint8_t i;
 
	for(i=0;i<1;i++)
		{
			Ws2812Led1Write24Bits(0xff, 0, 0);
	}
}
 
void Ws2812Led1Blue(void)
{
	uint8_t i;
 
	for(i=0;i<1;i++)
		{
			Ws2812Led1Write24Bits(0, 0, 0xff);
	}
}


//LED灯颜色设置
void TopLedColorSet(const LedColor ledColor)
{
	static LedColor lastColor = BLACK;

	//防止闪烁
	if(ledColor == lastColor){
		return;
	} else {
		lastColor = ledColor;
	}

	switch(ledColor){
		case RED:
			Ws2812Led1Red();//红
			break;
		case GREEN:
			Ws2812Led1Green();//绿
			break;
		case BLUE:
			Ws2812Led1Blue();//蓝
			break;
		case YELLOW:	//黄
			Ws2812Led1Write24Bits(0xff, 0xff, 0);
			break;
		case PURPLE:	//紫
			Ws2812Led1Write24Bits(0, 0xff, 0xff);
			break;
		case CYAN:	//青
			Ws2812Led1Write24Bits(0xff, 0, 0xff);
			break;
		case WHITE:	//白
			Ws2812Led1Write24Bits(0xff, 0xff, 0xff);
			break;
		default :
			Ws2812Led1Write24Bits(0, 0, 0);	//全灭
			break;
	}
}

/*
@func: 设置状态指示灯闪烁
@param ledColor: 灯的颜色
@param frequency: 闪烁的频率
@param delayMs: 当前线程的延时时间
*/
void TopLedBlink(LedColor ledColor, uint8_t frequency, uint16_t delayMs)
{
	static uint16_t blinkCount = 0;
	uint16_t period = 0;

	period = 1000 / (delayMs * frequency * 2);
	if(blinkCount == 0){
		TopLedColorSet(ledColor);
	}
	else if(blinkCount == period){
		TopLedColorSet(BLACK);
	}
	
	blinkCount++;
	if(blinkCount == (2 * period)){
		blinkCount = 0;
	}
}

//==========底部灯驱动==========
void BottomRgbWriteReset(void)
{
	RGB_LED_LOW;
//	delay_us(150);
	delay_ms(1);
}
 
void BottomRgbWriteByte(uint8_t byte)
{
	uint8_t i;
 
	for(i=0;i<8;i++)
		{
			if(byte&0x80)
				{
					Ws2812Write1(WS2812_2_PORT, WS2812_2_PIN);
			}
			else
				{
					Ws2812Write0(WS2812_2_PORT, WS2812_2_PIN);
			}
		byte <<= 1;
	}
}
 
void BottomRgbWrite24Bits(uint8_t green,uint8_t red,uint8_t blue)
{
	BottomRgbWriteByte(green);
	BottomRgbWriteByte(red);
	BottomRgbWriteByte(blue);
}
 

void BottomRed(void)
{
	 uint8_t i;
	for(i=0;i<2;i++)
		{
			BottomRgbWrite24Bits(0, 0xff, 0);
	}
}
 
void BottomGreen(void)
{
	uint8_t i;
 
	for(i=0;i<1;i++)
		{
			BottomRgbWrite24Bits(0xff, 0, 0);
	}
}
 
void BottomBlue(void)
{
	uint8_t i;
 
	for(i=0;i<1;i++)
		{
			BottomRgbWrite24Bits(0, 0, 0xff);
	}
}


//LED灯颜色设置
void BottomLedColorSet(const LedColor ledColor)
{
	static LedColor lastColor = BLACK;

	//防止闪烁
	if(ledColor == lastColor){
		return;
	} else {
		lastColor = ledColor;
	}

	switch(ledColor){
		case RED:
			BottomRed();//红
			break;
		case GREEN:
			BottomGreen();//绿
			break;
		case BLUE:
			BottomBlue();//蓝
			break;
		case YELLOW:	//黄
			BottomRgbWrite24Bits(0xff, 0xff, 0);
			break;
		case PURPLE:	//紫
			BottomRgbWrite24Bits(0, 0xff, 0xff);
			break;
		case CYAN:	//青
			BottomRgbWrite24Bits(0xff, 0, 0xff);
			break;
		case WHITE:	//白
			BottomRgbWrite24Bits(0xff, 0xff, 0xff);
			break;
		default :
			BottomRgbWrite24Bits(0, 0, 0);	//全灭
			break;
	}
}

/*
@func: 设置底部指示灯闪烁，一秒钟快闪两次
@param ledColor: 灯的颜色
*/
void BottomLedBlink(LedColor ledColor)
{
	static uint16_t blinkCount = 0;

	blinkCount++;
	if((blinkCount > 0 && blinkCount <= 5) || (blinkCount > 10 && blinkCount <= 15)){
		BottomLedColorSet(ledColor);
	}
	else if(blinkCount == 50){
			blinkCount = 0;
	}
	else {
		BottomLedColorSet(BLACK);
	}
	
}
