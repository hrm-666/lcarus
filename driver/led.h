

#ifndef _led_h_
#define _led_h_

#include "stm32f10x.h"
#include "receive_packet.h"


#define WS2812_RCC_PORT		RCC_APB2Periph_GPIOB
#define WS2812_PIN				GPIO_Pin_15
#define WS2812_PORT				GPIOB

#define		RGB_LED_HIGH	GPIOB->BSRR = WS2812_PIN;//拉高
#define 	RGB_LED_LOW		GPIOB->BRR = WS2812_PIN;//拉低
 
#define WS2812_2_RCC_PORT		RCC_APB2Periph_GPIOC
#define WS2812_2_PIN				GPIO_Pin_13
#define WS2812_2_PORT				GPIOC

#define		WS2812_2_HIGH	WS2812_2_PORT->BSRR = WS2812_2_PIN;//拉高
#define 	WS2812_2_LOW		WS2812_2_PORT->BRR = WS2812_2_PIN;//拉低

typedef enum{
	BLACK = 1,		//黑(灭)
	WHITE,		//白
	RED ,	//红		
	GREEN,		//绿
	BLUE,			//蓝
	YELLOW,		//黄
	PURPLE,		//紫
	CYAN,			//青
}LedColor;

void Ws2812LedInit(void);
void TopLedColorSet(const LedColor ledColor);
void TopLedBlink(LedColor ledColor, uint8_t frequency, uint16_t delayMs);
void BottomLedBlink(LedColor ledColor);
#endif
//#define	TOP_RGB_RED			GPIOA->BRR = GPIO_Pin_12
//#define	TOP_RGB_GREEN		GPIOB->BRR = GPIO_Pin_14
//#define	TOP_RGB_BLUE		GPIOB->BRR = GPIO_Pin_13

//#define	BUTTOM_RGB_RED		GPIOB->BRR = GPIO_Pin_9;
//#define	BUTTOM_RGB_GREEN	GPIOB->BRR = GPIO_Pin_8;
//#define	BUTTOM_RGB_BLUE		GPIOB->BRR = GPIO_Pin_7;


//typedef enum{
//	RED = 1,	//红		
//	GREEN,		//绿
//	BLUE,			//蓝
//	YELLOW,		//黄
//	PURPLE,		//紫
//	CYAN,			//青
//	WHITE,		//白
//}LedColor;


//void Ws2812LedInit(void);

////#define led1 1
////#define led2 2
////#define led3 3
////#define led4 4

///*   LED2          LED1   */ 
//    /** *   /|\   * * *
//         *   |   *
//          *  |  *
//           * | *
//            * *
//             *
//            * *
//           *   *
//          *     *
//         *       *
//    * * *         * * */
///*  LED3            LED4   */ 



//void LeftButtomLedBlinkSet(void);
//void TopLedColorSet(uint8_t LedColor);
//void LedStatusOff(void);
//void RGB_LedStatus(Plane_S plane);
//void RGB_LedBlink(uint8_t ledColor);
//void ButtomLedColorSet(const uint8_t LedColor);
//#endif

