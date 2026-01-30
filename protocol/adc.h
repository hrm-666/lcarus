#ifndef _adc_h_
#define _adc_h_

#include "stm32f10x.h"


#define ADC_RCC_PORT		RCC_APB2Periph_GPIOA
#define ADC_PIN					GPIO_Pin_1
#define ADC_PORT				GPIOA

#define ADC_NUM				ADC1
#define ADC_CHANNEL		ADC_Channel_1
#define NUM						1

void AdcInit(void);
uint16_t GetAdcValue(void);

#endif


