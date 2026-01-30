#ifndef _pwm_h_
#define _pwm_h_

#include "stm32f10x.h"

#define PWM1_RCC_PORT		RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO
#define PWM1_NUM				RCC_APB2Periph_TIM1
#define PWM1_PIN				GPIO_Pin_8

void PwmInit(void);
void Tim1Init(void);
void Tim2Init(void);
void Tim4Init(void);

void SetPwmConfig(uint16_t exArr, uint16_t exPsc, uint16_t exCcr);
void PwmOut(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4);

#endif

