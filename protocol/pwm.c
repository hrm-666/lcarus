//#include "pwm.h"
//#include "math_lib.h"
//#include "board_config.h"



//#if BRUSHED_FOUR_AXIS_UAV
//uint16_t arrValue = 1000-1;
//u8 pscValue = 8-1;
//u8 ccrValue = 0;
//#elif FIXED_WING_AIRCRAFT
//uint16_t arrValue = 1000-1;
//u8 pscValue = 8-1;
//u8 ccrValue = 0;
//#elif BRUSHLESS_FOUR_AXIS_UAV
//uint16_t arrValue = 10000-1;
//u8 pscValue = 16-1;
//u8 ccrValue = 4000;
//#endif

//void PwmInit(void)
//{
//	GPIO_InitTypeDef GPIO_initStructure;
//	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
//	TIM_OCInitTypeDef TIM_OCInitStructure;
//		
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);	
//    
//	GPIO_initStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
//	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_initStructure);

//	

//	//定时器周期 t=(ARR+1)(PSC+1)/T_clock=1000*8/72000000=1/9000		T_clock为时钟频率，一般为72Mhz
//	//f=1/t=9khz
//	
//	//配置时基
//	TIM_timeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
//	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
//	TIM_timeBaseInitStructure.TIM_Period = arrValue;						//设置ARR值
//	TIM_timeBaseInitStructure.TIM_Prescaler = pscValue;						//时钟预分频值
//	TIM_TimeBaseInit(TIM1,&TIM_timeBaseInitStructure);
//	
//	//配置OC输出通道
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
//  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
//	TIM_OCInitStructure.TIM_Pulse = ccrValue;									//设置TIM1的CCR值
//	
//  TIM_OC1Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
//	
//	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
//	
//	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
//	
//	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
//	
//	TIM_ARRPreloadConfig(TIM1,ENABLE);		//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出									
//         
//	TIM_Cmd(TIM1,ENABLE);
//	
//	TIM_CtrlPWMOutputs(TIM1,ENABLE);		//开始启动定时器输出pwm,这个是高级定时器才有的，输出pwm必须打开  
//}


//void PwmOut(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4)
//{
//#if BRUSHED_FOUR_AXIS_UAV
//	TIM1->CCR1 = ThrottleLimit(pwm1,0,1000);
//	TIM1->CCR2 = ThrottleLimit(pwm2,0,1000);
//	TIM1->CCR3 = ThrottleLimit(pwm3,0,1000);
//	TIM1->CCR4 = ThrottleLimit(pwm4,0,1000);
//#elif FIXED_WING_AIRCRAFT
//	TIM1->CCR1 = ThrottleLimit(pwm1,0,1000);
//	TIM1->CCR2 = ThrottleLimit(pwm2,0,1000);
//	TIM1->CCR3 = ThrottleLimit(pwm3,0,1000);
//	TIM1->CCR4 = ThrottleLimit(pwm4,0,1000);
//#elif BRUSHLESS_FOUR_AXIS_UAV
//	TIM1->CCR1 = ThrottleLimit(pwm1,0,10000);
//	TIM1->CCR2 = ThrottleLimit(pwm2,0,10000);
//	TIM1->CCR3 = ThrottleLimit(pwm3,0,10000);
//	TIM1->CCR4 = ThrottleLimit(pwm4,0,10000);
//#endif
//}

#include "pwm.h"

static uint16_t arrValue = 0;
static uint16_t pscValue = 0;
static uint16_t ccrValue = 0;


void PwmInit(void)
{
	Tim1Init();
	Tim4Init();
	Tim2Init();
}

void Tim1Init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
    
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_initStructure);

	//定时器周期 t=(ARR+1)(PSC+1)/T_clock=1000*8/72000000=1/9000		T_clock为时钟频率，一般为72Mhz
	//f=1/t=9khz
	
	//配置时基
	TIM_timeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_timeBaseInitStructure.TIM_Period = arrValue;						//设置ARR值
	TIM_timeBaseInitStructure.TIM_Prescaler = pscValue;						//时钟预分频值
	TIM_TimeBaseInit(TIM1,&TIM_timeBaseInitStructure);
	
	//配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
	TIM_OCInitStructure.TIM_Pulse = ccrValue;									//设置TIM1的CCR值
	
  TIM_OC1Init(TIM1,&TIM_OCInitStructure);		//初始化定时器TIM1 1通道
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);                                                  
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);		//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出，占空比 = CCR/ARR。								
         
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);		//开始启动定时器输出pwm 
}

void Tim2Init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	
    
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);//改变指定管脚的映射

	//配置时基
	TIM_timeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_timeBaseInitStructure.TIM_Period = arrValue;						//设置ARR值
	TIM_timeBaseInitStructure.TIM_Prescaler = pscValue;						//时钟预分频值
	TIM_TimeBaseInit(TIM2,&TIM_timeBaseInitStructure);
	
	//配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
	TIM_OCInitStructure.TIM_Pulse = ccrValue;									//设置TIM2的CCR值
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);	//使能TIMx在CCR2上的预装载寄存器
	TIM_ARRPreloadConfig(TIM2,ENABLE);		//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出，
																				//占空比 = CCR/ARR。								   
	TIM_Cmd(TIM2,ENABLE);
}

void Tim4Init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);	
    
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_initStructure);
//	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);//改变指定管脚的映射
	
	//配置时基
	TIM_timeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_timeBaseInitStructure.TIM_Period = arrValue;						//设置ARR值
	TIM_timeBaseInitStructure.TIM_Prescaler = pscValue;						//时钟预分频值
	TIM_TimeBaseInit(TIM4,&TIM_timeBaseInitStructure);
	
	//配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
	TIM_OCInitStructure.TIM_Pulse = ccrValue;									//设置TIM4的CCR值
	
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);	
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);	//使能TIMx在CCR2上的预装载寄存器
	
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);	
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);	//使能TIMx在CCR2上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);		//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出，
																				//占空比 = CCR/ARR。								   
	TIM_Cmd(TIM4,ENABLE);
 
}

void PwmOut(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4)
{
	TIM2->CCR1 = pwm1;
	TIM1->CCR1 = pwm2;
	TIM4->CCR4 = pwm3;
	TIM4->CCR3 = pwm4;
}

void SetPwmConfig(uint16_t exArr, uint16_t exPsc, uint16_t exCcr)
{
	arrValue = exArr;
	pscValue = exPsc;
	ccrValue = exCcr;
}
