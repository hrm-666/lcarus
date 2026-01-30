#include "adc.h"

static uint16_t adc_value;

/* 端口配置初始化 */
void AdcInitPort(void)
{
	GPIO_InitTypeDef GPIO_initStructure;    
	RCC_APB2PeriphClockCmd(ADC_RCC_PORT, ENABLE);
	
	GPIO_initStructure.GPIO_Pin = ADC_PIN;	    
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ADC_PORT, &GPIO_initStructure);	
}

/* adc配置 */ 
void AdcConfig(void)
{
  ADC_InitTypeDef ADC_initStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	ADC_initStructure.ADC_ContinuousConvMode = ENABLE;		//连续转换	
	ADC_initStructure.ADC_DataAlign = ADC_DataAlign_Right;		//数据右对齐				
	ADC_initStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;		//软件触发 
	ADC_initStructure.ADC_Mode = ADC_Mode_Independent;							
	ADC_initStructure.ADC_NbrOfChannel = 1;										
	ADC_initStructure.ADC_ScanConvMode = DISABLE;		//非扫描模式						
	ADC_Init(ADC_NUM,&ADC_initStructure);

	ADC_Cmd(ADC_NUM,ENABLE);
    
	ADC_DMACmd(ADC_NUM,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);		//ADC时钟分频											
                                                                                
	ADC_RegularChannelConfig(ADC_NUM, ADC_Channel_1, NUM, ADC_SampleTime_239Cycles5);		//通道配置，采样时间设置
	
	ADC_ResetCalibration(ADC_NUM);		//复位校准				
	while(ADC_GetCalibrationStatus(ADC_NUM));		//等待
	ADC_StartCalibration(ADC_NUM);		//启动校准
	while(ADC_GetCalibrationStatus(ADC_NUM));		//等待校准完成

	ADC_SoftwareStartConvCmd(ADC_NUM,ENABLE);		//开启转换                                     
}

void DmaConfig(uint32_t *memoryAddr)
{
	DMA_InitTypeDef DMA_initStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_initStructure.DMA_BufferSize = 1;										
	DMA_initStructure.DMA_DIR = DMA_DIR_PeripheralSRC;		//传输方向：外设->内存						
	DMA_initStructure.DMA_M2M = DMA_M2M_Disable;								
	DMA_initStructure.DMA_MemoryBaseAddr = (u32)memoryAddr;				        
	DMA_initStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		//DMA传输的内存数据大小：半字为单位			
	DMA_initStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址自增
	DMA_initStructure.DMA_Mode = DMA_Mode_Circular;								
	DMA_initStructure.DMA_PeripheralBaseAddr = ((u32)&ADC_NUM->DR);	            //外设地址			
	DMA_initStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//DMA传输的外设数据大小：半字为单位
	DMA_initStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不变
	DMA_initStructure.DMA_Priority = DMA_Priority_Medium;						
	DMA_Init(DMA1_Channel1,&DMA_initStructure);
    
	DMA_Cmd(DMA1_Channel1,ENABLE);	
}


void AdcInit(void)
{
    AdcInitPort();
    AdcConfig();
    DmaConfig((uint32_t *)&adc_value);
}

uint16_t GetAdcValue(void)
{
	return adc_value;
}
