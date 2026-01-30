#include "battery_detect.h"
#include "transmit_packet.h"
#include "adc.h"
#include "plane.h"

//电池电压监测
void VoltageDetect(void)
{
	static float lastVoltage = 0;
	float nowVoltage = 0;
	S_Plane plane;
	uint16_t adcValue = 0;
	
	GetPlaneInfo(&plane);
	adcValue = GetAdcValue();
	
	plane.voltage = 2.0f * (adcValue * 3.3f/4096.0f);
	
	//数据消抖，便于显示
	nowVoltage = lastVoltage - plane.voltage;		
	if(nowVoltage >= -0.05f && nowVoltage <= 0.05f){
		plane.voltage = lastVoltage;
	} else {
		lastVoltage = plane.voltage;
	}
	SetPlaneInfo(&plane, SET_PLANE_VOLTAGE);

	//电压低于3v，则设置标志位
	if(plane.voltage <= 3.3f){
		plane.power = POWER_LOWER;
	}
	else{
		plane.power = POWER_NORMAL;
	}
	SetPlaneInfo(&plane, SET_PLANE_POWER);
}    
