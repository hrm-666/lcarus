#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include "stm32f10x.h"

//时钟频率配置
#define SYSCLK_FREQ_72MHz  72000000
//#define SYSCLK_FREQ_104MHz  104000000		//对于stm32f1来说，这个频率已经超频，会缩短使用寿命

//机型选择，同时只能有一个为1
#define BRUSHED_FOUR_AXIS_UAV		四轴空心杯无人机
//FIXED_WING_AIRCRAFT		固定翼手抛机
//#define BRUSHLESS_FOUR_AXIS_UAV		无刷四轴无人机
//#define BRUSHED_FOUR_AXIS_UAV 	1//


//姿态解算方式
#define IMU_QUATERNION  1		//四元数姿态解算
//#define IMU_DMP		1		//6050自带dmp姿态解算


#ifdef BRUSHED_FOUR_AXIS_UAV

#elif defined FIXED_WING_AIRCRAFT

#elif defined BRUSHLESS_FOUR_AXIS_UAV

#endif

void BoardConfig(void);

#endif
