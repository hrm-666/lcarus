#ifndef _nav_h_
#define _nav_h_

#include "stm32f10x.h"

#define accmax_1g      4096
#define gravity_mss    9.80665f                    // acceleration due to gravity in m/s/s

#define acc_to_1g      gravity_mss / accmax_1g
#define one_g_to_acc   accmax_1g / gravity_mss

typedef struct{
	float globalAccX;		//导航系下东西方向的加速度
	float globalAccY;		//导航系下南北方向的加速度
	float globalAccZ;		//导航系下垂直方向的加速度
	
	float velZ;		//导航系下垂直方向的速度
	float positionZ;		//导航系下垂直方向的位置
}S_Nav;

void NavAcc(void);
uint8_t NavAccInit(void);
void GetNavAcc(S_Nav *exNav);
#endif

