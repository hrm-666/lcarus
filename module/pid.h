#ifndef _pid_h_
#define _pid_h_

#include "stm32f10x.h"

#define PID_WRITE_ADDRESS  0x08000000 + 1024*61		//页地址

typedef struct
{
    float err;
    float err_last;

    float expect;
    float feedback;

    float kp;
    float ki;
    float kd;

    float integral;
    float integral_max;

    float out;
    float out_max;
}S_PidPosition;		//位置式pid结构体

typedef struct
{
    float error0;		//当前误差
    float error1;		//上一次误差
	float error2;		//上上次误差

    float expect;		//期望值（目标值）
    float feedback;		//反馈值（测量值）

    float kp;
    float ki;
    float kd;

    float integral;
    float integral_max;

    float out;
    float out_max;
}S_PidIncremental;		//增量式pid结构体

typedef struct
{
	//姿态外环（角度环）
	S_PidPosition pitAngle;
	S_PidPosition rolAngle;
	S_PidPosition yawAngle;

	//姿态内环（角速度环）
	S_PidPosition pitGyro;      
	S_PidPosition rolGyro;
	S_PidPosition yawGyro;

	//竖直定高
	S_PidPosition acc_high;		//竖直加速度环
	S_PidPosition vel_high;		//竖直速度环
	S_PidPosition pos_high;		//竖直位置环
}S_AllPid;

float PidPosition(S_PidPosition *controller, float dt);
void AllPidInit(void);
void ClearIntegral(S_PidPosition *controller);
void GetAllPidInfo(S_AllPid *exAllPid);
void SetAllPidInfo(const S_AllPid *exAllPid);
void SetPidResetFlag(void);
const S_AllPid *pGetAllPid(void);
S_AllPid *pSetAllPid(void);
void PidDataReadFromFlash(uint32_t PidReadAddr,S_AllPid *pidArr);
void PidDataWriteToFlash(uint32_t pidWriteAddr, S_AllPid *pidArr);
float PidPositionError(S_PidPosition *controller, float error, float dt);
float PidIncremental(S_PidIncremental *controller, float dt);
#endif
