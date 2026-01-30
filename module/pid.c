#include "pid.h"
#include "flash.h"
#include "board_config.h"
#include "my_lib.h"
#include <string.h>

#define PID_STORE_ARR_LEN	30

static uint32_t pidResetFlag = 0;

static S_AllPid allPid = {0};
void PidInit(S_PidPosition *controller,uint8_t label);

//pid参数初始化
void AllPidInit(void)
{
	uint32_t res;
	
	PidInit(&allPid.pitAngle,0);
	PidInit(&allPid.rolAngle,1);
	PidInit(&allPid.yawAngle,2);
	
	PidInit(&allPid.pitGyro,3);
	PidInit(&allPid.rolGyro,4);
	PidInit(&allPid.yawGyro,5);

	PidInit(&allPid.acc_high,6);
	PidInit(&allPid.vel_high,7);
	PidInit(&allPid.pos_high,8);
	
	res = *(uint32_t *)(PID_WRITE_ADDRESS);
	if(res == 0xAE && pidResetFlag == 0){		//判断帧头及复位标志位
		PidDataReadFromFlash(PID_WRITE_ADDRESS,&allPid);		//pid数据从flash读出
	}else{
		PidDataWriteToFlash(PID_WRITE_ADDRESS,&allPid);		//pid数据写入flash
	}
	
} 

//存储pid控制器参数
const float pidIintData[9][5] =
{
    //0.kp 1.ki 2.kd 3.积分限幅  4.pid输出限幅值  
		#ifdef BRUSHED_FOUR_AXIS_UAV
    //姿态外环参数  
    {2.5,  0,   0,  300, 	800},          //俯仰角度值
    {2.5,  0,   0,  300,	800},          //横滚角度值
    {2.0,  0,   0,  300, 	800},          //偏航角度值
    
    //姿态内环参数
    {0.60,   0.08,  1.5,  300, 	800},    //俯仰角速度值
    {0.60,   0.08,  1.5,  300, 	800},    //横贯角速度值
    {1.30,   0.03,  1.0,  300, 	800},    //偏航角速度值
		#elif defined FIXED_WING_AIRCRAFT
    //姿态外环参数  
    {2.5,  0,   0,  300, 	800},          //俯仰角度值
    {2.5,  0,   0,  300,	800},          //横滚角度值
    {1.0,  0,   0,  300, 	800},          //偏航角度值
    
    //姿态内环参数
    {1.50,   0.0015,  0.010,  300, 	800},    //俯仰角速度值
    {0.60,   0.0015,  0.010,  300, 	800},    //横贯角速度值
    {1.35,   0.0020,  0.155,  300, 	800},    //偏航角速度值
		#elif defined BRUSHLESS_FOUR_AXIS_UAV
    //姿态外环参数  
    {2.5,  0,   0,  300, 	800},          //俯仰角度值
    {2.5,  0,   0,  300,	800},          //横滚角度值
    {2.0,  0,   0,  300, 	800},          //偏航角度值
    
    //姿态内环参数
    {0.60,   0.0015,  0.010,  300, 	800},    //俯仰角速度值
    {0.60,   0.0015,  0.010,  300, 	800},    //横贯角速度值
    {1.35,   0.0020,  0.155,  300, 	800},    //偏航角速度值
		#endif
		
		//竖直定高参数   
	{1.2,   0.05,	0.05,		185,	500},		//acc_high
    {0.5,   0,		0,	 		300,	500},		//vel_high
    {1,   	0,		0,	 		300,	500},		//pos_high
          
};

//pid参数初始化配置
void PidInit(S_PidPosition *controller,uint8_t label)
{
	controller->kp  = pidIintData[label][0];
	controller->ki  = pidIintData[label][1];
	controller->kd  = pidIintData[label][2];
	controller->integral_max = pidIintData[label][3];
	controller->out_max = pidIintData[label][4];      
}

//位置式pid控制器
float PidPosition(S_PidPosition *controller, float dt)
{
    controller->err_last = controller->err;		//更新上一次误差
    controller->err = controller->expect - controller->feedback;    //计算当前误差
    controller->integral += controller->ki * controller->err * dt;		//误差积分  

    //积分限幅
    if(controller->integral >  controller->integral_max){
			controller->integral =  controller->integral_max;
		}
    if(controller->integral < -controller->integral_max){
			controller->integral = -controller->integral_max;
		}
 
    //pid运算
    controller->out =  controller->kp * controller->err
                     + controller->integral
                     + controller->kd * (controller->err - controller->err_last);
    //输出限幅
    if(controller->out >  controller->out_max){
      controller->out =  controller->out_max;
    }
    else if(controller->out < -controller->out_max){
      controller->out = -controller->out_max;
    }

    return  controller->out;
}

//位置式pid控制器，和PidPosition()的区别在于参数的数量的不同
float PidPositionError(S_PidPosition *controller, float error, float dt)
{    
    controller->integral += controller->ki * controller->err * dt;		//误差积分

    //积分限幅
    if(controller->integral >  controller->integral_max){
			controller->integral =  controller->integral_max;
		}
    if(controller->integral < -controller->integral_max){
			controller->integral = -controller->integral_max;
		}
 
    //pid运算
    controller->out =  controller->kp * controller->err
                     + controller->integral
                     + controller->kd * (controller->err - controller->err_last);
    //输出限幅
    if(controller->out >  controller->out_max){
      controller->out =  controller->out_max;
    }
    else if(controller->out < -controller->out_max){
      controller->out = -controller->out_max;
    }

	controller->err_last = controller->err;		//更新上一次误差
	
    return  controller->out;
}

//增量式pid
float PidIncremental(S_PidIncremental *controller, float dt)
{
    controller ->error2 = controller -> error1;	//更新上上次误差
    controller -> error1 = controller -> error0;	//更新上次误差
    controller->error0 = controller->expect - controller->feedback;    //计算当前误差
 
    //pid运算
    controller->out +=  controller->kp * (controller->error0 - controller->error1)
                     + controller->ki * controller->error0
                     + controller->kd * (controller->error0 - 2 * controller->error1 + controller ->error2);
    //输出限幅
    if(controller->out >  controller->out_max){
      controller->out =  controller->out_max;
    }
    else if(controller->out < -controller->out_max){
      controller->out = -controller->out_max;
    }

    return  controller->out;
}

//清除积分
void ClearIntegral(S_PidPosition *controller)
{
    controller->integral = 0.0f;
}

//往flash中写入pid数据
void PidDataWriteToFlash(uint32_t pidWriteAddr, S_AllPid *pidArr)
{
	uint8_t arrLen = 0;
	uint32_t arrTemp[PID_STORE_ARR_LEN] = {0};

	arrTemp[arrLen++] = 0xAE;		//添加帧头，判断是否有数据
	
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->rolAngle.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->rolAngle.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->rolAngle.kd;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pitAngle.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pitAngle.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pitAngle.kd;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->yawAngle.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->yawAngle.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->yawAngle.kd;

	arrTemp[arrLen++] = *(uint32_t *)&pidArr->rolGyro.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->rolGyro.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->rolGyro.kd;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pitGyro.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pitGyro.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pitGyro.kd;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->yawGyro.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->yawGyro.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->yawGyro.kd;
	
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->acc_high.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->acc_high.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->acc_high.kd;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->vel_high.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->vel_high.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->vel_high.kd;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pos_high.kp;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pos_high.ki;
	arrTemp[arrLen++] = *(uint32_t *)&pidArr->pos_high.kd;
	
  FlashWriteSpecifyData(pidWriteAddr,arrTemp,arrLen);
}

//从flash中读取pid数据
void PidDataReadFromFlash(uint32_t PidReadAddr,S_AllPid *pidArr)
{
	uint32_t arrTemp[PID_STORE_ARR_LEN] = {0};
	uint8_t arrLen = 0;

	STM32_FLASH_Read(PidReadAddr, arrTemp, PID_STORE_ARR_LEN);
	
	arrLen++;  //除去帧头

	pidArr->rolAngle.kp = *(float *)&arrTemp[arrLen++];
	pidArr->rolAngle.ki = *(float *)&arrTemp[arrLen++];
	pidArr->rolAngle.kd = *(float *)&arrTemp[arrLen++];
	pidArr->pitAngle.kp = *(float *)&arrTemp[arrLen++];
	pidArr->pitAngle.ki = *(float *)&arrTemp[arrLen++];
	pidArr->pitAngle.kd = *(float *)&arrTemp[arrLen++];
	pidArr->yawAngle.kp = *(float *)&arrTemp[arrLen++];
	pidArr->yawAngle.ki = *(float *)&arrTemp[arrLen++];
	pidArr->yawAngle.kd = *(float *)&arrTemp[arrLen++];
	
	pidArr->rolGyro.kp = *(float *)&arrTemp[arrLen++];
	pidArr->rolGyro.ki = *(float *)&arrTemp[arrLen++];
	pidArr->rolGyro.kd = *(float *)&arrTemp[arrLen++];
	pidArr->pitGyro.kp = *(float *)&arrTemp[arrLen++];
	pidArr->pitGyro.ki = *(float *)&arrTemp[arrLen++];
	pidArr->pitGyro.kd = *(float *)&arrTemp[arrLen++];
	pidArr->yawGyro.kp = *(float *)&arrTemp[arrLen++];
	pidArr->yawGyro.ki = *(float *)&arrTemp[arrLen++];
	pidArr->yawGyro.kd = *(float *)&arrTemp[arrLen++];
	
	pidArr->acc_high.kp = *(float *)&arrTemp[arrLen++];
	pidArr->acc_high.ki = *(float *)&arrTemp[arrLen++];
	pidArr->acc_high.kd = *(float *)&arrTemp[arrLen++];
	pidArr->vel_high.kp = *(float *)&arrTemp[arrLen++];
	pidArr->vel_high.ki = *(float *)&arrTemp[arrLen++];
	pidArr->vel_high.kd = *(float *)&arrTemp[arrLen++];
	pidArr->pos_high.kp = *(float *)&arrTemp[arrLen++];
	pidArr->pos_high.ki = *(float *)&arrTemp[arrLen++];
	pidArr->pos_high.kd = *(float *)&arrTemp[arrLen++];
	
}

void GetAllPidInfo(S_AllPid *exAllPid)
{
  StructCopy((uint8_t *)&allPid, (uint8_t *)exAllPid, sizeof(allPid));
}

const S_AllPid *pGetAllPid(void)
{
  return (const S_AllPid *)&allPid;
}

//设置plane数据
void SetAllPidInfo(const S_AllPid *exAllPid)
{
	StructCopy((uint8_t *)exAllPid, (uint8_t *)&allPid, sizeof(allPid));
}

S_AllPid *pSetAllPid(void)
{
  return &allPid;
}

void SetPidResetFlag(void)
{
  pidResetFlag = 1;
}
