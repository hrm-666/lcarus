#include "controller.h"
#include "pid.h"
#include "imu.h"
#include "si24r1.h"
#include "pwm.h"
#include "receive_packet.h"
#include "fc_status.h"
#include "board_config.h"
#include "transmit_packet.h"
#include "bmp280.h"
#include "acc.h"
#include "gyro.h"
#include <rtthread.h>
#define DEBUG_LEVEL	DEBUG_INFO
#include "log_lib.h"
#include "barometer.h"
#include <math.h>
#include "math_lib.h"
#include "nav.h"
#include <stdlib.h>
#include "plane.h"
#include "hover_thrust_ekf.h"
#include "filter_lib.h"

#define THROTTLE_FLYING  10

//#define HOVER_THRUST_ESTIMATER	//悬停油门估计（未使用，先屏蔽）

static S_Throttle  throttle = {370, 0};		//油门结构体变量

static S_Motor motor = {0};		//最终输出的pwm值

static float dt = 0.005f;		//pid积分时间

#ifdef HOVER_THRUST_ESTIMATER
static HoverThrustEKF ekf = {0};
#endif

void AngleController(void);
void GyroController(void);
void HightControl(void);
void ControllerOutput(void);
#ifdef HOVER_THRUST_ESTIMATER
void EkfEstimateHoverThrottle(void);
#endif

void ControlInit(void)
{
	#ifdef HOVER_THRUST_ESTIMATER
	hover_thrust_ekf_init(&ekf, 0.5f, 0.01f, 0.5f, 9.81f);
	#endif
}

void ControlHandle(void)
{
	S_Plane plane = {LOCK};
	static uint32_t lastTime = 0;

	GetPlaneInfo(&plane);
	
	//根据系统时间，计算实际时间间隔
	dt = (rt_tick_get() - lastTime) * 0.001f;
	lastTime = rt_tick_get();	
	
	if(plane.lock == UNLOCK){
		#ifdef HOVER_THRUST_ESTIMATER
		EkfEstimateHoverThrottle();		//未使用，先屏蔽
		#endif
			
		//控制模式选择
		if(plane.flyMode == CONTROL_MODE_ATTITUDE){
			AngleController();  
			GyroController();
		}
		else if(plane.flyMode == CONTROL_MODE_ALTITUDE){
			AngleController();
			GyroController();
			HightControl();
		}
		
		//控制器输出
		ControllerOutput();
	}

}

//外环角度控制器
void AngleController(void)
{
	S_Imu imu = {0};
	S_Remote remote = {0};
	S_AllPid *pAllPid = NULL;
	static int8_t lastRemoteYaw = 0;
	
	GetImuInfo(&imu);
	GetRemoteInfo(&remote);
	pAllPid = pSetAllPid();

	pAllPid->rolAngle.expect = remote.roll;		//遥控器数据作为rol的期望角
	pAllPid->rolAngle.feedback = imu.roll;    //姿态解算的roll角作为反馈量
	PidPosition(&pAllPid->rolAngle, dt);        //外环pid控制器      

	pAllPid->pitAngle.expect = remote.pit;		//遥控器数据作为pit的期望角   
	pAllPid->pitAngle.feedback = imu.pitch;              //姿态解算的pitch角作为反馈量
	PidPosition(&pAllPid->pitAngle, dt);                 //外环pid控制器    
	
	if(remote.yaw == 0){
		if(lastRemoteYaw != remote.yaw){
			pAllPid->yawAngle.expect = imu.yaw;     //偏航杆从非回中->回中状态时，才更新期望角度
		}
		
		//防止由-179° -> 179°修正时，产生剧烈的自旋修正现象，计算公式调整为：误差 = （目标值 - 反馈值 + 180） % 360 - 180
		pAllPid->yawAngle.err = (int)(pAllPid->yawAngle.expect - imu.yaw + 180) % 360;
		if(pAllPid->yawAngle.err >= 0){
			pAllPid->yawAngle.err -= 180;
		} else {
			pAllPid->yawAngle.err += 180;
		}
		
		PidPositionError(&pAllPid->yawAngle, pAllPid->yawAngle.err, dt);         //外环pid控制器

		pAllPid->yawGyro.expect = pAllPid->yawAngle.out;
		
	} else {
		pAllPid->yawGyro.expect = remote.yaw*5;		//偏航进行打舵时，只进行角速度控制
	}
	
	lastRemoteYaw = remote.yaw;		//更新上一次的摇杆值
	
	throttle.finalOut = remote.throttle;		//最终油门输出来至于遥控器油门	
}

//内环角速度控制器
void GyroController(void)
{
	S_Gyro gyro = {0};
	S_AllPid *pAllPid = NULL;

	GetGyroInfo(&gyro);
	pAllPid = pSetAllPid();
	
	pAllPid->pitGyro.expect = pAllPid->pitAngle.out;         //外环输出作为内环期望值
	pAllPid->pitGyro.feedback = gyro.degS.x;             //角速度值作为反馈量 
	PidPosition(&pAllPid->pitGyro, dt);                   //内环pid控制器     
																									
	pAllPid->rolGyro.expect = pAllPid->rolAngle.out;         //外环输出作为内环期望值
	pAllPid->rolGyro.feedback = gyro.degS.y;             //角速度值作为反馈量
	PidPosition(&pAllPid->rolGyro, dt);                   //内环pid控制器 
																									
	pAllPid->yawGyro.feedback = gyro.degS.z;             //角速度值作为反馈量   
	PidPosition(&pAllPid->yawGyro, dt);                   //内环pid控制器    
	
//	LogInfo("%d, %f, %f, %f", throttle.finalOut, pAllPid->pitGyro.out, pAllPid->rolGyro.out, pAllPid->yawGyro.out);
}

//悬停油门估计
uint16_t EstimateHoverThrottle(float planeVoltage)
{
	uint16_t hoverThrottle = 0;
	
	if(planeVoltage >= 3.6f){
		hoverThrottle = 320 + 410 - (uint16_t)(planeVoltage * 100);
	}
	else if(planeVoltage > 3.4f){
		hoverThrottle = 320 + 430 - (uint16_t)(planeVoltage * 100);
	}
	else if(planeVoltage > 3.2f){
		hoverThrottle = 320 + 450 - (uint16_t)(planeVoltage * 100);
	}
	
	return hoverThrottle;
	
}

#ifdef HOVER_THRUST_ESTIMATER
void EkfEstimateHoverThrottle(void)
{
	S_Nav nav= {0};
	S_Remote remote= {0};
	float throttlePercent = 0;
	S_Plane plane = {LOCK};
	static uint8_t num = 0;
	static float moveAverageArr[10] = {0};
	
	
	GetNavAcc(&nav);
	GetRemoteInfo(&remote);
	
	if(remote.throttle < 300){
		remote.throttle = 300;
	}
	
//	throttlePercent = (float)ekf.hover_throttle;
	
//	if(fabs(nav.globalAccZ) < 0.5f){
//		nav.globalAccZ = 0;
//	}
	
//	if(ekf.hover_throttle < 0.3f || ekf.hover_throttle > 0.8f){
//		
//	}	
	
	if(remote.throttle < 300){
		remote.throttle = 300;
	}
	throttlePercent = (float)remote.throttle / 1000;
	
//	ekf.hover_throttle = ConstrainFloat(throttlePercent, 0.3f, 0.8f);
	
	hover_thrust_ekf_update(&ekf, throttlePercent, nav.globalAccZ);
	ekf.hover_throttle = MoveAverageFilter(&num, moveAverageArr, 10, ekf.hover_throttle);
//	LogRaw("%f, %d, %f\r\n", ekf.hover_throttle, remote.throttle, nav.globalAccZ);
	
	plane.voltage = ekf.hover_throttle;
	SetPlaneInfo(&plane, SET_PLANE_VOLTAGE);
}
#endif

void HightControl(void)
{
	S_Barometer baro = {0};
	static uint16_t lastThrottle = 0;
	const uint16_t middleThrottle = 500;
	S_Remote remote = {0};
	S_AllPid *pAllPid = NULL;
	S_Nav nav = {0};
	
	GetBarometerInfo(&baro);
	pAllPid = pSetAllPid();
	GetRemoteInfo(&remote);
	GetNavAcc(&nav);
	
	if(remote.throttle == middleThrottle){		//油门位于中位时才进行位置环控制
		if(lastThrottle != middleThrottle){		//油门杆从非回中状态->回中状态时，才记录当前高度
			#ifdef HOVER_THRUST_ESTIMATER
			throttle.baseValue = (uint16_t)(ekf.hover_throttle * 1000);
			#endif
			
			pAllPid->pos_high.expect = baro.relativeAltitude * 100;	
			ClearIntegral(&pAllPid->acc_high);			
		}
	
		pAllPid->pos_high.feedback = baro.relativeAltitude * 100;
		PidPosition(&pAllPid->pos_high, dt);
		
		pAllPid->vel_high.expect = pAllPid->pos_high.out;
//		pAllPid->vel_high.expect = 0;
		
	} else {
		pAllPid->vel_high.expect = (remote.throttle - middleThrottle) / 5;
//		ClearIntegral(&pAllPid->acc_high);
	}	
	
	pAllPid->vel_high.feedback = nav.velZ * 100;
	PidPosition(&pAllPid->vel_high, dt);		//垂直速度环
	
	pAllPid->acc_high.expect = pAllPid->vel_high.out;	//速度环输出作为加速度环期望
	pAllPid->acc_high.feedback = nav.globalAccZ * 100;
	PidPosition(&pAllPid->acc_high, dt);
	
	lastThrottle = remote.throttle;		//更新上一次油门值
	
	pAllPid->acc_high.out = ConstrainFloat(pAllPid->acc_high.out, -40.0, 500.0);
	
//	throttle.finalOut = throttle.baseValue + (int)pAllPid->acc_high.out;
	throttle.finalOut = throttle.baseValue;
	
//	LogInfo("%d, %f, %f, %f", throttle.finalOut, pAllPid->pos_high.expect, pAllPid->acc_high.out, baro.relativeAltitude);
}

//控制器pwm输出
void ControllerOutput(void)
{
	S_Plane plane;
	S_Imu imu = {0};
	S_Remote remote = {0};
	S_AllPid *pAllPid = NULL;

	GetPlaneInfo(&plane);
	GetImuInfo(&imu);
	GetRemoteInfo(&remote);
	pAllPid = pSetAllPid();
	
	//姿态异常上锁
	if(imu.roll <= -80 || imu.roll >= 80 || imu.pitch <= -80 || imu.pitch >= 80){
		plane.lock = LOCK;
		SetPlaneInfo(&plane, SET_PLANE_LOCK);
	}
	
	if(plane.lock == UNLOCK){		//解锁才输出            
			if(remote.throttle > THROTTLE_FLYING){		//大于起飞油门
				
				#ifdef FIXED_WING_AIRCRAFT
					motor.out1 = (throttle.finalOut + 2*pAllPid->yawGyro.out + pAllPid->pitGyro.out);  
					motor.out1 = (throttle.finalOut - 2*pAllPid->yawGyro.out + pAllPid->pitGyro.out);
					motor.out1 = (throttle.finalOut - 2*pAllPid->yawGyro.out + pAllPid->pitGyro.out);  
					motor.out1 = (throttle.finalOut + 2*pAllPid->yawGyro.out + pAllPid->pitGyro.out);
				#else 
					motor.out1 = throttle.finalOut + pAllPid->pitGyro.out - pAllPid->rolGyro.out - pAllPid->yawGyro.out;
					motor.out2 = throttle.finalOut + pAllPid->pitGyro.out + pAllPid->rolGyro.out + pAllPid->yawGyro.out;   
					motor.out3 = throttle.finalOut - pAllPid->pitGyro.out + pAllPid->rolGyro.out - pAllPid->yawGyro.out;
					motor.out4 = throttle.finalOut - pAllPid->pitGyro.out - pAllPid->rolGyro.out + pAllPid->yawGyro.out; 
				#endif
			} else {		//小于起飞油门
					motor.out1 = remote.throttle;  
					motor.out2 = remote.throttle;
					motor.out3 = remote.throttle;
					motor.out4 = remote.throttle;
					
					ClearIntegral(&pAllPid->pitAngle);         //清除积分
					ClearIntegral(&pAllPid->pitGyro);          //清除积分        
					ClearIntegral(&pAllPid->rolAngle);         //清除积分    
					ClearIntegral(&pAllPid->rolGyro);          //清除积分
					ClearIntegral(&pAllPid->yawAngle);         //清除积分
					ClearIntegral(&pAllPid->yawGyro);          //清除积分
			}          
	} else {		//未解锁
		motor.out1 = 0;
		motor.out2 = 0;
		motor.out3 = 0;
		motor.out4 = 0;

		ClearIntegral(&pAllPid->pitAngle);	//清除积分
		ClearIntegral(&pAllPid->pitGyro);		//清除积分        
		ClearIntegral(&pAllPid->rolAngle);	//清除积分    
		ClearIntegral(&pAllPid->rolGyro);		//清除积分
		ClearIntegral(&pAllPid->yawAngle);	//清除积分
		ClearIntegral(&pAllPid->yawGyro);		//清除积分        

		ClearIntegral(&pAllPid->pos_high);	
		ClearIntegral(&pAllPid->vel_high);
		ClearIntegral(&pAllPid->acc_high);
		
	}
	
#ifdef BRUSHLESS_FOUR_AXIS_UAV
		motor.out1 += 1000;
		motor.out2 += 1000;
		motor.out3 += 1000;
		motor.out4 += 1000;
#endif

//	TIM_SetCompare1(TIM4,100);
#ifdef BRUSHLESS_FOUR_AXIS_UAV
	motor.out1 = ThrottleLimit(motor.out1, 1000, 2000);
	motor.out2 = ThrottleLimit(motor.out2, 1000, 2000);
	motor.out3 = ThrottleLimit(motor.out3, 1000, 2000);
	motor.out4 = ThrottleLimit(motor.out4, 1000, 2000);
#else
	motor.out1 = ThrottleLimit(motor.out1, 0, 1000);
	motor.out2 = ThrottleLimit(motor.out2, 0, 1000);
	motor.out3 = ThrottleLimit(motor.out3, 0, 1000);
	motor.out4 = ThrottleLimit(motor.out4, 0, 1000);
#endif

	PwmOut(motor.out1, motor.out2, motor.out3, motor.out4);
}


