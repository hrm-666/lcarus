#include "fc_status.h"
#include "receive_packet.h"
#include "board_config.h"
#include "transmit_packet.h"
#include "log_lib.h"
#include "plane.h"

#define UNLOCK_COUNT		60		//解锁计数阈值
#define LOCK_COUNT			60		//锁定计数阈值
#define AUTO_LOCK_COUNT		200		//自动锁定计数阈值

//检查锁定状态
void PlaneLockStatus(void)
{
	static uint16_t autoLockCount = 0;
	static uint16_t unlockCount = 0;
	S_Plane plane;
	S_Remote remote = {0};
	static uint16_t lockCount = 0;
	
	GetPlaneInfo(&plane);
	GetRemoteInfo(&remote);
	
	//解锁
	if(plane.lock == LOCK || plane.lock == UNLOCKING){
		if(remote.throttle <= 20 && remote.yaw <= -25 && remote.pit >= 25 && remote.roll <= -25){
			unlockCount++;
			plane.lock = UNLOCKING;
			if(unlockCount > UNLOCK_COUNT){
				unlockCount = 0;
				plane.lock = UNLOCK;		//解锁标志置位，指示灯使用该标志 
			}
		} else {
			plane.lock = LOCK;
			unlockCount = 0;
		}
	}
	else if(plane.lock == UNLOCK || plane.lock == LOCKING){//上锁

		if(remote.throttle <= 20 && remote.yaw >= 25 && remote.pit >= 25 && remote.roll >= 25){
			lockCount++;
			plane.lock = LOCKING;
			if(lockCount > LOCK_COUNT){
				lockCount = 0;
				plane.lock = LOCK;		//上锁标志置位，指示灯使用该标志
			}
		}
		else if(remote.throttle <= 20){
			autoLockCount++;
			if(autoLockCount > AUTO_LOCK_COUNT){
				autoLockCount = 0;
				plane.lock = LOCK;		//上锁标志置位，指示灯使用该标志
			}
		}
		else{
			plane.lock = UNLOCK;
			autoLockCount = 0;		//油门大于10要清空计数
			lockCount = 0;
		}
	}

	SetPlaneInfo(&plane, SET_PLANE_LOCK);
}

uint32_t  chip_id[3] = {0};  

//读取芯片ID
void get_chip_id(void)
{
    chip_id[0] = *(__IO u32 *)(0X1FFFF7F0); // 高字节
    chip_id[1] = *(__IO u32 *)(0X1FFFF7EC); //
    chip_id[2] = *(__IO u32 *)(0X1FFFF7E8); // 低字节
}

