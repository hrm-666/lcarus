#include "ano_communicate.h"
#include "usart.h"
#include "imu.h"
#include "receive_packet.h"
#include "pid.h"
#include "systick.h"
#include "led.h"
#include "flash.h"
#include "acc.h"
#include "acc_cal.h"
#include "transmit_packet.h"
#include "gyro.h"
#include <rtthread.h>
#include "log_lib.h"
#include "queue_lib.h"
#include "plane.h"
#include "compass.h"
#include "barometer.h"

/* 将大于一个字节的数据拆分成多个字节发送 */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define RECEIVEBUFF_LENGTH	128		//串口接收数据缓冲长度

static rt_thread_t ano_communicate_thread = RT_NULL;

static uint8_t usartReceiveBuff[RECEIVEBUFF_LENGTH] = {0};		//串口接收缓冲区
static S_Queue usartMessageQueue = {0};		//串口接收队列

static void AnoDataSend(u16 sendBuff[],uint8_t funcByte,uint8_t dataLen);
static void DateTransfer(void);
static void Usart2Send(const uint8_t *data,uint8_t len);
static void ANODataReceiveAnalysis(uint8_t *dataBuffer,uint8_t num);
static void ANODTDataReceivePrepare(uint8_t data);
static void AnoReceiveData(void);

void ano_communicate_thread_entry(void *param)
{
	while(1){
		
		if(QueueLength(&usartMessageQueue) == 0){		//当有数据过来时，停止发送数据，防止响应超时
			ANOSendData();	//向匿名地面站发送姿态数据
		}
		
		AnoReceiveData();
		
		rt_thread_mdelay(10);
	}
}

void AnoCommunicate_Main(void)
{	
	QueueInit(&usartMessageQueue, usartReceiveBuff, RECEIVEBUFF_LENGTH);
	
	//创建线程
	ano_communicate_thread = rt_thread_create("ano_communicate_thread", ano_communicate_thread_entry, RT_NULL, 512, 10, 5);
	
	//启动线程
	if (ano_communicate_thread != RT_NULL){
		rt_thread_startup(ano_communicate_thread);
	} else {
		LogError("ano_communicate_thread start fail");
		return;
	}

}

/* 向匿名上位机发送STATUS帧，详见匿名上位机通信协议表 */
void ANOSendStatus(void)
{
	S_Plane plane;
	S_Imu imu = {0};
	uint16_t sendBuf[10] = {0};
	uint8_t buffLen = 0;
	S_Remote remote = {0};
	S_Barometer baro = {0};
	
	GetPlaneInfo(&plane);
	GetImuInfo(&imu);
	GetRemoteInfo(&remote);
	GetBarometerInfo(&baro);

	sendBuf[buffLen++] = (int16_t)(imu.roll*100);
	sendBuf[buffLen++] = (int16_t)(imu.pitch*100);
	sendBuf[buffLen++] = (int16_t)(imu.yaw*100);
	
	//高度
	sendBuf[buffLen++] = (int16_t)((int)(baro.height * 100) >> 16);		//海拔高度
	sendBuf[buffLen++] = (int16_t)((int)baro.height * 100);	
	
	sendBuf[buffLen++] = remote.flyMode | plane.lock;		//飞行模式和锁定状态
	
	AnoDataSend(sendBuf, 0x01, buffLen);
}

void ANOSendSenser(void)
{
	uint16_t sendBuf[15] = {0};
	uint8_t buffLen = 0;
	S_Acc acc = {0};
	S_Gyro gyro = {0};
	S_Compass compass = {0};

	GetAccInfo(&acc);
	GetGyroInfo(&gyro);
	GetCompassInfo(&compass);
	
	//加速度原始数据
	sendBuf[buffLen++] = acc.rawData.x;
	sendBuf[buffLen++] = acc.rawData.y;
	sendBuf[buffLen++] = acc.rawData.z;
	
	//陀螺仪原始数据
	sendBuf[buffLen++] = gyro.rawData.x;
	sendBuf[buffLen++] = gyro.rawData.y;
	sendBuf[buffLen++] = gyro.rawData.z;
	
	//地磁原始数据，没有磁力计，所以置成0
	sendBuf[buffLen++] = compass.rawX;
	sendBuf[buffLen++] = compass.rawY;
	sendBuf[buffLen++] = compass.rawZ;
	
	AnoDataSend(sendBuf, 0x02, buffLen);
}

/* 遥控器通道数据 */
void ANOSendRCData(void)
{
	S_Remote remote = {0};
	uint16_t sendBuf[15] = {0};
	uint8_t buffLen = 0;
	
	GetRemoteInfo(&remote);
	
	sendBuf[buffLen++] = (int16_t)(remote.throttle + 1000);
	sendBuf[buffLen++] = (int16_t)(remote.yaw + 1500);
	sendBuf[buffLen++] = (int16_t)(remote.roll + 1500);
	sendBuf[buffLen++] = (int16_t)(remote.pit + 1500);
	
	sendBuf[buffLen++] = 0;
	sendBuf[buffLen++] = 0;
	sendBuf[buffLen++] = 0;
	sendBuf[buffLen++] = 0;
	sendBuf[buffLen++] = 0;
	sendBuf[buffLen++] = 0;
	
	AnoDataSend(sendBuf, 0x03, buffLen);
}


void ANOSendData(void)
{
	static uint8_t sendDataCount = 0;
	
	sendDataCount++;
	
	if(sendDataCount == 1){
		ANOSendStatus();
	}
	else if(sendDataCount == 2){
		ANOSendSenser();
	}
	else if(sendDataCount == 3){
		ANOSendRCData();
	}
	else if(sendDataCount == 4){
		sendDataCount = 0;
	}
	
}

//pid数据发送到匿名上位机
static void AnoDataSend(u16 *sendBuff,uint8_t funcByte,uint8_t dataLen)
{
	uint8_t count = 0;
	vs16 temp;
	uint8_t sum = 0;
	uint8_t i;
	uint8_t dataToSend[50];
	
	dataToSend[count++] = 0xAA;		//帧头：AAAA
	dataToSend[count++] = 0xAA;		//
	dataToSend[count++] = funcByte;		//功能字：0xFn只接受数据，不显示图像；0x0n显示数据和图像；0x01表示发送的是STATUS
	dataToSend[count++] = 0;			//需要发送数据的字节数，暂给0

	for(i = 0; i < dataLen; i++){//数据存入待发送数组
		temp = (int)(sendBuff[i]);    
	  dataToSend[count++] = BYTE1(temp);
	  dataToSend[count++] = BYTE0(temp);
	}
	
	dataToSend[3] = count-4;		//补充字节数
	
	sum = 0;
	for(i = 0; i < count; i++){		//计算校验位
		sum += dataToSend[i];
	}
	dataToSend[count++] = sum;		//帧尾补上校验位
  
	if(funcByte >= 0x10 && funcByte <= 0x12){
		Usart2Send(dataToSend, count);		//串口发送
	} else{
		Usart2DMASendData(dataToSend, count);
	}
	
}

//pid数据整理，便于发送
static void DateTransfer(void)
{
	uint8_t arrLen = 0;
	u16 sendBuff[10] = {0};
	const S_AllPid *pAllPid = NULL;

	pAllPid = pGetAllPid();

	sendBuff[arrLen++] = pAllPid->rolGyro.kp*100;
	sendBuff[arrLen++] = pAllPid->rolGyro.ki*100;
	sendBuff[arrLen++] = pAllPid->rolGyro.kd*100;
	sendBuff[arrLen++] = pAllPid->pitGyro.kp*100;
	sendBuff[arrLen++] = pAllPid->pitGyro.ki*100;
	sendBuff[arrLen++] = pAllPid->pitGyro.kd*100;
	sendBuff[arrLen++] = pAllPid->yawGyro.kp*100;
	sendBuff[arrLen++] = pAllPid->yawGyro.ki*100;
	sendBuff[arrLen++] = pAllPid->yawGyro.kd*100;
	AnoDataSend(sendBuff,0x10,arrLen);
	delay_ms(2);//延时，等待数据发送完
	
	arrLen = 0;
	sendBuff[arrLen++] = pAllPid->rolAngle.kp*100;
	sendBuff[arrLen++] = pAllPid->rolAngle.ki*100;
	sendBuff[arrLen++] = pAllPid->rolAngle.kd*100;
	sendBuff[arrLen++] = pAllPid->pitAngle.kp*100;
	sendBuff[arrLen++] = pAllPid->pitAngle.ki*100;
	sendBuff[arrLen++] = pAllPid->pitAngle.kd*100;
	sendBuff[arrLen++] = pAllPid->yawAngle.kp*100;
	sendBuff[arrLen++] = pAllPid->yawAngle.ki*100;
	sendBuff[arrLen++] = pAllPid->yawAngle.kd*100;
	AnoDataSend(sendBuff,0x11,arrLen);
	delay_ms(2);//延时，等待数据发送完
	
	arrLen = 0;
	sendBuff[arrLen++] = pAllPid->vel_high.kp*100;
	sendBuff[arrLen++] = pAllPid->vel_high.ki*100;
	sendBuff[arrLen++] = pAllPid->vel_high.kd*100;
	sendBuff[arrLen++] = pAllPid->pos_high.kp*100;
	sendBuff[arrLen++] = pAllPid->pos_high.ki*100;
	sendBuff[arrLen++] = pAllPid->pos_high.kd*100;
	//一组数据由3组pid组成，多余部分用0补充，不然会数据错误
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = 0;
	AnoDataSend(sendBuff, 0x12, arrLen);
	delay_ms(2);//延时，等待数据发送完
	
	arrLen = 0;
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = pAllPid->acc_high.kp*100;
	sendBuff[arrLen++] = pAllPid->acc_high.ki*100;
	sendBuff[arrLen++] = pAllPid->acc_high.kd*100;
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = 0;
	sendBuff[arrLen++] = 0;
	AnoDataSend(sendBuff, 0x13, arrLen);
}

static void Usart2Send(const uint8_t *data,uint8_t len)
{
  uint8_t i;
	for(i=0;i<len;i++){
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET){		//等待发送完成
			
		}        
			USART2->DR = *(data+i);		//向串口2发送数据
		//USART_SendData(USART1,data);
	}
	USART_ClearFlag(USART2,USART_FLAG_TC);		//清空标志位
}

//摘自匿名地面站
//返回校验数据帧
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	uint8_t dataToSend[10];
	dataToSend[0]=0xAA;
	dataToSend[1]=0xAA;
	dataToSend[2]=0xEF;
	dataToSend[3]=2;
	dataToSend[4]=head;
	dataToSend[5]=check_sum;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += dataToSend[i];
	dataToSend[6]=sum;

	Usart2Send(dataToSend, 7);
}


void ANOSendMSG(void)
{
	uint8_t msgId = 1;
	uint8_t msgData = 0x01;
	
	uint8_t cnt = 0;
	vs16 temp;
	uint8_t sum = 0;
	uint8_t i;
	uint8_t dataToSend[10];
	
	dataToSend[cnt++] = 0xAA;		//帧头：AAAA
	dataToSend[cnt++] = 0xAA;		//
	dataToSend[cnt++] = 0xEE;		//功能字：0xFn只接受数据，不显示图像；0x0n显示数据和图像；0x01表示发送的是STATUS
	dataToSend[cnt++] = 0;			//需要发送数据的字节数，暂给0

  
	dataToSend[cnt++] = msgId;
	dataToSend[cnt++] = msgData;

	
	dataToSend[3] = cnt-4;		//补充字节数
	
	sum = 0;
	for(i = 0; i < cnt; i++){		//计算校验位
		sum += dataToSend[i];
	}
	dataToSend[cnt++] = sum;		//帧尾补上校验位
  
	Usart2Send(dataToSend,cnt);		//串口发送
	
}



//摘自匿名地面站
//解析数据帧，调用对应功能
static void ANODataReceiveAnalysis(uint8_t *dataBuffer,uint8_t num)
{
	uint8_t sum = 0;
	S_AllPid *pAllPid = NULL;

	for(uint8_t i=0;i<(num-1);i++){
		sum += *(dataBuffer+i);
	}
	
	if(!(sum==*(dataBuffer+num-1))){		//判断sum
		return;		
	}
	
	if(!(*(dataBuffer)==0xAA && *(dataBuffer+1)==0xAF)){		//判断帧头
		return;		
	}
	
	if(*(dataBuffer+2)==0X01)		//命令集合1
	{
		if(*(dataBuffer+4)==0X01){		//ACC校准
//			acc.Acc_CALIBRATE = 1;
//			ANOSendMSG();
		}
//		if(*(dataBuffer+4)==0X02)		//GYRO校准
//			//acc.Gyro_CALIBRATE = 1;
//		if(*(dataBuffer+4)==0X03)		//
//		{
//			//acc.Acc_CALIBRATE = 1;		
//			//acc.Gyro_CALIBRATE = 1;			
//		}
//		if(*(dataBuffer+4)==0X20){		//ACC校准
////			acc.Acc_CALIBRATE = 1;

//		}
//		if(*(dataBuffer+4)==0X21){		//加速度校准第一步
//			calStep = UP;
//			AccCalibrate(calStep);
//		}
//		if(*(dataBuffer+4)==0X22){		//加速度校准第二步
//			calStep = DOWN;
//			AccCalibrate(calStep);
//		}
//		if(*(dataBuffer+4)==0X23){		//加速度校准第三步
//			calStep = FORWARD;
//			AccCalibrate(calStep);
//		}
//		if(*(dataBuffer+4)==0X24){		//加速度校准第四步
//			calStep = BACK;
//			AccCalibrate(calStep);
//		}
//		if(*(dataBuffer+4)==0X25){		//加速度校准第五步
//			calStep = LEFT;
//			AccCalibrate(calStep);
//		}
//		if(*(dataBuffer+4)==0X26){		//加速度校准第六步
//			calStep = RIGHT;
//			AccCalibrate(calStep);
//		}
		
	}
	
	if(*(dataBuffer+2)==0X02){		//命令集合2
		if(*(dataBuffer+4)==0X01){		//请求pid数据
			DateTransfer();		//发送pid数据
		}
		if(*(dataBuffer+4)==0X02){		//读取飞行模式设置请求

		}
		if(*(dataBuffer+4)==0XA0){		//读取下位机版本信息
			
		}
		if(*(dataBuffer+4)==0XA1){		//恢复默认参数
			// pidResetFlag = 1;		//标志位置1
			SetPidResetFlag();
			AllPidInit();		//重新初始化pid数据
			DateTransfer();		//发送pid数据
		}
	}

	if(*(dataBuffer+2)==0X10){		//写入PID1
		pAllPid = pSetAllPid();
		
		pAllPid->rolGyro.kp = 0.01*( (vs16)(*(dataBuffer+4)<<8)|*(dataBuffer+5) );
		pAllPid->rolGyro.ki = 0.01*( (vs16)(*(dataBuffer+6)<<8)|*(dataBuffer+7) );
		pAllPid->rolGyro.kd = 0.01*( (vs16)(*(dataBuffer+8)<<8)|*(dataBuffer+9) );
		pAllPid->pitGyro.kp = 0.01*( (vs16)(*(dataBuffer+10)<<8)|*(dataBuffer+11) );
		pAllPid->pitGyro.ki	= 0.01*( (vs16)(*(dataBuffer+12)<<8)|*(dataBuffer+13) );
		pAllPid->pitGyro.kd	= 0.01*( (vs16)(*(dataBuffer+14)<<8)|*(dataBuffer+15) );
		pAllPid->yawGyro.kp = 0.01*( (vs16)(*(dataBuffer+16)<<8)|*(dataBuffer+17) );
		pAllPid->yawGyro.ki	= 0.01*( (vs16)(*(dataBuffer+18)<<8)|*(dataBuffer+19) );
		pAllPid->yawGyro.kd	= 0.01*( (vs16)(*(dataBuffer+20)<<8)|*(dataBuffer+21) );
		
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
		PidDataWriteToFlash(PID_WRITE_ADDRESS, pAllPid);
	}
	 if(*(dataBuffer+2)==0X11){		//PID2
		pAllPid = pSetAllPid();

		pAllPid->rolAngle.kp = 0.01*( (vs16)(*(dataBuffer+4)<<8)|*(dataBuffer+5) );
		pAllPid->rolAngle.ki = 0.01*( (vs16)(*(dataBuffer+6)<<8)|*(dataBuffer+7) );
		pAllPid->rolAngle.kd = 0.01*( (vs16)(*(dataBuffer+8)<<8)|*(dataBuffer+9) );
		pAllPid->pitAngle.kp = 0.01*( (vs16)(*(dataBuffer+10)<<8)|*(dataBuffer+11) );
		pAllPid->pitAngle.ki = 0.01*( (vs16)(*(dataBuffer+12)<<8)|*(dataBuffer+13) );
		pAllPid->pitAngle.kd = 0.01*( (vs16)(*(dataBuffer+14)<<8)|*(dataBuffer+15) );
		pAllPid->yawAngle.kp = 0.01*( (vs16)(*(dataBuffer+16)<<8)|*(dataBuffer+17) );
		pAllPid->yawAngle.ki = 0.01*( (vs16)(*(dataBuffer+18)<<8)|*(dataBuffer+19) );
		pAllPid->yawAngle.kd = 0.01*( (vs16)(*(dataBuffer+20)<<8)|*(dataBuffer+21) );
		
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
		PidDataWriteToFlash(PID_WRITE_ADDRESS, pAllPid);
		}
	if(*(dataBuffer+2)==0X12){		//PID3	
		pAllPid = pSetAllPid();

		pAllPid->vel_high.kp = 0.01*( (vs16)(*(dataBuffer+4)<<8)|*(dataBuffer+5) );
		pAllPid->vel_high.ki = 0.01*( (vs16)(*(dataBuffer+6)<<8)|*(dataBuffer+7) );
		pAllPid->vel_high.kd = 0.01*( (vs16)(*(dataBuffer+8)<<8)|*(dataBuffer+9) );
		pAllPid->pos_high.kp = 0.01*( (vs16)(*(dataBuffer+10)<<8)|*(dataBuffer+11) );
		pAllPid->pos_high.ki = 0.01*( (vs16)(*(dataBuffer+12)<<8)|*(dataBuffer+13) );
		pAllPid->pos_high.kd = 0.01*( (vs16)(*(dataBuffer+14)<<8)|*(dataBuffer+15) );
		
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
		PidDataWriteToFlash(PID_WRITE_ADDRESS, pAllPid);
	}
	if(*(dataBuffer+2)==0X13){		//PID4
		pAllPid = pSetAllPid();

		pAllPid->acc_high.kp = 0.01*( (vs16)(*(dataBuffer+10)<<8)|*(dataBuffer+11) );
		pAllPid->acc_high.ki = 0.01*( (vs16)(*(dataBuffer+12)<<8)|*(dataBuffer+13) );
		pAllPid->acc_high.kd = 0.01*( (vs16)(*(dataBuffer+14)<<8)|*(dataBuffer+15) );
		
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
		PidDataWriteToFlash(PID_WRITE_ADDRESS, pAllPid);
	}
	if(*(dataBuffer+2)==0X14){		//PID5		
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
	}
	if(*(dataBuffer+2)==0X15){		//PID6
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
	}

}

//摘自匿名四轴
//对上位机发过来的数据帧进行解析
static void ANODTDataReceivePrepare(uint8_t data)
{
	static uint8_t receiveBuffer[50];
	static uint8_t dataLen = 0,cnt = 0;
	static uint8_t state = 0;
	
	if(state == 0 && data == 0xAA){
		state = 1;
		receiveBuffer[0]=data;
	}
	else if(state == 1 && data == 0xAF){
		state = 2;
		receiveBuffer[1] = data;
	}
	else if(state == 2 && data < 0XF1){
		state = 3;
		receiveBuffer[2] = data;
	}
	else if(state ==3 && data < 50){
		state = 4;
		receiveBuffer[3] = data;
		dataLen = data;
		cnt = 0;
	}
	else if(state == 4 && dataLen > 0){
		dataLen--;
		receiveBuffer[4+cnt++] = data;
		if(dataLen==0)
			state = 5;
	}
	else if(state==5){
		state = 0;
		receiveBuffer[4+cnt] = data;
		ANODataReceiveAnalysis(receiveBuffer,cnt+5);
	}
	else{
		state = 0;
	}

}

void AnoReceiveData(void)
{
	uint16_t dataLen = 0;
	uint8_t data = 0;
	
	dataLen = QueueLength(&usartMessageQueue);
	
	while(dataLen > 0){
		dataLen = QueueLength(&usartMessageQueue);
		DeQueue(&usartMessageQueue, &data);
		ANODTDataReceivePrepare(data);
	}

}

//串口2中断服务程序
void USART2_IRQHandler(void)                	
{
	uint8_t data = 0;
	
//	rt_interrupt_enter();
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){  //接收中断
		data = USART_ReceiveData(USART2);		//(USART1->DR);	//读取接收到的数据
		EnQueue(&usartMessageQueue, data);		//先将数据存入数组，后进行处理，以免影响系统实时性
	}
	
//	rt_interrupt_leave();

} 	

