#include "shell_lib.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "log_lib.h"

#ifdef SHELL_ENABLE

int cmd_help(int argc, char**argv);

static S_Shell shellCmd[SHELL_CMD_AMOUNT] = {"help", cmd_help, "Show this list"};
static int shellCmdCount = 1;		//记录注册的指令数量

static char shellCmdStore[SHELL_CMD_STORE][64] = {0};		//二维数组指针，指向用于存放历史指令的数组
static uint8_t cmdStoreNum = 0;		//指令存取位置

static uint8_t readNum = 0;		//记录读取的位置
static uint8_t fillFlag = 0;		//历史指令存储已满标志

int ShellCmdRegister(S_Shell *cmdType)
{
	if(shellCmdCount < SHELL_CMD_AMOUNT){		//判断指令数量是否已满
		
		shellCmd[shellCmdCount].name = cmdType->name;
		shellCmd[shellCmdCount].func = cmdType->func;
		shellCmd[shellCmdCount].help = cmdType->help;
		
		shellCmdCount++;
		
		return shellCmdCount;
	} else{
		return -1;
	}
}

int cmd_help(int argc, char**argv)
{
	for(int i=0; i < shellCmdCount; i++){
		printf("%-10s %s\r\n", shellCmd[i].name, shellCmd[i].help);
	}
	
	return 0;
}

void ShellCmdStore(char *cmd)
{
	if(strcmp(cmd, "\0") != 0){
		
		strcpy(shellCmdStore[cmdStoreNum++], cmd);
		if(cmdStoreNum == SHELL_CMD_STORE){
			cmdStoreNum = 0;
			fillFlag = 1;
		}
		
		readNum = cmdStoreNum;
		
	}
	
}

typedef enum{
	READ_FRONT = 0,
	READ_BACK,
}E_ReadDirect;

int ShellCmdGet(char *cmd, uint8_t readDirect)
{
	LogDebug("readNum: %d", readNum);
	LogDebug("cmdStoreNum: %d", cmdStoreNum);
	
	if(readDirect == READ_FRONT){
		
		if(cmdStoreNum == 0 && fillFlag == 0){		//判断是否有历史指令
			return -1;
		}
		else if(readNum == 0 && fillFlag == 1){		//如果读取位置到首部时，则更新位置到尾部
			readNum = SHELL_CMD_STORE;
		}
		else if((readNum - 1) == cmdStoreNum){		//读取位置和存储位置相同时，则不更新读取位置
			strcpy(cmd, shellCmdStore[readNum]);
			return 0;
		}
		else if(readNum == 0 && fillFlag == 0){		//若位置已经移到最早的指令，则不更新位置
			strcpy(cmd, shellCmdStore[readNum]);
			return 0;
		}
		
		readNum--;		//更新读取位置
		strcpy(cmd, shellCmdStore[readNum]);		//读取
	}
	else if(readDirect == READ_BACK){
		if(cmdStoreNum == 0 && fillFlag == 0){		//判断是否有历史指令
			return -1;
		}
		else if(readNum == SHELL_CMD_STORE - 1 && fillFlag == 1){		//如果读取位置到末尾时，则更新位置到头部
			readNum = 0;
		}
		else if(readNum == cmdStoreNum - 1){		//读取位置和存储位置相同时，则不更新读取位置
			strcpy(cmd, shellCmdStore[readNum]);
			return 0;
		}
		
		readNum++;		//更新读取位置
		strcpy(cmd, shellCmdStore[readNum]);		//读取
	}
		
	return 0;
}

/**
* @brief 将输入的字符串按delim字符分割开
* @param msg: 要分割的字符串
* @param get[]: 存放分割后的字符串地址
* @param max_num: 允许最大分割的数量
* @return	返回-1, 失败；返回其他大于0的数值，表示分割的字符串数量
*/
int ShellSplitParam(char* msg, char*delim, char* get[], int max_num)
{
	int i = 0, ret = 0;
	char *ptr = NULL;
	
	//strtok()是字符串分割函数，msg是要分割的字符，delim作分隔符的字符
	ptr = strtok(msg, delim);
	
	for(i = 0; ptr != NULL; i++){
		//判断参数数量是否存在溢出
		if(i == max_num){
			printf("error: param so much\r\n");
			return -1;
		}
				
		get[i] = ptr;
		
		//传入的参数为NULL，使得该函数默认使用上一次未分割完的字符串继续分割，
		//就从上一次分割的位置作为本次分割的起始位置，直到分割结束
		ptr = strtok(NULL, delim);
		
	}
	
	ret = i;
	
	return ret;
}

/**
* @brief 用于从串口获取一个字符
* @param receiveChar: 串口接收的字符
* @param line: 字符缓冲区
* @param maxLen: 允许接收的最大数量
* @return	1，接收完输入的字符串；0，等待接收回车；-1，接收超出长度
*/
int ShellGetChar(char receiveChar, char *line, uint8_t maxLen)
{
	static uint8_t count = 0; /*用于记录除特殊字符外的其他有效字符的数量*/
	static uint8_t oprationCount = 0;
	
	if (count >= maxLen){ /*长度超限*/
		count = 0; /*清零计数器以便后续使用*/
		return -1;  /*返回有效标志*/
	}
	
	line[count] = receiveChar; /*记录数据*/
	
	switch (receiveChar){
		case 0x1B:
			oprationCount++;
			return 0;
		case 0x5B:
			oprationCount++;
			return 0;
		case 'A':
			if(oprationCount == 2){
				oprationCount = 0;
				ShellCmdGet(line, READ_FRONT);
				count = strlen(line);
				printf("\r\n->%s", line);
			}
			return 0;
		case 'B':
			if(oprationCount == 2){
				oprationCount = 0;
				ShellCmdGet(line, READ_BACK);
				count = strlen(line);
				printf("\r\n->%s", line);
			}
			return 0;
		case 0x08:
		case 0x7F: /*退格键或者删除键*/
			if (count > 0)
			{
				count--; /*删除上一个接收到的字符*/
			}
			
			line[count] = '\0';
			printf("\r\n%s", line);		//打印删除后的字符串
			
			return 0;		//不打印退格键和删除键
		case '\r':
		case '\n': /*接收到回车换行，证明已经收到一个完整的命令*/
			if(line[count] != '\0'){
				line[count] = '\0'; /*添加字符串结束符，刚好可以去掉'\r'或者'\n'*/
			}
			
		ShellCmdStore(line);		//保存当前指令
			count = 0;			/*清零计数器以便后续使用*/
			return 1;			/*返回有效标志*/
		default:
			count++;
	}
 
	printf("%c", receiveChar);		/*把收到的字符输出到串口*/

	return 0;
}

/**
* @brief 查找拆分出来的字符串，若存在对应命令（函数），则执行对应命令
* @param argc: 命令的参数数量
* @param argv: 存放字符串指针的数组
* @return	0，成功；-1，不存在对应命令或参数数量不正确
*/
int ShellMatchCmd(int argc, char**argv)
{
	int cmdErrorFlag = 1;
	
	if(argc == -1){
		return -1;
	}
	
	for(int i = 0; i < shellCmdCount; i++){
		
		//argv[0]存放的是函数名，查询是否存在对应函数
		if(strcmp(argv[0], shellCmd[i].name) == 0){
			cmdErrorFlag = 0;
			shellCmd[i].func(argc, argv);
		}
	}
	
	if(cmdErrorFlag == 1){
		printf("cmd \"%s\" does not exist!!\r\n", argv[0]);
		return -1;
	}
	
	return 0;
}

//void ShellHandle(void)
//{
//	uint8_t data = 0;
//	static char shellBuff[64] = {0};
//	
//	if(QueueLength(&usart1MessageQueue) > 0){		//判断队列是否有数据
//		DeQueue(&usart1MessageQueue, &data);		//从队列中取出一个字符
//		
//		if(ShellGetChar((char)data, shellBuff, 64)){		//将字符整合成字符串存入到shellBuff中
//			
//			printf("\r\n");
//			
//			/* 获得到指令 */
//			if(strlen(shellBuff)){ /* 判断接收到的指令字符串是否为0 */
//				
//				char *argv[16] = {NULL};		//指针数组，存放的数据为指针
//				
//				int argc = ShellSplitParam(shellBuff, " ", argv, 16);		//将字符串以空格进行拆分
//				
//				ShellMatchCmd(argc, argv);		//执行对应的函数
//			
//			}
//			printf("->");
//		}
//	}
//}

///*
//shell使用示例，仅展示主要部分，仅供参考

//int TestCmd(int argc, char**argv);

////创建一个cmd_t变量，用于注册shell
//static cmd_t testCmd = {"testcmd", TestCmd, "test tips"};

//int TestCmd(int argc, char**argv)
//{
//	printf("TestCmd\r\n");
//}

//int main(void)
//{
//	ShellCmdRegister(&testCmd);

//	while(1){
//		ShellHandle();
//	}
//	
//}

////串口1中断服务程序
//void USART1_IRQHandler(void)
//{
//	uint8_t data = 0;
//	
////	rt_interrupt_enter();
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){  //接收中断
//		data = USART_ReceiveData(USART1);		//读取接收到的数据
////		UsartDataStorage(data);		//先将数据存入数组，后进行处理，以免影响系统实时性
//		EnQueue(&usart1MessageQueue, data);
//	} 
//	
////	rt_interrupt_leave();

////} 

#endif
