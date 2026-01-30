#ifndef _SHELL_LIB_H_
#define _SHELL_LIB_H_

#include "stm32f10x.h"

//#define SHELL_ENABLE

#ifdef SHELL_ENABLE
#define SHELL_CMD_AMOUNT 5		//shell指令的总注册数量，可根据需求扩大，最小为1
#define SHELL_CMD_STORE  10		//保存shell历史指令的总数量，最小为1

/* 定义函数类型 */
typedef int(*cmd_func_t)(int argc, char**argv);

typedef struct /* 定义命令结构体 */
{
	char* name;		//命令名
	cmd_func_t func;		//要执行的函数
	char* help;		//命令提示
}S_Shell;

int ShellMatchCmd(int argc, char**argv);
int ShellGetChar(char receiveChar, char *line, uint8_t maxLen);
int ShellSplitParam(char* msg, char*delim, char* get[], int max_num);
int ShellCmdRegister(S_Shell *cmdType);

#endif

#endif
