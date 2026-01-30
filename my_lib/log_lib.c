#include "log_lib.h"
#include "usart.h"

#define UART1_SEND_BUFF_LENGTH	512

static uint8_t uart1SendBuff[UART1_SEND_BUFF_LENGTH] = {0};

/******************************************************************************************/

/* 在合适的位置引用下面头文件 */
#include <stdio.h>

/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 
其中串口可根据实际使用情况调整 */
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0);             /* 等待上一个字符发送完成 */

	USART1->DR = (uint8_t)ch;                     /* 将要发送的字符 ch 写入到DR寄存器 */
	
	return ch;
}
#endif
/***********************************************END*******************************************/


/*串口DMA方式打印*/
#include <stdio.h>
#include "stdarg.h"
void Usart1DmaPrintf(const char *format, ...)
{
	uint16_t len;
	va_list args;	
	va_start(args, format);
//	len = vsnprintf((char*)uart1SendBuff, sizeof(uart1SendBuff) + 1, (char*)format, args);
	len = vsprintf((char*)uart1SendBuff, (char*)format, args);
	va_end(args);
	Usart1DMASendData((uint8_t *)uart1SendBuff, len);
}

/**
* @brief 计数打印，达到设定的计数值后打印
* @param delayCount: 设定计数次数
* @param count: 累计计数次数，需声明为static，初值为0
* @param format: 打印的字符串
* @param ...: 可变打印参数
*/
void CounterPrintf(uint16_t delayCount, uint16_t *count, const char *format, ...)
{
	if((*count) == 0){		//调用第一次就打印
		(*count)++;
		
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	} else{
		
		(*count)++;
		if(*count >= delayCount){
			*count = 1;
			
			va_list args;
			va_start(args, format);
			vprintf(format, args);
			va_end(args);
		}
	}

}

static uint16_t printfCount = 0;
static uint16_t delayTime = 0;

/**
* @brief 打印计数值处理
* @param time: 线程的间隔时间
*/
void PrintfCountHandle(uint16_t time)
{
	if(delayTime == 0){
		delayTime = time;
	}
	
	printfCount++;
	if(printfCount > 5000){
		printfCount = 0;
	}
}

/**
* @brief 周期打印，该接口需配合PrintfCountHandle()使用
* @param periodTime: 打印设定的周期时间,单位为秒，最小为0.001
* @param format: 打印的字符串
* @param ...: 可变打印参数
*/
void PeriodPrintf(float periodTime, const char *format, ...)
{
	if(delayTime == 0){
		return;
	}
		
	periodTime *= (1000 / delayTime);		//计算每个间隔所需要的计数值

	if((printfCount % (uint16_t)periodTime) == 0){		//达到设定的计数值就打印
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}
}
