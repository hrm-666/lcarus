#ifndef _LOG_LIB_H_
#define _LOG_LIB_H_

#include <stdio.h>
#include "usart.h"

//打印颜色
#define COLOR_FMT_NONE                  "\033[0m"       //默认色
#define COLOR_FMT_DARK_GRAY             "\033[1;30m"    //深灰色
#define COLOR_FMT_RED                   "\033[0;31m"    //红色
#define COLOR_FMT_LIGHT_RED             "\033[1;31m"    //亮红色
#define COLOR_FMT_GREEN                 "\033[0;32m"    //绿色
#define COLOR_FMT_LIGHT_GREEN           "\033[1;32m"    //亮绿色
#define COLOR_FMT_YELLOW                "\033[0;33m"    //黄色
#define COLOR_FMT_LIGHT_YELLOW          "\033[1;33m"    //亮黄色
#define COLOR_FMT_BLUE                  "\033[0;34m"    //蓝色
#define COLOR_FMT_LIGHT_BLUE            "\033[1;34m"    //亮蓝色
#define COLOR_FMT_PURPLE                "\033[0;35m"    //紫色
#define COLOR_FMT_LIGHT_PURPLE          "\033[1;35m"    //亮紫色
#define COLOR_FMT_CYAN                  "\033[0;36m"    //青色
#define COLOR_FMT_LIGHT_CYAN            "\033[1;36m"    //亮青色
#define COLOR_FMT_LIGHT_GRAY            "\033[0;37m"    //浅灰色
#define COLOR_FMT_WHITE                 "\033[1;37m"    //白色
#define COLOR_FMT_GREEN_92_0            "\033[92;0m"    //绿色0
#define COLOR_FMT_GREEN_92_1            "\033[92;1m"    //绿色1
#define COLOR_FMT_GREEN_92_2            "\033[92;2m"    //绿色2
#define COLOR_FMT_GREEN_92_3            "\033[92;3m"    //绿色3
#define COLOR_FMT_GREEN_92_4            "\033[92;4m"    //绿色4
#define COLOR_FMT_GREEN_92_5            "\033[92;5m"    //绿色5

/* DEBUG level */
#define DEBUG_INFO            0
#define DEBUG_LOG             1

//设置debug等级，该宏定义要放在#include"log_lib.h"之前才能生效
//示例如下：
//#define DEBUG_LEVEL	DEBUG_LOG
//#include "log_lib.h"
#ifdef  DEBUG_LEVEL
#define DEBUG_LVL       DEBUG_LEVEL
#endif

void Usart1DmaPrintf(const char *format, ...);
void CounterPrintf(uint16_t delayCount, uint16_t *count, const char *format, ...);
void PeriodPrintf(float periodTime, const char *format, ...);
	
#define LogInfo(format, ...)   printf(COLOR_FMT_WHITE"[info]: "format"\r\n",##__VA_ARGS__)
#define LogWarnning(format, ...)   printf(COLOR_FMT_YELLOW"[Warnning]====: "format"\r\n",##__VA_ARGS__)
#define LogError(format, ...)   printf(COLOR_FMT_RED"[error]========: "format"\r\n",##__VA_ARGS__)
#define LogRaw(format, ...)     printf(format"\r\n", ##__VA_ARGS__)
#define LogDma(format, ...)		Usart1DmaPrintf("[dma] "format"\r\n",##__VA_ARGS__)		//不占用cpu，效率高，能有效防止高频率打印卡死问题

#define LogCount(delayCount, count, format, ...)			CounterPrintf(delayCount, &count, format"\r\n", ##__VA_ARGS__)
#define LogPeriod(periodTime, format, ...)			PeriodPrintf(periodTime, format"\r\n", ##__VA_ARGS__)
#define LogErrorPeriod(periodTime, format, ...)		PeriodPrintf(periodTime, COLOR_FMT_RED"[error]========: "format"\r\n", ##__VA_ARGS__)
//=================dbug打印=================↓
#if (DEBUG_LVL >= DEBUG_LOG)
//=================打印当前函数=================↓
#ifdef  PRINTF_FUNC     //
#define LogDebug(format, ...)      printf(""COLOR_FMT_CYAN"[debug]/%s: "format"\r\n",__func__,##__VA_ARGS__)
//=================打印当前函数=================↑
#else
#define LogDebug(format, ...)      printf(""COLOR_FMT_CYAN"[debug]: "format"\r\n",##__VA_ARGS__)
#endif
//=================dbug打印=================↑
#else

#define LogDebug(...)

#endif

void PrintfCountHandle(uint16_t time);
void PeriodPrintf(float periodTime, const char *format, ...);

#endif

