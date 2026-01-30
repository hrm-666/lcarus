#include "filter_lib.h"
#include <string.h>
#include "log_lib.h"

#define PI_FILTER	3.1415926f

//滑动中值滤波器demo
void MoveMiddleFilterTest(void)
{
	float arr[11] = {0.1f, 0.66f, 0.14f, 0.55f, 0.69f, 0.789f, 0.357f, 0.159f, 0.456f, 0.147f, 0.258f};
	static float arrStore[11] = {0};
	static uint8_t num = 0;
	float temp = 0;

	// SmallToLargeSort((uint32_t *)arr, 11);

	for(uint8_t i = 0; i < 11; i++){
		temp = MoveMiddleFilter(&num, arrStore, 11, arr[i]);
		LogRaw("%f,", temp);

		for(uint8_t i = 0; i < 11; i++){
			printf("%f, ", arrStore[i]);
		}
	}
	
	// SmallToLargeSort((uint32_t *)arr, 11);
	// memcpy((void *)arrStore, (void *)arr, sizeof(uint32_t) * 11);
	// for(uint8_t i = 0; i < 11; i++){
	// 	printf("%f, ", arr[i]);
	// }
}

//小到大排序
void SmallToLargeSort(uint32_t *arr, uint8_t len)
{
    uint32_t temp = 0;

    for(int i = 0; i < len; i++){
        for(int j = 0; j < (len - i - 1); j++){
            if(arr[j] > arr[j + 1]){
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
* @brief 滑动中值滤波
* @param num: 存储当前数据序号，需声明成static，初始化为0
* @param arr: 滤波值数组，需声明成static
* @param n: 均值个数，范围 <= 21，范围越大，计算量越大，需为单数
* @param input: 滤波前的值
* @return 滤波后的值
*/
float MoveMiddleFilter(uint8_t *num, float *arr, uint8_t n, float input)
{
	static float arrTemp[21] = {0};

	if(n > 21){
		printf("length > 21, filter fail\r\n");
		return 1;
	}

	arr[*num] = input;
	(*num)++;

	memcpy((void *)arrTemp, (void *)arr, sizeof(uint32_t) * n);		//拷贝一份数据，防止打乱原数据
	SmallToLargeSort((uint32_t *)arrTemp, n);		//小到大排序

	if(*num == n){		//数据到末尾，则复位到首位
		*num = 0;
	}

	return arrTemp[(n - 1) / 2];
}

//滑动均值滤波，待验证
/**
* @brief 滑动均值滤波
* @param num: 存储当前数据序号，需声明成static，初始化为0
* @param arr: 滤波值数组，需声明成static
* @param n: 均值个数
* @param input: 滤波前的值
* @return 滤波后的值
*/
float MoveAverageFilter(uint8_t *num, float *arr, uint8_t n, float input)
{
	float sum = 0;
	
	arr[*num] = input;
	(*num)++;
	
	for(int i = 0; i < n; i++){
		sum += arr[i];
	}
	
	if(*num == n){		//数据到末尾，则复位到首位
		*num = 0;
	}

	return sum / n;
}

/**
* @brief : 一阶巴特沃斯低通滤波器
* @param filter: 指向滤波器参数指针
* @param input: 滤波前的值
* @return: 滤波后的值
*/
float ButterworthLpf(S_ButterworthLpf *filter, float input)
{
	float output = 0;
	
	if(filter->alpha == 0){		//防止重复计算alpha
		filter->alpha = (2 * PI_FILTER * filter->fc) / (2 * PI_FILTER * filter->fc + filter->fs);
	}
	
	output = filter->alpha * input + (1 - filter->alpha) *  filter->lastOutput;
	
	filter->lastOutput = output;

	return output;
}

/**
* @brief : 抖动滤波
* @param debounceParam: 滤波参数，需声明为static
* @param input: 滤波前的值
* @return: 滤波后的值
*/
uint16_t DebounceFilter(S_DebounceFilter *debounceParam, uint16_t input)
{
	if(debounceParam->lastValue == input){
		debounceParam->count++;		//当前值等于上一次的值，计数值累加
		if(debounceParam->count == debounceParam->num){		//计数值累加到设定值，则判断为稳定
			debounceParam->recordValue = input;		//更新输出值
			debounceParam->count = 0;		//清空计数值
		}
	} else{
		debounceParam->lastValue = input;
		debounceParam->count = 0;
	}
	
	return debounceParam->recordValue;
}
