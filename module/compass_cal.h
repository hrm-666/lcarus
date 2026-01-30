#ifndef _COMPASS_CAL_H_
#define _COMPASS_CAL_H_

#include "stm32f10x.h"

typedef enum{
	COMPASS_CAL_SETP_BEGIN = 0,
	COMPASS_CAL_SETP_HORIZONTAL_ROTATION,		//水平旋转
	COMPASS_CAL_SETP_VERTICAL_ROTATION,		//竖直旋转
	COMPASS_CAL_SETP_SAVE,
	COMPASS_CAL_SETP_DONE,
}E_CompassCalStep;

typedef struct{
	//磁力计偏移值
	int16_t x;
	int16_t y;
	int16_t z;
}S_CompassOffset;

void CompassCalibrateHandle(void);
uint8_t GetCompassCalData(int16_t *compassX, int16_t *compassY, int16_t *compassZ);
#endif
