#include "board_config.h"
#include "pwm.h"

void BoardConfig(void)
{
#ifdef BRUSHLESS_FOUR_AXIS_UAV
SetPwmConfig(2000-1, 64-1, 1000);
#else
SetPwmConfig(1000-1, 4-1, 0);
#endif
}



