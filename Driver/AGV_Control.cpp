#include "AGV_Control.h"
#include "../macros.h"

unsigned short AGV_Control_Class::Cal_Cycle(void)
{
	unsigned short time = 0;	//控制周期(10us)
	if (velocity > FLOAT_DELTA)	//AGV移动速度不为0
	{
		time = (unsigned short)(1 / velocity * 100000UL);	//精度为1mm,1°
		time++;
	}
	return time;
}
