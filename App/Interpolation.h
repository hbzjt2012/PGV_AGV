#pragma once
/*
* 速度插补
* 采用梯形插补
* 分为四段（考虑到电机存在最小速度）
*	匀加速段ta
*	匀速段tc
*	匀减速段td
*	匀低速段ts 优先满足匀低速段距离>=20mm
*/
#include "../macros.h"

namespace Interpolation
{
	typedef struct
	{
		float displacement;		//位移(mm或°)
		float max_velocity_abs; //最大速度(mm/ms或°/ms)
		float min_velocity_abs; //最小速度(mm/ms或°/ms)
		float acceleration_abs; //加速度(mm/ms2或°/ms2)
		float slow_distance_abs;	//最小速度移动的位移(mm或°)
	} Actual_INPUT_TypedefStructure;

	void Init(Actual_INPUT_TypedefStructure &Input);
	bool Get_Expectation(float &output_velocity, float current_coor,float &target_coor);
}
