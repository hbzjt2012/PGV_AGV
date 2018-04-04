#pragma once
#include <math.h>

namespace MyMath
{
	typedef struct
	{
		float x;
		float y;
	} Coor;

	//过点point,求垂直于斜率为slope的线的交点solution
	void Get_Vertical_Line_Crossover_Point(const float slope, const Coor &point, Coor &solution);
}
