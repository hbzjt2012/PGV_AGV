#include "MyMath.h"

//************************************
// Method:    Get_Vertical_Line_Crossover_Point
// FullName:  MyMath::Get_Vertical_Line_Crossover_Point
// Access:    public
// Returns:   void
// Parameter: const float slope
// Parameter: const Coor & point
// Parameter: Coor & solution
// Description: 过点point,求垂直于斜率为slope的线的交点solution
//************************************
void MyMath::Get_Vertical_Line_Crossover_Point(const float slope, const Coor &point, Coor &solution)
{
	solution.x = (slope * point.y + point.x) / (1 + slope * slope);
	solution.y = solution.x * slope;
}
