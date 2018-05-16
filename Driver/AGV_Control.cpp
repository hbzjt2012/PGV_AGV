#include "AGV_Control.h"
#include "../macros.h"


void AGV_Control_Class::Write_Velocity(const Coordinate_Class&AGV_Current_Coor_InWorld, const Coordinate_Class&AGV_Target_Coor_InWorld, Velocity_Class &AGV_Target_Velocity_InAGV)
{
	Coordinate_Class Error_Coor_InAGV;	//跟踪误差

	Error_Coor_InAGV = AGV_Target_Coor_InWorld - AGV_Current_Coor_InWorld;	//获取跟踪误差

	Update_Velocity_By_ErrorCoor(Error_Coor_InAGV, AGV_Target_Velocity_InAGV);	//根据跟踪误差更新期望速度
	Write_Velocity(AGV_Target_Velocity_InAGV);	//根据速度控制电机运动
}
