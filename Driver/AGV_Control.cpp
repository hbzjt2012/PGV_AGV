#include "AGV_Control.h"
#include "../macros.h"


void AGV_Control_Class::Write_Velocity(const Coordinate_Class&AGV_Current_Coor_InWorld, const Coordinate_Class&AGV_Target_Coor_InWorld, Velocity_Class &AGV_Target_Velocity_InAGV)
{
	Coordinate_Class Error_Coor_InAGV;	//跟踪误差

	//跟踪误差应该是目标坐标在当前坐标系的坐标
	//Error_Coor_InAGV = AGV_Target_Coor_InWorld - AGV_Current_Coor_InWorld;	//获取跟踪误差

	//绝对坐标差方便计算
	Error_Coor_InAGV.x_coor = AGV_Target_Coor_InWorld.x_coor - AGV_Current_Coor_InWorld.x_coor;
	Error_Coor_InAGV.y_coor = AGV_Target_Coor_InWorld.y_coor - AGV_Current_Coor_InWorld.y_coor;
	Error_Coor_InAGV.angle_coor = Coordinate_Class::Transform_Angle(AGV_Target_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor);

	Update_Velocity_By_ErrorCoor(Error_Coor_InAGV, AGV_Target_Velocity_InAGV, AGV_Current_Coor_InWorld);	//根据跟踪误差更新期望速度
	Write_Velocity(AGV_Target_Velocity_InAGV);	//根据速度控制电机运动

}
