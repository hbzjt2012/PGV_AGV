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

void AGV_Control_Class::Write_Velocity(const Position_Class::Coordinate_Class&AGV_Current_Coor_InWorld, const Position_Class::Coordinate_Class&AGV_Target_Coor_InWorld, Position_Class::Velocity_Class &AGV_Target_Velocity_InAGV)
{
	Position_Class::Coordinate_Class Error_Coor_InWorld = AGV_Target_Coor_InWorld - AGV_Current_Coor_InWorld;	//获取世界坐标系中的误差
	Position_Class::Coordinate_Class Error_Coor_InAGV;
	//Error_Coor_InAGV = Position_Class::Absolute_To_Relative(Error_Coor_InWorld, Error_Coor_InAGV, AGV_Current_Coor_InWorld);	//获取在当前AGV坐标系中的误差

	Error_Coor_InAGV = Position_Class::Absolute_To_Relative(AGV_Target_Coor_InWorld, Error_Coor_InAGV, AGV_Current_Coor_InWorld);	//获取在当前AGV坐标系中的误差

	AGV_Target_Velocity_InAGV = Update_Velocity_By_ErrorCoor(Error_Coor_InAGV, AGV_Target_Velocity_InAGV);	//根据误差更新速度
	AGV_Target_Velocity_InAGV = Update_Velocity_By_Limit(AGV_Target_Velocity_InAGV);	//对速度限幅
	Write_Velocity(AGV_Target_Velocity_InAGV);	//更新速度
}
