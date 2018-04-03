#pragma once
/*
* AGV运动控制
*/
#include "../Math/Trigonometric.h"
#include "../App/Position.h"

class AGV_Control_Class
{
  public:
	AGV_Control_Class() = default;
	virtual ~AGV_Control_Class() = default;
	virtual void Init(void) = 0;
	virtual void Brake(bool value) = 0;
	virtual void Run(bool value) = 0;

	virtual void Write_Velocity(Position_Class::Velocity_Class &velocity_InAGV) = 0;												  //根据输入速度控制小车
	virtual Position_Class &Update_Post(Position_Class &Current_Position, const Position_Class::Velocity_Class &Target_velocity) = 0; //更新当前坐标和速度

	Position_Class::Coordinate_Class &Absolute_To_Relative(Position_Class::Coordinate_Class &Absolute_Coor, Position_Class::Coordinate_Class &Relative_Coor)
	{
		return Position_Class::Absolute_To_Relative(Absolute_Coor, Relative_Coor, this->V_InAGV_Coor_InWorld.Coordinate);
	} //绝对坐标转换为相对坐标
	Position_Class::Coordinate_Class &Relative_To_Absolute(Position_Class::Coordinate_Class &Absolute_Coor, Position_Class::Coordinate_Class &Relative_Coor)
	{
		return Position_Class::Relative_To_Absolute(Absolute_Coor, Relative_Coor, this->V_InAGV_Coor_InWorld.Coordinate);
	} //相对坐标转换为绝对坐标

	Position_Class V_InAGV_Coor_InWorld; //AGV坐标系下的速度和世界坐标系下的坐标

  private:
};
