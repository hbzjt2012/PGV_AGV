#pragma once
#include "Movement.h"

class Movement_Mecanum_Class:public Movement_Class
{
public:
	bool Init(Interpolation_Parameter_TypedefStructure Input, const Coordinate_Class &Current_Coor, const float threshold) override;	//根据插补参数以及阈值插补运动路径

private:
	float Cal_Destination_Displacement(const Coordinate_Class Destination_Coor_InOrigin) override;	//根据终点坐标在起点坐标中的坐标计算插补距离
	Velocity_Class& Assign_Velocity(const Coordinate_Class&Destination_Coor_InOrigin, const float velocity) override;	//根据终点坐标在起点坐标中的坐标，将合速度分配给各个轴
	float Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin) override;	//根据当前坐标计算在源坐标系上的位移


};
