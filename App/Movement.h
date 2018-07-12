#pragma once

//用于存放运动插补指令

#include "Position.h"
#include "../parameter_define.h"
#include "Interpolation.h"

class Movement_Class :public Interpolation_Class
{
public:
	Movement_Class() = default;
	~Movement_Class() = default;

	bool Init(const Coordinate_Class& Origin_Coor, const Interpolation_Parameter_TypedefStructure& Input_Para);	//根据目标坐标和插补参数插补路径
	bool Cal_Velocity(const Coordinate_Class Current_Coor_InWorld);	//根据当前坐标计算目标坐标，目标速度，返回计算结果

	void Set_Destination(const Coordinate_Class& Destination, const float threshold) {
		Destination_Coor_InWorld = Destination;
		this->threshold = threshold;
	}	//设置终点

	static Velocity_Class Target_Velocity_InAGV;	//目标速度
	static Coordinate_Class Target_Coor_InWorld;	//目标坐标

protected:
	virtual Coordinate_Class& Cal_Projection_Coor(const Coordinate_Class &Current_Coor_InOrigin) = 0;	//计算投影坐标
	virtual float Cal_Destination_Displacement(const Coordinate_Class Destination_Coor_InOrigin) = 0;	//根据终点坐标在起点坐标中的坐标计算插补距离
	virtual Velocity_Class& Assign_Velocity(const Coordinate_Class&Destination_Coor_InOrigin, const float velocity) = 0;	//根据终点坐标在起点坐标中的坐标，将总速度分配给各个轴
	virtual float Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin) = 0;	//根据当前坐标计算在起点坐标系上的位移

	Coordinate_Class Destination_Coor_InWorld;	//终点坐标
	Coordinate_Class Origin_Coor_InWorld;	//起点坐标
	Coordinate_Class Destination_Coor_InOrigin;	//起点坐标系中的终点坐标
};


