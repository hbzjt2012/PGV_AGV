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

	virtual void Init(void) = 0;	//初始化控制车体所需的硬件接口
	virtual void Brake(bool value) = 0;	//刹车
	virtual void Run(bool value) = 0;	//使能

	//根据AGV的跟踪误差与期望速度，更新速度
	void Write_Velocity(const Coordinate_Class&AGV_Current_Coor_InWorld, const Coordinate_Class&AGV_Target_Coor_InWorld, Velocity_Class &AGV_Target_Velocity_InAGV);
	//根据跟踪误差更新期望速度
	virtual void Update_Velocity_By_ErrorCoor(const Coordinate_Class&Error_Coor_InAGV, Velocity_Class& Target_Velocity) = 0;
	//根据物理约束限幅速度,将车体速度转换为轮速，控制电机
	virtual void Write_Velocity(Velocity_Class &AGV_Velocity_InAGV) = 0;

	//由编码器计算AGV车体速度
	virtual float Cal_Velocity_By_Encoder(Velocity_Class & AGV_Velocity) = 0;

	Velocity_Class AGV_Velocity_InAGV;	//AGV坐标系下由编码器获取的AGV速度
	Coordinate_Class AGV_Coor_InWorld;	//世界坐标系下由编码器获取的AGV坐标
};
