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

	unsigned short Cal_Cycle(void);	//根据车体速度计算控制周期

	void Write_Velocity(const Coordinate_Class&AGV_Current_Coor_InWorld, const Coordinate_Class&AGV_Target_Coor_InWorld, Velocity_Class &AGV_Target_Velocity_InAGV);	//根据AGV在世界坐标系中的当前位置和期望位置，更新速度

	virtual Coordinate_Class &Update_Coor_By_Encoder_demo(const Coordinate_Class Coor_InWorld_Last)=0;	//上一时刻的AGV坐标
	virtual Velocity_Class & Update_Velocity_By_ErrorCoor(const Coordinate_Class&Error_Coor_InAGV, Velocity_Class &AGV_Velocity_InAGV) = 0;	//根据位姿误差更新期望速度
	virtual Velocity_Class& Update_Velocity_By_Limit(Velocity_Class&Velocity) = 0;	//对速度限幅
	virtual void Write_Velocity(Velocity_Class &AGV_Velocity_InAGV) = 0;	//将AGV速度转换为车轮速度，更新velocity
	virtual Velocity_Class &Cal_Velocity_By_Encoder(void)= 0;	//根据编码器获取AGV速度

	Velocity_Class AGV_Velocity_InAGV;	//AGV坐标系下由编码器获取的AGV速度
	Coordinate_Class AGV_Coor_InWorld;	//世界坐标系下由编码器获取的AGV坐标

	float velocity;	//AGV速度大小的度量，根据实际车型决定计算方法，由Write_Velocity()方法计算

};
