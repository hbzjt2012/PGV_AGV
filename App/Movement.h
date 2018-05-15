#pragma once

//用于存放运动指令

/*
* 速度插补
* 采用梯形插补
* 分为四段（考虑到电机存在最小速度）
*	匀加速段ta
*	匀速段tc
*	匀减速段td
*	匀低速段ts
*/

#include "Position.h"
#include "../macros.h"

class Movement_Class
{
public:
	Movement_Class() :Interpolation_OK(false) {}
	~Movement_Class() = default;
	typedef struct
	{
		float max_velocity_abs; //最大速度(mm/ms或°/ms)
		float min_velocity_abs; //最小速度(mm/ms或°/ms)
		float acceleration_abs; //加速度(mm/ms2或°/ms2)
		float slow_distance_abs;	//最小速度移动的位移(mm或°)
	} Actual_INPUT_TypedefStructure;

	virtual bool Init(const Actual_INPUT_TypedefStructure&Input, float threshold, bool Is_Linear = true) = 0;	//根据运动输入条件(限制)插补运动路径
	bool Get_Expectation(const Coordinate_Class Current_Coor_InWorld);	//根据当前坐标计算目标坐标，目标速度
	void Set_Destination(Coordinate_Class &coor) { Destination_Coor_InWorld = coor; }
	void Set_Origin(Coordinate_Class&coor) { Origin_Coor_InWorld = coor; }

	bool Interpolation_OK;	//true表示插补完成
	bool Is_Linear;	//表示当前为直线运动

	static Velocity_Class Target_Velocity_InAGV;	//目标速度
	static Coordinate_Class Target_Coor_InWorld;	//目标坐标

protected:

	virtual float Cal_Displacement(const Coordinate_Class Destination_Coor_InOrigin) = 0;	//根据终点坐标在起点坐标中的坐标计算插补距离
	virtual Velocity_Class& Cal_Velocity(const Coordinate_Class&Destination_Coor_InOrigin, const float velocity) = 0;	//根据终点坐标在起点坐标中的坐标，将合速度分配给各个轴
	virtual float Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin) = 0;	//根据当前坐标计算在源坐标系上的位移

	Coordinate_Class Destination_Coor_InWorld;	//终点坐标
	Coordinate_Class Origin_Coor_InWorld;	//起点坐标
	Coordinate_Class Destination_Coor_InOrigin;	//起点坐标系中的终点坐标

	Actual_INPUT_TypedefStructure Input_Para;



	//因为同一时间只会执行一条运动指令，故为静态变量
	static float X_H_mul_X;	//用于计算投影向量
	static int Distance_Symbols; //指示待插补数据的符号

	static float acc_distance;	//加速段距离(mm)
	static float const_distance;  //匀速段距离(mm)
	static float dec_distance;	//减速段距离(mm)
	static float slowly_distance; //慢速段距离(mm)

	static float acceleration_time; //加速段时间(ms)
	static float const_time;		 //匀速段时间(ms)
	static float deceleration_time; //减速段时间(ms)
	static float slowly_time;		 //慢速段时间(ms)
};
