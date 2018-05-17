#pragma once

//用于存放运动指令

/*
* 速度插补
* 采用梯形插补
* 若插补对象是角度，则将角度转化为线距离
* 分为四段（考虑到电机存在最小速度死区）
*	匀加速段ta
*	匀速段tc
*	匀减速段td
*	匀低速段ts
*/

#include "Position.h"
#include "../parameter_define.h"
#include "../macros.h"

class Movement_Class
{
public:
	Movement_Class() :Interpolation_State(NO_Interpolation) {}
	~Movement_Class() = default;

	typedef struct
	{
		float max_velocity_abs; //最大速度(mm/s)
		float min_velocity_abs; //最小速度(mm/s)
		float acceleration_abs; //加速度(mm/s2)
		float slow_time_abs;	//最小速度移动时间(s)
	} Interpolation_Parameter_TypedefStructure;

	enum
	{
		NO_Interpolation,  //未插补
		IS_Interpolating, //正在插补
		IS_Interpolated   //插补完毕
	} Interpolation_State;	//插补状态

	static void Init_Parameter(void);	//初始化插补参数

	bool Init(const Coordinate_Class &Current_Coor) { Init(Interpolation_Parameter, Current_Coor, this->threshold); }	//插补运动路径
	virtual bool Init(Interpolation_Parameter_TypedefStructure Input, const Coordinate_Class &Current_Coor, float threshold) = 0;	//根据插补参数以及阈值插补运动路径
	bool Cal_Velocity(const Coordinate_Class Current_Coor_InWorld);	//根据当前坐标计算目标坐标，目标速度，返回计算结果

	void Set_Destination(const Coordinate_Class& Destination, const float threshold, const bool Is_Linear = true) {
		Destination_Coor_InWorld = Destination;
		this->Is_Linear = Is_Linear;
		this->threshold = threshold;
	}	//设置终点
	//void Set_Origin(const Coordinate_Class& Origin) { Origin_Coor_InWorld = Origin; }	//设置起点
	static void Update_Interpolation_Parameter(const Interpolation_Parameter_TypedefStructure& Input_Para) {
		Interpolation_Parameter = Input_Para;
	}	//更新默认参数

	static Velocity_Class Target_Velocity_InAGV;	//目标速度
	static Coordinate_Class Target_Coor_InWorld;	//目标坐标

	

protected:

	virtual float Cal_Displacement(const Coordinate_Class Destination_Coor_InOrigin) = 0;	//根据终点坐标在起点坐标中的坐标计算插补距离
	virtual Velocity_Class& Assign_Velocity(const Coordinate_Class&Destination_Coor_InOrigin, const float velocity) = 0;	//根据终点坐标在起点坐标中的坐标，将合速度分配给各个轴
	virtual float Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin) = 0;	//根据当前坐标计算在源坐标系上的位移

	Coordinate_Class Destination_Coor_InWorld;	//终点坐标
	Coordinate_Class Origin_Coor_InWorld;	//起点坐标
	Coordinate_Class Destination_Coor_InOrigin;	//起点坐标系中的终点坐标

	static Interpolation_Parameter_TypedefStructure Interpolation_Parameter;
	Interpolation_Parameter_TypedefStructure Input_Para;

	bool Is_Linear;	//指示当前插补是直线插补还是圆弧插补(未实现)

	float threshold;	//插补阈值

	//因为同一时间只会执行一条运动指令，故为静态变量
	static int Distance_Symbols; //指示待插补数据的符号

	static float X_H_mul_X;	//用于计算投影向量

	static float acc_distance;	//加速段距离(mm)
	static float const_distance;  //匀速段距离(mm)
	static float dec_distance;	//减速段距离(mm)
	static float slowly_distance; //慢速段距离(mm)

	static float acceleration_time; //加速段时间(s)
	static float const_time;		 //匀速段时间(s)
	static float deceleration_time; //减速段时间(s)
	static float slowly_time;		 //慢速段时间(s)
};


