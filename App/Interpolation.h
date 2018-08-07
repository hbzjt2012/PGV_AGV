#pragma once

/*
* 路径插补算法
* 采用梯形插补
* 分为四段（考虑到电机存在最小速度死区）
*	匀加速段ta
*	匀速段tc
*	匀减速段td
*	匀低速段ts
*/

//插补时会先记录待插补距离的符号
#include "../macros.h"

class Interpolation_Class
{
public:
	Interpolation_Class() = default;
	~Interpolation_Class() = default;

	typedef struct
	{
		float max_velocity_abs; //最大速度(mm/s)
		float min_velocity_abs; //最小速度(mm/s)
		float acceleration_abs; //加速度(mm/s2)
		float slow_time_abs;	//最小速度移动时间(s)
	} Interpolation_Parameter_TypedefStructure;	//插补参数结构体定义

	enum Interpolation_State_Enum
	{
		NO_Interpolation,  //未插补
		IS_Interpolating, //正在插补
		IS_Interpolated   //插补完毕
	};	//插补状态

	enum Interpolation_State_Enum Interpolation_State;

	//使用前需更新插补参数和插补阈值
	bool Init(const float distance) { return Init(distance, Interpolation_Class::threshold); }	//根据插补参数和插补距离插补路径
	bool Init(const float distance, const float threshold);	//根据待插补距离和插补阈值对路径插补
	bool Cal_Velocity(float current_distance);	//根据当前移动距离计算期望距离和期望速度，返回计算结果

	void Update_Interpolation_Parameter(const Interpolation_Parameter_TypedefStructure& Input_Para) {
		Interpolation_Parameter = Input_Para;
	}	//更新插补参数

	static float target_distance;
	static float target_velocity;

	static Interpolation_Parameter_TypedefStructure Interpolation_Parameter;	//插补参数、参数暂存
	static float threshold;	//插补阈值

private:
	static float distance;
	//因为同一时间只会执行一条运动指令，故为静态变量
	static int Distance_Symbols; //指示待插补距离的符号

	static float acc_distance;	//加速段距离(mm)
	static float const_distance;  //匀速段距离(mm)
	static float dec_distance;	//减速段距离(mm)
	static float slowly_distance; //慢速段距离(mm)

	static float acceleration_time; //加速段时间(s)
	static float const_time;		 //匀速段时间(s)
	static float deceleration_time; //减速段时间(s)
	static float slowly_time;		 //慢速段时间(s)
};
