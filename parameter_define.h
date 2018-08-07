#pragma once

#include "./DSP_Lib/arm_math.h"
#include "../HardwareDefine/Version_Boards.h"
class Parameter_Class
{
public:
	static const float wheel_lx_ly_distance ;	//运动学公式里的(lx+ly)

	static const float wheel_max_line_velocity_hard;	//硬件定义的轮子最大线速度(mm/s)
	static const float wheel_min_line_velocity_hard ;	//硬件定义的轮子最小线速度(mm/s)

	static float motor_max_rotationl_velocity_soft;	//软件定义电机最高转速
	static float motor_min_rotationl_velocity_soft;	//软件定义电机最低转速

	//定义车轮最大速度、最小速度和最大加减速度
	static float wheel_max_line_velocity;     //轮子最大线速度(mm/s);
	static float wheel_min_line_velocity;      //轮子最小线速度(mm/s);

	static float wheel_acceleration_time;		//定义车轮的最大加减速时间
	static float wheel_acceleration_line_velocity;		//车轮最大线加速度(mm/s2)

	static float line_slowest_time;	//最低速移动的时间

	static const float wheel_resolution;	//车轮分辨率

	static bool Is_Absolute_Coor;	//指示当前坐标是否为绝对坐标
	static unsigned int AGV_Address_NUM;	//AGV的地址码

	static float movement_threshold;	//运动阈值(mm)
	//static float line_threshold;	//直线距离插补阈值(mm)
	//static float rotate_threshold;	//旋转角度插补阈值(mm)(需转化为直线距离)

	enum Parameter_Num
	{
		Parameter_Motor_Max_Rotationl_Velocity = 0,	//电机最大转速
		Parameter_Motor_Min_Rotationl_Velocity,	//电机最小转速
		Parameter_Wheel_Acceleration_Time,//定义车轮的最大加减速时间
		Parameter_Line_Slowest_Time,	//最低速移动的时间
		Parameter_Is_Absolute_Coor,	//指示当前是否是绝对坐标
		Parameter_AGV_Address_Code,	//AGV的地址码
		Parameter_Movement_Threshold	//运动阈值
	};	//定义参数序号

	//void Update_Parameter(int num, float para);
	static void Init_Parameter(void);

};

namespace AGV_State
{
	typedef enum {
		Gcode_Command_IDLE,	//空闲
		Gcode_Command_BUSY,	//缓存区繁忙
		Gcode_Command_OK,		//指令接收正常
		Gcode_Command_ERROR	//指令错误
	} Gcode_Command_State;

	typedef enum
	{
		Movement_Command_IDLE,	//空闲
		Movement_Command_BUSY,	//缓存区繁忙
		Movement_Command_OK		//缓存区正常
	}Movement_Command_State;

}