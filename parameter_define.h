#pragma once

class Parameter_Class
{
public:
	//定义车体参数
#define DISTANCE_OF_WHEEL_X_AXES 534.0f	//左右两轮在X轴上的距离(mm)
#define DISTANCE_OF_WHEEL_Y_AXES 534.0f	//前后两轮在Y轴上的距离(mm)
#define MECANUM_WHEEL_DIAMETER	127.0		//定义麦克纳姆轮直径(mm)

	//定义了编码器参数
#define ENCODER_FIX_WHEEL true       //指示编码器和轮子固连在一起
#define REDUCTION_RATIO 30           //减速比
#define ENCODER_RESOLUTION_INIT 2000 //编码器的分辨率（线数）

	//定义电机转速
#define MOTOR_MAX_ROTATIONL_VELOCITY_HARD 3000.0f	//硬件定义的电机最高转速
#define MOTOR_MIN_ROTATIONL_VELOCITY_HARD 0.0f		//硬件定义的电机最低转速

	static const float wheel_lx_ly_distance;	//运动学公式里的(lx+ly)

	static const float wheel_max_line_velocity_hard;	//硬件定义的轮子最大线速度(mm/s)
	static const float wheel_min_line_velocity_hard;	//硬件定义的轮子最小线速度(mm/s)

	static float motor_max_rotationl_velocity_soft;	//软件定义电机最高转速
	static float motor_min_rotationl_velocity_soft;	//软件定义电机最低转速

	//定义车轮最大速度、最小速度和最大加减速度
	static float wheel_max_angular_velocity;	//轮子最大角速度(rad/s)
	static float wheel_min_angular_velocity;	//轮子最小角速度(rad/s)

	static float wheel_max_line_velocity;     //轮子最大线速度(mm/s);
	static float wheel_min_line_velocity;      //轮子最小线速度(mm/s);

	static float wheel_acceleration_time;		//定义车轮的最大加减速时间
	static float wheel_acceleration_line_velocity;		//车轮最大线加速度(mm/s2)

	static float line_slowest_time;	//最低速移动的时间

	static const float wheel_resolution;	//车轮分辨率

	static bool Is_Absolute_Coor;	//指示当前坐标是否为绝对坐标
	static int AGV_Address_Code;	//AGV的地址码

	static float line_threshold;	//直线距离插补阈值(mm)
	static float rotate_threshold;	//旋转角度插补阈值(mm)(需转化为直线距离)

	enum Parameter_Num
	{
		Parameter_Motor_Max_Rotationl_Velocity = 0,	//电机最大转速
		Parameter_Motor_Min_Rotationl_Velocity,	//电机最小转速
		Parameter_Wheel_Acceleration_Time,//定义车轮的最大加减速时间
		Parameter_Line_Slowest_Time,	//最低速移动的时间
		Parameter_Is_Absolute_Coor,	//指示当前是否是绝对坐标
		Parameter_AGV_Address_Code	//AGV的地址码
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