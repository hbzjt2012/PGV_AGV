#include "parameter_define.h"
#include "Configure.h"
#include "macros.h"


inline constexpr float Update_wheel_angular_velocity(float motor_rotationl_velocity)
{
	return motor_rotationl_velocity / 60 / REDUCTION_RATIO * 2 * M_PI;
}

inline constexpr float Update_wheel_line_velocity(float motor_rotationl_velocity)
{
	return motor_rotationl_velocity / 60 / REDUCTION_RATIO * MECANUM_WHEEL_DIAMETER * M_PI;
}

inline float Update_wheel_acceleration_line(float acceleration_time)
{
	return (Parameter_Class::wheel_max_line_velocity - Parameter_Class::wheel_min_line_velocity) / acceleration_time;
	//return (MOTOR_MAX_ROTATIONL_VELOCITY_HARD - MOTOR_MIN_ROTATIONL_VELOCITY_HARD) / 60 / REDUCTION_RATIO * MECANUM_WHEEL_DIAMETER * M_PI / acceleration_time;
}

//轮子最终分辨率（线数）
#if ENCODER_FIX_WHEEL
const float Parameter_Class::wheel_resolution = (float)ENCODER_RESOLUTION_INIT;
#else
const float Parameter_Class::wheel_resolution = (float)(ENCODER_RESOLUTION_INIT*REDUCTION_RATIO);
#endif

const float Parameter_Class::wheel_lx_ly_distance = (DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2;	//运动公式中的(lx+ly)

const float Parameter_Class::wheel_max_line_velocity_hard = Update_wheel_line_velocity(MOTOR_MAX_ROTATIONL_VELOCITY_HARD);     //轮子最大线速度(mm/s);
const float Parameter_Class::wheel_min_line_velocity_hard = Update_wheel_line_velocity(MOTOR_MIN_ROTATIONL_VELOCITY_HARD);      //轮子最小线速度(mm/s);

float Parameter_Class::motor_max_rotationl_velocity_soft = 3000.0f;	//软件定义电机最高转速(3000rpm/min)
float Parameter_Class::motor_min_rotationl_velocity_soft = 100.0f;	//软件定义电机最低转速(100rpm/min)

//定义车轮最大速度、最小速度和最大加减速度
float Parameter_Class::wheel_max_line_velocity = Update_wheel_line_velocity(motor_max_rotationl_velocity_soft);     //轮子最大线速度(mm/s);
float Parameter_Class::wheel_min_line_velocity = Update_wheel_line_velocity(motor_min_rotationl_velocity_soft);      //轮子最小线速度(mm/s);

float Parameter_Class::wheel_acceleration_time = 3.0f;//车轮最大加减速所需时间(从最低速到最高速)(单位s)
float Parameter_Class::wheel_acceleration_line_velocity = Update_wheel_acceleration_line(wheel_acceleration_time);		//车轮最大线加减速度(mm/s2)

float Parameter_Class::line_slowest_time = 3.0f;	//最低速移动的时间

bool Parameter_Class::Is_Absolute_Coor = true; //指示当前坐标是否为绝对坐标
unsigned int Parameter_Class::AGV_Address_NUM = 1;

float Parameter_Class::movement_threshold = 3.0f;	//运动阈值(mm)


//void Parameter_Class::Update_Parameter(int num, float para)
//{
//	switch ((Parameter_Class::Parameter_Num) num)
//	{
//	case Parameter_Motor_Max_Rotationl_Velocity:	//电机最大转速
//		motor_max_rotationl_velocity_soft = para;
//		wheel_max_angular_velocity = Update_wheel_angular_velocity(para);	//更新轮子最大角速度(rad/s)
//		wheel_max_line_velocity = Update_wheel_line_velocity(motor_max_rotationl_velocity_soft);     //更新轮子最大线速度(mm/s);
//		break;
//	case	Parameter_Motor_Min_Rotationl_Velocity:	//电机最小转速
//		motor_min_rotationl_velocity_soft = para;
//		wheel_min_angular_velocity = Update_wheel_angular_velocity(para);	//更新轮子最小角速度(rad/s)
//		wheel_min_line_velocity = Update_wheel_line_velocity(motor_min_rotationl_velocity_soft);      //更新轮子最小线速度(mm/s);
//		break;
//	case	Parameter_Wheel_Acceleration_Time://定义车轮的最大加减速时间
//		wheel_acceleration_time = para;
//		wheel_acceleration_line_velocity = Update_wheel_acceleration_line(para);
//		break;
//	case	Parameter_Line_Slowest_Time://最低速移动的时间
//		line_slowest_time = para;
//		break;
//	default:
//		break;
//	}
//}

void Parameter_Class::Init_Parameter(void)
{
	wheel_max_line_velocity = Update_wheel_line_velocity(motor_max_rotationl_velocity_soft);     //轮子最大线速度(mm/s);
	wheel_min_line_velocity = Update_wheel_line_velocity(motor_min_rotationl_velocity_soft);      //轮子最小线速度(mm/s);

	wheel_acceleration_line_velocity = Update_wheel_acceleration_line(wheel_acceleration_time);
}
