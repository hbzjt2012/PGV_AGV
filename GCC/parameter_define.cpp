#include "Configure.h"

namespace Parameter_Define
{
	const float distance_of_wheel_x_axes = 534.0f;	//左右两轮在X轴上的距离为534mm
	const float distance_of_wheel_y_axes = 534.0f;	//前后两轮在Y轴上的距离为534mm
	const float Mecanum_wheel_diameter = 127.0f;	//定义麦克纳姆轮直径(mm)

	//轮子最终分辨率（线数）
#if ENCODER_FIX_WHEEL
	const float encoder_resolution = (float)ENCODER_RESOLUTION_INIT;
#else
	const float encoder_resolution = (float)(ENCODER_RESOLUTION_INIT*REDUCTION_RATIO);
#endif

	float motor_max_rotationl_velocity = 3000.0f;	//电机最高转速3000rpm/min
	float motor_min_rotationl_velocity = 100.0f;	//电机最低转速100rpm/min

	float wheel_max_angular_velocity = motor_max_rotationl_velocity * 6 / REDUCTION_RATIO;
	float wheel_min_angular_velocity = motor_min_rotationl_velocity * 6 / REDUCTION_RATIO;

	float wheel_max_line_velocity = motor_max_rotationl_velocity / 60 / REDUCTION_RATIO * Mecanum_wheel_diameter * M_PI;      //轮子最大线速度(mm/s);
	float wheel_min_line_velocity = motor_min_rotationl_velocity / 60 / REDUCTION_RATIO * Mecanum_wheel_diameter * M_PI;      //轮子最小线速度(mm/s);

	float wheel_acceleration_time = 5.0f;//车轮最大加减速所需时间(从最低速到最高速)(单位s)
	float wheel_acceleration_line = (wheel_max_line_velocity - wheel_min_line_velocity) / wheel_acceleration_time;	//车轮最大线加速度(mm/s2)

	float line_slowest_time = 20.0f;	//最低速移动的时间

}