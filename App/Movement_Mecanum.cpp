#include "Movement_Mecanum.h"

//麦克纳姆轮的实现

#define INTER_FLOAT_DELTA 0.001 //插补用的浮点邻域，若两浮点数差值的绝对值小于该数，则认为两浮点数一致


bool Movement_Mecanum_Class::Init(const Actual_INPUT_TypedefStructure & Input, float threshold, bool Is_Linear)
{
	Destination_Coor_InOrigin = Destination_Coor_InWorld - Origin_Coor_InWorld;	//计算终点在起点坐标系上的位姿

	if (Cal_Displacement(Destination_Coor_InOrigin) < threshold)
	{
		return false;
	}

	float distance_temp = (Input.max_velocity_abs * Input.max_velocity_abs - Input.min_velocity_abs * Input.min_velocity_abs) / Input.acceleration_abs;	//若存在匀速段最小位移,最小位移的距离
	float input_distance = Cal_Displacement(Destination_Coor_InOrigin);	//计算移动距离
	Input_Para = Input;
	Interpolation_OK = false;

	if (ABS(Input.min_velocity_abs) < INTER_FLOAT_DELTA) //最小速度为0
	{
		Input_Para.slow_distance_abs = 0.0f;	//实际最小速度位移为0
	}

	Distance_Symbols = (input_distance > 0.0f ? 1 : -1); //待插补的值的符号

	input_distance *= Distance_Symbols;

	//复位参数值
	acceleration_time = 0.0f;
	const_time = 0.0f;
	deceleration_time = 0.0f;
	slowly_time = 0.0f;

	acc_distance = 0.0f;
	const_distance = 0.0f;
	dec_distance = 0.0f;
	slowly_distance = 0.0f;

	input_distance -= Input_Para.slow_distance_abs; //减去优先满足匀低速段的距离

	if (input_distance < 0)	//只有最低速位移
	{
		slowly_time = (input_distance + Input_Para.slow_distance_abs) / Input_Para.min_velocity_abs;	//计算最低速时间
		Input_Para.max_velocity_abs = Input.min_velocity_abs;	//最大速度为最小速度
	}
	else if (input_distance < distance_temp)	//不存在匀速段
	{
		Input_Para.max_velocity_abs = sqrtf(input_distance * Input.acceleration_abs + Input.min_velocity_abs * Input.min_velocity_abs);
		acceleration_time = (Input_Para.max_velocity_abs - Input_Para.min_velocity_abs) / Input_Para.acceleration_abs;
		deceleration_time = acceleration_time;
	}
	else//存在匀速段
	{
		acceleration_time = deceleration_time = (Input.max_velocity_abs - Input.min_velocity_abs) / Input.acceleration_abs;
		const_time = (input_distance - distance_temp) / Input.max_velocity_abs;
	}

	deceleration_time = acceleration_time = (long)((Input_Para.max_velocity_abs - Input_Para.min_velocity_abs) / Input_Para.acceleration_abs * 100.0f) / 100.0f; //获取加减速时间(ms)，圆整
	Input_Para.max_velocity_abs = Input_Para.min_velocity_abs + acceleration_time * Input_Para.acceleration_abs;	//更新最大速度
	const_time = (long)(const_time*100.0f) / 100.0f;		//圆整匀速时间

	dec_distance = acc_distance = (Input_Para.max_velocity_abs + Input_Para.min_velocity_abs) * acceleration_time / 2.0f;	//计算加减速段位移
	const_distance = Input_Para.max_velocity_abs * const_time;	//计算匀速段位移
	slowly_distance = input_distance + Input_Para.slow_distance_abs - dec_distance - acc_distance - const_distance;	//低速位移
																													//slowly_distance = Input_Para.min_velocity_abs * Result.slowly_time + Input.slow_distance_abs;
	slowly_time = (long)(slowly_distance / Input_Para.min_velocity_abs * 100.0f) / 100.0f; //获取总的慢速时间，圆整

	X_H_mul_X = Destination_Coor_InOrigin.x_coor*Destination_Coor_InOrigin.x_coor \
		+ Destination_Coor_InOrigin.y_coor*Destination_Coor_InOrigin.y_coor \
		+ Destination_Coor_InOrigin.angle_coor*Destination_Coor_InOrigin.angle_coor;

	return true;
}

//************************************
// Method:    Cal_Displacement
// FullName:  Movement_Mecanum_Class::Cal_Displacement
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Destination_Coor_InOrigin
// Description: 根据终点坐标在起点坐标中的坐标计算插补距离
//************************************
float Movement_Mecanum_Class::Cal_Displacement(const Coordinate_Class Destination_Coor_InOrigin)
{
	x_temp_InOrigin = Destination_Coor_InOrigin.x_coor;
	y_temp_InOrigin = Destination_Coor_InOrigin.y_coor;
	angle_equivalent_temp_InOrigin = Destination_Coor_InOrigin.angle_coor / 180.0f*M_PI*(DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2;

	distance_InOrigin = ABS(x_temp_InOrigin) + ABS(y_temp_InOrigin) + ABS(angle_equivalent_temp_InOrigin);

	return distance_InOrigin;
}

//************************************
// Method:    Cal_Velocity
// FullName:  Movement_Mecanum_Class::Cal_Velocity
// Access:    private 
// Returns:   Velocity_Class&
// Parameter: const Coordinate_Class & Destination_Coor_InOrigin
// Parameter: const float velocity
// Description: 根据终点坐标在起点坐标中的坐标，将和速度分配给各个轴
//************************************
Velocity_Class &Movement_Mecanum_Class::Cal_Velocity(const Coordinate_Class & Destination_Coor_InOrigin, const float velocity)
{
	float k = x_temp_InOrigin / distance_InOrigin;
	float x_velocity = k*velocity;
	k = y_temp_InOrigin / distance_InOrigin;
	float y_velocity = k*velocity;
	k = angle_equivalent_temp_InOrigin / distance_InOrigin;
	float angle_equivalent_velocity = k*velocity;

	Target_Velocity_InAGV.velocity_angle = ArcTan_Lookup(x_velocity, y_velocity) / 10.0f;
	x_velocity = ABS(x_velocity);
	y_velocity = ABS(y_velocity);

	Target_Velocity_InAGV.velocity = sqrtf(x_velocity*x_velocity + y_velocity*y_velocity);

	Target_Velocity_InAGV.angular_velocity = angle_equivalent_velocity / M_PI*180.0f / ((DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2);

	return Target_Velocity_InAGV;
}

//************************************
// Method:    Cal_Current_Coor_InOrigin
// FullName:  Movement_Mecanum_Class::Cal_Current_Coor_InOrigin
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Current_Coor_InOrigin
// Description: 根据当前坐标计算移动的距离
//************************************
float Movement_Mecanum_Class::Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin)
{
	float x_temp, y_temp, angle_equivalent_temp, distance_temp;
	x_temp = Current_Coor_InOrigin.x_coor;
	y_temp = Current_Coor_InOrigin.y_coor;
	angle_equivalent_temp = Current_Coor_InOrigin.angle_coor / 180.0f*M_PI*(DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2;


	x_temp = x_temp_InOrigin > 0.0f ? x_temp : -x_temp;
	y_temp = y_temp_InOrigin > 0.0f ? y_temp : -y_temp;
	angle_equivalent_temp = angle_equivalent_temp_InOrigin > 0.0f ? angle_equivalent_temp : -angle_equivalent_temp;


	distance_temp = x_temp + y_temp + angle_equivalent_temp;

	return distance_temp;
}
