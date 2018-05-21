#include "Movement_Mecanum.h"

//麦克纳姆轮的实现

//#define INTER_FLOAT_DELTA 0.001 //插补用的浮点邻域，若两浮点数差值的绝对值小于该数，则认为两浮点数一致

//************************************
// Method:    Init
// FullName:  Movement_Mecanum_Class::Init
// Access:    public 
// Returns:   bool 插补结果，返回false表示起点、终点太接近
// Parameter: Interpolation_Parameter_TypedefStructure Input 插补参数
// Parameter: const float threshold	阈值，若起点、终点距离值小于该阈值，则插补失败（无需插补）
// Description: 根据插补参数以及阈值插补运动路径结果
//************************************
bool Movement_Mecanum_Class::Init(Interpolation_Parameter_TypedefStructure Input, const Coordinate_Class &Current_Coor, const float threshold)
{
	Origin_Coor_InWorld = Current_Coor;
	Destination_Coor_InOrigin = Destination_Coor_InWorld - Origin_Coor_InWorld;	//计算终点在起点坐标系上的位姿

	float slowly_distance_temp = Input.min_velocity_abs*Input.slow_time_abs;	//低速段移动距离暂存
	float input_distance_abs = Cal_Displacement(Destination_Coor_InOrigin);	//计算待插补的距离

	if (input_distance_abs < threshold)
	{
		return false;	//起点终点太近，插补失败，返回
	}

	if (Is_X_Y)
	{
		X_H_mul_X = Destination_Coor_InOrigin.x_coor*Destination_Coor_InOrigin.x_coor \
			+ Destination_Coor_InOrigin.y_coor*Destination_Coor_InOrigin.y_coor;
	}
	else
	{
		X_H_mul_X = Destination_Coor_InOrigin.angle_coor*Destination_Coor_InOrigin.angle_coor;
	}

	//加减速段的最大位移，用于判断是否存在匀速段
	float distance_acc_dec_temp = (Input.max_velocity_abs * Input.max_velocity_abs - Input.min_velocity_abs * Input.min_velocity_abs) / Input.acceleration_abs;

	Distance_Symbols = (input_distance_abs > 0.0f ? 1 : -1); //待插补的值的符号

	input_distance_abs *= Distance_Symbols;	//移动距离的绝对值

	//复位参数值
	acceleration_time = 0.0f;
	const_time = 0.0f;
	deceleration_time = 0.0f;
	slowly_time = 0.0f;

	acc_distance = 0.0f;
	const_distance = 0.0f;
	dec_distance = 0.0f;
	slowly_distance = 0.0f;

	input_distance_abs -= slowly_distance_temp;	//先减去慢速段的距离


	if (input_distance_abs < 0)	//无法满足慢速段的距离（即只有慢速段）
	{
		slowly_distance = input_distance_abs + slowly_distance_temp;	//慢速段移动距离(mm)
		slowly_time = slowly_distance / Input.min_velocity_abs;	//慢速段时间(s)
		Input_Para = Input;	//保存插补参数
		return true;
	}
	else if (input_distance_abs < distance_acc_dec_temp)	//不存在匀速段
	{
		//计算加减速时间(s)
		Input.max_velocity_abs = sqrtf(input_distance_abs * Input.acceleration_abs + Input.min_velocity_abs * Input.min_velocity_abs);	//更新最大速度(mm/s)
		acceleration_time = (Input.max_velocity_abs - Input.min_velocity_abs) / Input.acceleration_abs;	//计算加减速时间(s)
		deceleration_time = acceleration_time;
	}
	else//存在匀速段
	{
		acceleration_time = deceleration_time = (Input.max_velocity_abs - Input.min_velocity_abs) / Input.acceleration_abs;	//计算加减速时间(s)
		const_time = (input_distance_abs - distance_acc_dec_temp) / Input.max_velocity_abs;//匀速段时间(s)
	}

	//根据上述插补结果，圆整加减速、匀速时间，计算相应阶段位移


	deceleration_time = acceleration_time = (unsigned long)((Input.max_velocity_abs - Input.min_velocity_abs) / Input.acceleration_abs * 1000.0f) / 1000.0f; //获取加减速时间(s)，圆整
	Input.max_velocity_abs = Input.min_velocity_abs + acceleration_time * Input.acceleration_abs;	//更新最大速度(mm/s)
	const_time = (unsigned long)(const_time*1000.0f) / 1000.0f;		//圆整匀速时间(s)

	dec_distance = acc_distance = (Input.max_velocity_abs + Input.min_velocity_abs) * acceleration_time / 2.0f;	//计算加减速段位移
	const_distance = Input.max_velocity_abs * const_time;	//计算匀速段位移

	slowly_distance = input_distance_abs + slowly_distance_temp - dec_distance - acc_distance - const_distance;	//低速位移
	slowly_time = slowly_distance / Input.min_velocity_abs; //获取总的慢速时间(s)

	Input_Para = Input;	//保存插补参数

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
	//储存各个轴在终点坐标系上的距离

	//x_temp_InOrigin = Destination_Coor_InOrigin.x_coor;
	//y_temp_InOrigin = Destination_Coor_InOrigin.y_coor;
	//angle_equivalent_temp_InOrigin = Destination_Coor_InOrigin.angle_coor / 180.0f*M_PI*Parameter_Class::wheel_lx_ly_distance;

	if (Is_X_Y)
	{
		x_temp_InOrigin = Destination_Coor_InOrigin.x_coor;
		y_temp_InOrigin = Destination_Coor_InOrigin.y_coor;
		angle_equivalent_temp_InOrigin = 0.0f;
	}
	else
	{
		x_temp_InOrigin = 0.0f;
		y_temp_InOrigin = 0.0f;
		angle_equivalent_temp_InOrigin = Destination_Coor_InOrigin.angle_coor / 180.0f*M_PI*Parameter_Class::wheel_lx_ly_distance;
	}

	distance_InOrigin = ABS(x_temp_InOrigin) + ABS(y_temp_InOrigin) + ABS(angle_equivalent_temp_InOrigin);

	return distance_InOrigin;
}

//************************************
// Method:    Assign_Velocity
// FullName:  Movement_Mecanum_Class::Assign_Velocity
// Access:    private 
// Returns:   Velocity_Class &
// Parameter: const Coordinate_Class & Destination_Coor_InOrigin
// Parameter: const float velocity
// Description: 根据起点在终点坐标系下的坐标，将速度分配个各个轴
//************************************
Velocity_Class & Movement_Mecanum_Class::Assign_Velocity(const Coordinate_Class & Destination_Coor_InOrigin, const float velocity)
{
	//存在分配问题
	//float k = 0.0f;

	//k = x_temp_InOrigin / distance_InOrigin;
	//float x_velocity = k*velocity;
	//k = y_temp_InOrigin / distance_InOrigin;
	//float y_velocity = k*velocity;
	//k = angle_equivalent_temp_InOrigin / distance_InOrigin;
	//float angle_equivalent_velocity = k*velocity;

	//Target_Velocity_InAGV.velocity_angle = ArcTan_Lookup(x_velocity, y_velocity) / 10.0f;
	//x_velocity = ABS(x_velocity);
	//y_velocity = ABS(y_velocity);

	//Target_Velocity_InAGV.velocity = sqrtf(x_velocity*x_velocity + y_velocity*y_velocity);

	//Target_Velocity_InAGV.angular_velocity = angle_equivalent_velocity / (Parameter_Class::wheel_lx_ly_distance);

	if (Is_X_Y)	//当前在插补XY平面
	{
		Target_Velocity_InAGV.velocity = velocity;
		Target_Velocity_InAGV.velocity_angle = ArcTan_Lookup(x_temp_InOrigin, y_temp_InOrigin) / 10.0f;
		Target_Velocity_InAGV.angular_velocity = 0.0f;
	}
	else
	{
		Target_Velocity_InAGV.velocity = 0.0f;
		Target_Velocity_InAGV.velocity_angle = 0.0f;
		Target_Velocity_InAGV.angular_velocity = velocity / (Parameter_Class::wheel_lx_ly_distance);
		if (Destination_Coor_InOrigin.angle_coor<0.0f)
		{
			Target_Velocity_InAGV.angular_velocity *= -1;
		}
	}

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
	angle_equivalent_temp = Current_Coor_InOrigin.angle_coor / 180 * M_PI *Parameter_Class::wheel_lx_ly_distance;

	if (Is_X_Y)
	{
		distance_temp = ABS(x_temp) + ABS(y_temp);
	}
	else
	{
		distance_temp = ABS(angle_equivalent_temp);
	}

	distance_temp = ABS(x_temp) + ABS(y_temp) + ABS(angle_equivalent_temp);

	return distance_temp;
}
