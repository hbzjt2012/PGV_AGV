#include "Movement_Mecanum.h"

//麦克纳姆轮的实现


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
	float input_distance_abs = Cal_Destination_Displacement(Destination_Coor_InOrigin);	//计算待插补的距离

	if (input_distance_abs < threshold)
	{
		return false;	//起点终点太近，插补失败，返回
	}

	X_H_mul_X = x_temp_InOrigin*x_temp_InOrigin \
		+ y_temp_InOrigin*y_temp_InOrigin\
		+ angle_equivalent_temp_InOrigin*angle_equivalent_temp_InOrigin;

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
// Method:    Cal_Destination_Displacement
// FullName:  Movement_Mecanum_Class::Cal_Destination_Displacement
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Destination_Coor_InOrigin
// Description: 根据终点坐标在起点坐标中的坐标计算插补距离
//************************************
float Movement_Mecanum_Class::Cal_Destination_Displacement(const Coordinate_Class Destination_Coor_InOrigin)
{
	//储存各个轴在终点坐标系上的距离

	x_temp_InOrigin = Destination_Coor_InOrigin.x_coor;
	y_temp_InOrigin = Destination_Coor_InOrigin.y_coor;
	angle_equivalent_temp_InOrigin = Destination_Coor_InOrigin.angle_rad *Parameter_Class::wheel_lx_ly_distance;

	distance_InOrigin_ABS = ABS(x_temp_InOrigin) + ABS(y_temp_InOrigin) + ABS(angle_equivalent_temp_InOrigin);

	return distance_InOrigin_ABS;
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
	float k = 0.0f;

	k = x_temp_InOrigin / distance_InOrigin_ABS;
	Target_Velocity_InAGV.velocity_x = k*velocity;
	
	k = y_temp_InOrigin / distance_InOrigin_ABS;
	Target_Velocity_InAGV.velocity_y = k*velocity;
	
	k = angle_equivalent_temp_InOrigin / distance_InOrigin_ABS;
	Target_Velocity_InAGV.angular_velocity_mm = k*velocity;

	Target_Velocity_InAGV.velocity_angle = ArcTan_Lookup(Target_Velocity_InAGV.velocity_x, Target_Velocity_InAGV.velocity_y) / 10.0f;

	Target_Velocity_InAGV.velocity = sqrtf(Target_Velocity_InAGV.velocity_x*Target_Velocity_InAGV.velocity_x + Target_Velocity_InAGV.velocity_y*Target_Velocity_InAGV.velocity_y);

	Target_Velocity_InAGV.angular_velocity_rad = Target_Velocity_InAGV.angular_velocity_mm / (Parameter_Class::wheel_lx_ly_distance);
	Target_Velocity_InAGV.angular_velocity_angle = Target_Velocity_InAGV.angular_velocity_rad / M_PI * 180;

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
	float angle_equivalent_temp, distance_temp;
	angle_equivalent_temp = Current_Coor_InOrigin.angle_rad*Parameter_Class::wheel_lx_ly_distance;

	distance_temp = ABS(Current_Coor_InOrigin.x_coor) + ABS(Current_Coor_InOrigin.y_coor) + ABS(angle_equivalent_temp);

	return distance_temp;
}
