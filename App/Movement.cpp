#include "Movement.h"

#define INTER_FLOAT_DELTA 0.001 //插补用的浮点邻域，若两浮点数差值的绝对值小于该数，则认为两浮点数一致

//因为同一时间只会执行一条运动指令，故为静态变量
Velocity_Class Movement_Class::Target_Velocity_InAGV;	//目标速度
Coordinate_Class Movement_Class::Target_Coor_InWorld;	//目标坐标

int Movement_Class::Distance_Symbols = 1; //指示待插补数据的符号

float Movement_Class::acc_distance = 0.0f;	//加速段距离(mm)
float Movement_Class::const_distance = 0.0f;  //匀速段距离(mm)
float Movement_Class::dec_distance = 0.0f;	//减速段距离(mm)
float Movement_Class::slowly_distance = 0.0f; //慢速段距离(mm)

float Movement_Class::acceleration_time = 0.0f; //加速段时间(ms)
float Movement_Class::const_time = 0.0f;		 //匀速段时间(ms)
float Movement_Class::deceleration_time = 0.0f; //减速段时间(ms)
float Movement_Class::slowly_time = 0.0f;		 //慢速段时间(ms)


void Movement_Class::Init(const Actual_INPUT_TypedefStructure & Input)
{
	float distance_temp = (Input.max_velocity_abs * Input.max_velocity_abs - Input.min_velocity_abs * Input.min_velocity_abs) / Input.acceleration_abs;	//若存在匀速段最小位移,最小位移的距离
	float input_distance = Cal_Displacement(Origin_Coor_InWorld, Destination_Coor_InWorld);	//计算移动距离
	Actual_INPUT_TypedefStructure Input_Para = Input;

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
	Input_Para.max_velocity_abs = Input_Para.min_velocity_abs +acceleration_time * Input_Para.acceleration_abs;	//更新最大速度
	const_time = (long)(const_time*100.0f) / 100.0f;		//圆整匀速时间

	dec_distance = acc_distance = (Input_Para.max_velocity_abs + Input_Para.min_velocity_abs) * acceleration_time / 2.0f;	//计算加减速段位移
	const_distance = Input_Para.max_velocity_abs * const_time;	//计算匀速段位移
	slowly_distance = input_distance - dec_distance * 2 - const_distance;	//低速位移
																					//slowly_distance = Input_Para.min_velocity_abs * Result.slowly_time + Input.slow_distance_abs;
	slowly_time = (long)(slowly_distance / Input_Para.min_velocity_abs * 100.0f) / 100.0f; //获取总的慢速时间，圆整

}

bool Movement_Class::Get_Expectation(const Coordinate_Class Current_Coor_InWorld)
{
	Target_Coor_InWorld = Current_Coor_InWorld;	//最初的目标坐标
	target_coor = current_coor;
	current_coor *= Distance_Symbols;

	if (current_coor < 0.0f)	//在反方向
	{
		output_velocity = Input_Para.min_velocity_abs * Distance_Symbols;
		target_coor = 0.0f;
	}
	else if (current_coor < acc_distance)//在加速区内
	{
		output_velocity = sqrtf(2 * ABS(current_coor) * Input_Para.acceleration_abs + Input_Para.min_velocity_abs * Input_Para.min_velocity_abs) * Distance_Symbols;
		//target_coor = (current_coor)* Distance_Symbols;
	}
	else if (current_coor < (acc_distance + const_distance))//在匀速区
	{
		output_velocity = Input_Para.max_velocity_abs * Distance_Symbols;
		//target_coor = (current_coor )* Distance_Symbols;
	}
	else if (current_coor < (acc_distance + const_distance + dec_distance))//在减速区
	{
		output_velocity = sqrtf(Input_Para.max_velocity_abs * Input_Para.max_velocity_abs - 2 * ABS(current_coor - acc_distance - const_distance) * Input_Para.acceleration_abs) * Distance_Symbols;
		//target_coor = (current_coor )* Distance_Symbols;
	}
	else if (current_coor < (acc_distance + const_distance + dec_distance + slowly_distance - DISATNCE_DELTA))//在慢速区
	{
		output_velocity = Input_Para.min_velocity_abs * Distance_Symbols;
		//target_coor = (current_coor)* Distance_Symbols;
	}
	else
	{
		output_velocity = 0.0f;
		//target_coor = (current_coor)* Distance_Symbols;
		return false;
	}
	return true;
}
