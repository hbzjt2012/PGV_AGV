#include "Movement.h"
#include "../Math/MyMath.h"

#define DISATNCE_DELTA 0.3f		//当实际总位移与理论总位移差距0.3f时，认为插补已完成
#define INTER_FLOAT_DELTA 0.001 //插补用的浮点邻域，若两浮点数差值的绝对值小于该数，则认为两浮点数一致

//因为同一时间只会执行一条运动指令，故为静态变量
Velocity_Class Movement_Class::Target_Velocity_InAGV;	//目标速度
Coordinate_Class Movement_Class::Target_Coor_InWorld;	//目标坐标

float Movement_Class::X_H_mul_X = 0.0f;

int Movement_Class::Distance_Symbols = 1; //指示待插补数据的符号

float Movement_Class::acc_distance = 0.0f;	//加速段距离(mm)
float Movement_Class::const_distance = 0.0f;  //匀速段距离(mm)
float Movement_Class::dec_distance = 0.0f;	//减速段距离(mm)
float Movement_Class::slowly_distance = 0.0f; //慢速段距离(mm)

float Movement_Class::acceleration_time = 0.0f; //加速段时间(ms)
float Movement_Class::const_time = 0.0f;		 //匀速段时间(ms)
float Movement_Class::deceleration_time = 0.0f; //减速段时间(ms)
float Movement_Class::slowly_time = 0.0f;		 //慢速段时间(ms)


void Movement_Class::Init(const Actual_INPUT_TypedefStructure & Input, bool Is_Linear)
{
	Destination_Coor_InOrigin = Destination_Coor_InWorld - Origin_Coor_InWorld;	//计算终点在起点坐标系上的位姿
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


}

//************************************
// Method:    Get_Expectation
// FullName:  Movement_Class::Get_Expectation
// Access:    public 
// Returns:   bool 若插补完成，返回false
// Parameter: const Coordinate_Class Current_Coor_InWorld 当前坐标
// Description: 根据当前坐标，计算期望速度，期望坐标
//************************************
bool Movement_Class::Get_Expectation(const Coordinate_Class Current_Coor_InWorld)
{
	Coordinate_Class Current_Coor_InOrigin = Current_Coor_InWorld - Origin_Coor_InWorld;	//获取当前坐标在起点坐标系中的坐标
	Coordinate_Class Target_Coor_InOrigin;	//当前坐标向量在终点坐标向量上的投影（即目标坐标）

	float X_H_mul_y = Destination_Coor_InOrigin.x_coor*Current_Coor_InOrigin.x_coor \
		+ Destination_Coor_InOrigin.y_coor*Current_Coor_InOrigin.y_coor \
		+ Destination_Coor_InOrigin.angle_coor*Current_Coor_InOrigin.angle_coor;

	float k = X_H_mul_y / X_H_mul_X;

	//获取当前向量在终点向量上的投影向量
	Target_Coor_InOrigin.x_coor = k*Destination_Coor_InOrigin.x_coor;
	Target_Coor_InOrigin.y_coor = k*Destination_Coor_InOrigin.y_coor;
	Target_Coor_InOrigin.angle_coor = k*Destination_Coor_InOrigin.angle_coor;



	////根据当前坐标求垂直轨迹的目标坐标
	//MyMath::Coor coor_temp1, coor_temp2;

	//coor_temp1.x = Current_Coor_InOrigin.x_coor;
	//coor_temp1.y = Current_Coor_InOrigin.y_coor;

	////获取斜率不存在的交点
	//if (ABS(Destination_Coor_InOrigin.x_coor) < FLOAT_DELTA)
	//{
	//	coor_temp2.y = coor_temp1.y;
	//	coor_temp2.x = Destination_Coor_InOrigin.x_coor;
	//}
	//else
	//{
	//	MyMath::Get_Vertical_Line_Crossover_Point(Destination_Coor_InOrigin.y_coor / Destination_Coor_InOrigin.x_coor, coor_temp1, coor_temp2);
	//}

	//Target_Coor_InOrigin.x_coor = coor_temp2.x;
	//Target_Coor_InOrigin.y_coor = coor_temp2.y;


	//Target_Coor_InOrigin.angle_coor = Current_Coor_InOrigin.angle_coor;


	float current_coor = Cal_Current_Coor_InOrigin(Target_Coor_InOrigin)*Distance_Symbols;	//获取在源坐标系上的位移
	float output_velocity = 0.0f;

	Interpolation_OK = false;

	//获取插补速度
	if (current_coor < 0.0f)	//在反方向
	{
		output_velocity = Input_Para.min_velocity_abs * Distance_Symbols;
		Target_Coor_InOrigin.Clear();
	}
	else if (current_coor < acc_distance)//在加速区内
	{
		output_velocity = sqrtf(2 * current_coor * Input_Para.acceleration_abs + Input_Para.min_velocity_abs * Input_Para.min_velocity_abs) * Distance_Symbols;
	}
	else if (current_coor < (acc_distance + const_distance))//在匀速区
	{
		output_velocity = Input_Para.max_velocity_abs * Distance_Symbols;
	}
	else if (current_coor < (acc_distance + const_distance + dec_distance))//在减速区
	{
		output_velocity = sqrtf(Input_Para.max_velocity_abs * Input_Para.max_velocity_abs - 2 * (current_coor - acc_distance - const_distance) * Input_Para.acceleration_abs) * Distance_Symbols;
	}
	else if (current_coor < (acc_distance + const_distance + dec_distance + slowly_distance - DISATNCE_DELTA))//在慢速区
	{
		output_velocity = Input_Para.min_velocity_abs * Distance_Symbols;
	}
	else
	{
		output_velocity = 0.0f;
		Interpolation_OK = true;
	}

	//计算期望坐标在世界坐标系上的坐标
	Target_Coor_InWorld = Origin_Coor_InWorld + Target_Coor_InOrigin;
	//计算AGV坐标系中的期望速度
	Target_Velocity_InAGV = Cal_Velocity(Destination_Coor_InOrigin, output_velocity);

	return !Interpolation_OK;	//返回插补结果
}
