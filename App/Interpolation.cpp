#include "interpolation.h"
#include "math.h"

#define DISATNCE_DELTA 0.3f		//当实际总位移与理论总位移差距0.3f时，认为插补已完成
#define INTER_FLOAT_DELTA 0.001 //插补用的浮点邻域，若两浮点数差值的绝对值小于该数，则认为两浮点数一致

typedef struct
{
	float acceleration_time; //加速段时间(ms)
	float const_time;		 //匀速段时间(ms)
	float deceleration_time; //减速段时间(ms)
	float slowly_time;		 //慢速段时间(ms)
} Interpolation_Result_TypedefStructure;

//typedef enum
//{
//	Interpolation_Running,	//插补中
//	Interpolation_Over	//插补完毕
//}Interpolation_State_TypedefEnum;

Interpolation::Actual_INPUT_TypedefStructure Input_Para; //插补的运行参数
Interpolation_Result_TypedefStructure Result;			 //插补结果
//Interpolation_State_TypedefEnum State = Interpolation_State_TypedefEnum::Interpolation_Running;	//插补状态

//static float expectation_output;	//插补输出结果
static int Distance_Symbols; //指示待插补数据的符号

static float acc_distance = 0.0f;	//加速段距离(mm)
static float const_distance = 0.0f;  //匀速段距离(mm)
static float dec_distance = 0.0f;	//减速段距离(mm)
static float slowly_distance = 0.0f; //慢速段距离(mm)

//************************************
// Method:    Init
// FullName:  Interpolation::Init
// Access:    public
// Returns:   void
// Parameter: Actual_INPUT_TypedefStructure & Input
// Description: 对输入的位移进行插补，规划理想路径
//************************************
void Interpolation::Init(Actual_INPUT_TypedefStructure &Input)
{
	//若存在匀速段最小位移,最小位移的距离
	float distance_temp = (Input.max_velocity_abs * Input.max_velocity_abs - Input.min_velocity_abs * Input.min_velocity_abs) / Input.acceleration_abs;

	if (Input.min_velocity_abs < INTER_FLOAT_DELTA) //最小速度不为0
	{
		Input.slow_distance_abs = 0.0f;
	}

	Distance_Symbols = (Input.displacement > 0.0f ? 1 : -1); //待插补的值的符号
	Input.displacement *= Distance_Symbols;
	Input_Para = Input;

	Result.acceleration_time = 0.0f;
	Result.const_time = 0.0f;
	Result.deceleration_time = 0.0f;
	Result.slowly_time = 0.0f;

	acc_distance = 0.0f;
	const_distance = 0.0f;
	dec_distance = 0.0f;
	slowly_distance = 0.0f;

	Input.displacement -= Input.slow_distance_abs; //减去优先满足匀低速段的距离

	if (Input.displacement < 0)	//只有最低速位移
	{
		Result.slowly_time = Input_Para.slow_distance_abs / Input_Para.min_velocity_abs;	//计算最低速时间
		Input_Para.max_velocity_abs = Input.min_velocity_abs;	//最大速度为最小速度
	}
	else if (Input.displacement < distance_temp)	//不存在匀速段
	{
		Input_Para.max_velocity_abs = sqrtf(Input.displacement * Input.acceleration_abs + Input.min_velocity_abs * Input.min_velocity_abs);
		Result.acceleration_time = (Input_Para.max_velocity_abs - Input_Para.min_velocity_abs) / Input_Para.acceleration_abs;
		Result.deceleration_time = Result.acceleration_time;
	}
	else//存在匀速段
	{
		Result.acceleration_time = Result.deceleration_time = (Input.max_velocity_abs - Input.min_velocity_abs) / Input.acceleration_abs;
		Result.const_time = (Input.displacement - distance_temp) / Input.max_velocity_abs;
	}

	Result.deceleration_time = Result.acceleration_time = (long)((Input_Para.max_velocity_abs - Input_Para.min_velocity_abs) / Input_Para.acceleration_abs * 100.0f) / 100.0f; //获取加减速时间(ms)，圆整
	Input_Para.max_velocity_abs = Input_Para.min_velocity_abs + Result.acceleration_time * Input_Para.acceleration_abs;	//更新最大速度
	Result.const_time = (long)(Result.const_time*100.0f) / 100.0f;		//圆整匀速时间

	dec_distance = acc_distance = (Input_Para.max_velocity_abs + Input_Para.min_velocity_abs) * Result.acceleration_time / 2.0f;	//计算加减速段位移
	const_distance = Input_Para.max_velocity_abs * Result.const_time;	//计算匀速段位移
	slowly_distance = Input_Para.displacement - dec_distance * 2 - const_distance;	//低速位移
	//slowly_distance = Input_Para.min_velocity_abs * Result.slowly_time + Input.slow_distance_abs;
	Result.slowly_time = (long)(slowly_distance / Input_Para.min_velocity_abs * 100.0f) / 100.0f; //获取总的慢速时间，圆整

}

//************************************
// Method:    Get_Expectation
// FullName:  Interpolation::Get_Expectation
// Access:    public
// Returns:   bool 若插补完成，则返回flase
// Parameter: float & output_velocity 插补得出的理论速度 mm/ms
// Parameter: float current_coor 计算理论速度和位移所需的当前位移 mm
// Parameter: float &target_coor 下一个目标位移 mm
// Description: 根据当前位移，计算理论速度和位移,当实际总位移和理论位移差距小于阈值时，认为插补完成
//************************************
bool Interpolation::Get_Expectation(float &output_velocity, float current_coor, float &target_coor)
{
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


	//if (current_coor < 0.0f)	//在反方向
	//{
	//	output_velocity = Input_Para.min_velocity_abs * Distance_Symbols;
	//	target_coor = 0.0f;
	//	return true;
	//}

	//if (current_coor < acc_distance) //在加速区内
	//{
	//	output_velocity = sqrtf(2 * ABS(current_coor) * Input_Para.acceleration_abs + Input_Para.min_velocity_abs * Input_Para.min_velocity_abs) * Distance_Symbols;
	//	target_coor = (current_coor)* Distance_Symbols;
	//	return true;
	//}
	//else
	//	current_coor -= acc_distance;

	//if (current_coor < const_distance) //在匀速区
	//{
	//	output_velocity = Input_Para.max_velocity_abs * Distance_Symbols;
	//	target_coor = (current_coor)* Distance_Symbols;
	//	return true;
	//}
	//else
	//	current_coor -= const_distance;

	//if (current_coor < dec_distance) //在减速区
	//{
	//	output_velocity = sqrtf(Input_Para.max_velocity_abs * Input_Para.max_velocity_abs - 2 * ABS(current_coor) * Input_Para.acceleration_abs) * Distance_Symbols;
	//	target_coor = (current_coor)* Distance_Symbols;
	//	return true;
	//}
	//else
	//	current_coor -= dec_distance;

	//if (current_coor < (slowly_distance - DISATNCE_DELTA)) //在慢速区
	//{
	//	output_velocity = Input_Para.min_velocity_abs * Distance_Symbols;
	//	target_coor = current_coor* Distance_Symbols;
	//	return true;
	//}
	//else
	//{
	//	output_velocity = 0.0f;
	//	target_coor = current_coor* Distance_Symbols;
	//	return false; //插补完成
	//}
}
