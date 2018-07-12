#include "Interpolation.h"
#include "../DSP_Lib/arm_math.h"

Interpolation_Class::Interpolation_Parameter_TypedefStructure Interpolation_Class::Interpolation_Parameter;

enum Interpolation_Class::Interpolation_State_Enum Interpolation_Class::Interpolation_State;

float Interpolation_Class::target_distance = 0.0f;
float Interpolation_Class::target_velocity = 0.0f;
float Interpolation_Class::distance = 0.0f;

float Interpolation_Class::threshold = 0.0f;	//插补阈值

//因为同一时间只会执行一条运动指令，故为静态变量
int Interpolation_Class::Distance_Symbols = 1; //指示待插补距离的符号

float Interpolation_Class::acc_distance = 0.0f;	//加速段距离(mm)
float Interpolation_Class::const_distance = 0.0f;  //匀速段距离(mm)
float Interpolation_Class::dec_distance = 0.0f;	//减速段距离(mm)
float Interpolation_Class::slowly_distance = 0.0f; //慢速段距离(mm)

float Interpolation_Class::acceleration_time = 0.0f; //加速段时间(s)
float Interpolation_Class::const_time = 0.0f;		 //匀速段时间(s)
float Interpolation_Class::deceleration_time = 0.0f; //减速段时间(s)
float Interpolation_Class::slowly_time = 0.0f;		 //慢速段时间(s)


bool Interpolation_Class::Init(const float distance, const float threshold)
{
	if (ABS(distance) < threshold)
	{
		return false;	//待插补距离小于阈值，插补直接结束
	}

	Interpolation_Class::threshold = threshold;

	float slowly_distance_temp = \
		Interpolation_Parameter.min_velocity_abs*Interpolation_Parameter.slow_time_abs;	//计算最低速移动距离
	//加减速段的最大位移，用于判断是否存在匀速段
	float distance_acc_dec_temp = \
		(Interpolation_Parameter.max_velocity_abs * Interpolation_Parameter.max_velocity_abs \
			- Interpolation_Parameter.min_velocity_abs * Interpolation_Parameter.min_velocity_abs) \
		/ Interpolation_Parameter.acceleration_abs;

	Distance_Symbols = (distance > 0.0f ? 1 : -1); //待插补的值的符号
	Interpolation_Class::distance = distance;
	float input_distance_abs = distance;
	input_distance_abs *= Distance_Symbols;	//移动距离的绝对值

	//复位目标距离和速度
	target_velocity = target_distance = 0.0f;

	//复位参数值
	acceleration_time = const_time = deceleration_time = slowly_time = 0.0f;
	acc_distance = const_distance = dec_distance = slowly_distance = 0.0f;

	input_distance_abs -= slowly_distance_temp;	//先减去慢速段的距离

	if (input_distance_abs < 0)	//无法满足慢速段的距离（即只有慢速段）
	{
		slowly_distance = input_distance_abs + slowly_distance_temp;	//慢速段移动距离(mm)
		slowly_time = slowly_distance / Interpolation_Parameter.min_velocity_abs;	//慢速段时间(s)
		Interpolation_Parameter.max_velocity_abs = Interpolation_Parameter.min_velocity_abs;	//只有最低速
		return true;
	}
	else if (input_distance_abs < distance_acc_dec_temp)	//不存在匀速段
	{
		//计算加减速时间(s)
		Interpolation_Parameter.max_velocity_abs = \
			sqrtf(input_distance_abs * Interpolation_Parameter.acceleration_abs \
				+ Interpolation_Parameter.min_velocity_abs *Interpolation_Parameter.min_velocity_abs);	//更新最大速度(mm/s)
		acceleration_time = (Interpolation_Parameter.max_velocity_abs - Interpolation_Parameter.min_velocity_abs)\
			/ Interpolation_Parameter.acceleration_abs;	//计算加减速时间(s)
		deceleration_time = acceleration_time;
	}
	else//存在匀速段
	{
		acceleration_time = deceleration_time = \
			(Interpolation_Parameter.max_velocity_abs - Interpolation_Parameter.min_velocity_abs) \
			/ Interpolation_Parameter.acceleration_abs;	//计算加减速时间(s)
		const_time = (input_distance_abs - distance_acc_dec_temp) / Interpolation_Parameter.max_velocity_abs;//匀速段时间(s)
	}

	//根据上述插补结果，圆整加减速、匀速时间，计算相应阶段位移

	deceleration_time = acceleration_time = \
		(unsigned long)((Interpolation_Parameter.max_velocity_abs - Interpolation_Parameter.min_velocity_abs)\
			/ Interpolation_Parameter.acceleration_abs * 1000.0f) / 1000.0f; //获取加减速时间(s)，圆整

	Interpolation_Parameter.max_velocity_abs = \
		Interpolation_Parameter.min_velocity_abs + acceleration_time * Interpolation_Parameter.acceleration_abs;	//更新最大速度(mm/s)

	const_time = (unsigned long)(const_time*1000.0f) / 1000.0f;		//圆整匀速时间(s)

	dec_distance = acc_distance = \
		(Interpolation_Parameter.max_velocity_abs + Interpolation_Parameter.min_velocity_abs) * acceleration_time / 2.0f;	//计算加减速段位移

	const_distance = Interpolation_Parameter.max_velocity_abs * const_time;	//计算匀速段位移

	slowly_distance = input_distance_abs + slowly_distance_temp - dec_distance - acc_distance - const_distance;	//低速位移
	slowly_time = slowly_distance / Interpolation_Parameter.min_velocity_abs; //获取总的慢速时间(s)

	return true;
}

bool Interpolation_Class::Cal_Velocity(float current_distance)
{
	Interpolation_State = IS_Interpolating;	//正在插补

	current_distance *= Distance_Symbols;
	//获取插补速度
	if (current_distance < 0.0f)	//在反方向
	{
		target_velocity = Interpolation_Parameter.min_velocity_abs * Distance_Symbols;
		//Target_Coor_InOrigin.Clear();
	}
	else if (current_distance < acc_distance)//在加速区内
	{
		target_velocity = sqrtf(2 * current_distance * Interpolation_Parameter.acceleration_abs + Interpolation_Parameter.min_velocity_abs * Interpolation_Parameter.min_velocity_abs) * Distance_Symbols;
	}
	else if (current_distance < (acc_distance + const_distance))//在匀速区
	{
		target_velocity = Interpolation_Parameter.max_velocity_abs * Distance_Symbols;
	}
	else if (current_distance < (acc_distance + const_distance + dec_distance))//在减速区
	{
		target_velocity = sqrtf(Interpolation_Parameter.max_velocity_abs * Interpolation_Parameter.max_velocity_abs - 2 * (current_distance - acc_distance - const_distance) * Interpolation_Parameter.acceleration_abs) * Distance_Symbols;
	}
	else if (current_distance < (acc_distance + const_distance + dec_distance + slowly_distance - threshold))//在慢速区
	{
		target_velocity = Interpolation_Parameter.min_velocity_abs * Distance_Symbols;
		//Target_Coor_InOrigin = Destination_Coor_InOrigin;
	}
	else if (current_distance > (acc_distance + const_distance + dec_distance + slowly_distance + threshold))//在慢速区
	{
		target_velocity = -Interpolation_Parameter.min_velocity_abs * Distance_Symbols;
	}
	else    //在误差范围内
	{
		float temp = ABS(acc_distance + const_distance + dec_distance + slowly_distance - current_distance);
		target_velocity = 0.0f;
		target_distance = Interpolation_Class::distance;
		Interpolation_State = IS_Interpolated;
	}

	return (Interpolation_State != IS_Interpolated);	//返回插补结果，若插补完成，返回false
}
