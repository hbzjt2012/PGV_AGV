#pragma once
#include "../HALayer/Tim.h"
#include "../Configure.h"

/*
* 使用定时器对编码器硬件解码
* 定时器工作在编码器模式，在TI1、TI2边沿计数
* 未做定时器溢出处理
* 电机最低速的情况下，2.25ms/脉冲（4倍频）
* 对应轮子直径127mm的情况下，约0.05mm/脉冲(4倍频),即1mm/20脉冲(4倍频)
*/
class Encoder_Class : protected TIM_Base_Class
{
public:
	Encoder_Class(TIM_TypeDef *TIMx) : TIM_Base_Class(TIMx) {}
	~Encoder_Class() = default;

	void Init(bool dir);			//设置默认的计数方向
	static void Init_Period_TIM(void); //初始化用于读取编码器数据时间间隔的的定时器

	float Cal_Angular_Velocity(float time_ms);	//计算轮子角速度(单位为rad/ms)
	int16_t Get_Pulse(void);			//读取编码器旋转的角度
	void Clear(void) { TIM_Base_Class::Write(0x7FFF); }

	static unsigned long Read_Time(void) { return Encoder_Fre_Tim.Read(); }	//获取定时器时间
	static void Clear_Time_US(void) { Encoder_Fre_Tim.Init_UG(); }

private:
	static TIM_Base_Class Encoder_Fre_Tim;
	int16_t pulse_cnt;									   //编码器旋转的脉冲数(4倍频)
};
