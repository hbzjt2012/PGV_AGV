#pragma once
#include "../HALayer/Tim.h"
#include "../Configure.h"

extern "C" void TIM1_BRK_TIM9_IRQHandler(void);
/*
* 使用定时器对编码器硬件解码
* 定时器工作在编码器模式，在TI1、TI2边沿计数
* 未做定时器溢出处理
* 电机最低速的情况下，2.25ms/脉冲（4倍频）
* 对应轮子直径127mm的情况下，约0.05mm/脉冲(4倍频),即1mm/20脉冲(4倍频)
*/
class Encoder_Class : protected TIM_Base_Class
{
	friend void TIM1_BRK_TIM9_IRQHandler(void);

  public:
	Encoder_Class(TIM_TypeDef *TIMx) : TIM_Base_Class(TIMx) {}
	~Encoder_Class() = default;

	void Init(bool dir);			//设置默认的计数方向
	static void Init_Fre_TIM(void); //初始化用于采样频率计算的定时器

	float Get_Palstance(float time_ms); //根据采样时间计算角速度（单位为°/ms）
	int16_t Get_Pulse(void);			//读取编码器旋转的角度

	void Set_Pulse(int16_t);	//测试用

	static unsigned long Update_Period(void); //更新
	static void Clear_Time_US(void)
	{
		Encoder_Fre_Tim.Init_UG();
		time_10us_Interrupt_cnt = 0;
	}

  private:
	static TIM_Base_Class Encoder_Fre_Tim;
	static volatile unsigned long time_10us_Interrupt_cnt; //单位10us
	int16_t pulse_cnt;									   //编码器旋转的脉冲数(4倍频)
};
