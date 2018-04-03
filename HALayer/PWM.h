#pragma once
#include "Tim.h"
#include "../macros.h"


/*
* 使用定时器产生PWM类
* 使用时请注意该定时器是否有输出通道
* 未对IO口进行设置
*/

class PWM_Class : protected TIM_Base_Class
{
  public:
	PWM_Class(TIM_TypeDef *TIMx) : TIM_Base_Class(TIMx) {}
	virtual ~PWM_Class() = default;

	void Init(uint32_t fre, uint16_t res, uint8_t channel); //初始化用于产生PWM波的定时器通道，不打开中断
	void Set_duty(float duty);								//设置占空比
	void Set_duty(uint16_t duty);							//设置占空比
	void Set_duty(int duty) { Set_duty((uint16_t)duty); }   //设置占空比

  private:
	void Init_OC(uint8_t channel); //设置PWM波通道,未设置IO口

	//uint32_t frequency;	//PWM波频率
	uint16_t resolution; //分辨率
	uint32_t *CCRx;		 //指示当前使用的捕获比较寄存器
};
