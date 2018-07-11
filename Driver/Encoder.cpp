#include "Encoder.h"
#include "../parameter_define.h"
#include <misc.h>
#include "../macros.h"

TIM_Base_Class Encoder_Class::Encoder_Fre_Tim = TIM_Base_Class(TIM11);

void Encoder_Class::Init(bool dir)
{
	TIM_Base_Class::Init(0, 1);
	_TIMx->SMCR |= TIM_EncoderMode_TI12;				 //工作在编码器模式3，在TI1、TI2的边沿进行计数
	_TIMx->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //CC1、2通道为输入，IC1-TI1，IC2-TI2，无滤波器
	_TIMx->CCER |= ((uint16_t)dir) << 1;				 //根据DIR来确定CC1通道是否反向（CC1P），CC1、CC2捕获禁止
	TIM_Base_Class::Write(0x7FFF);						 //将初值计数设定为溢出计数的一半
	TIM_Base_Class::Begin();							 //开启定时器
}

void Encoder_Class::Init_Period_TIM(void)
{
	//设置时基，TIM11的时钟为168Mhz，1680分频，最大计数时间为655.3ms
	Encoder_Fre_Tim.Init((uint16_t)65530, 1680);

	Encoder_Fre_Tim.Begin(); //开始计数
}

//读取脉冲
int16_t Encoder_Class::Get_Pulse(void)
{
	pulse_cnt = ((TIM_Base_Class::Read()) - 0x7FFF); //在TI1、TI2边沿采样，4倍频
	Clear();
	//TIM_Base_Class::Write(0x7FFF);					 //将初值计数设定为溢出计数的一半
	return pulse_cnt;								 //4倍频
}

//计算车轮角速度(rad/ms)
float Encoder_Class::Cal_Angular_Velocity(float time_ms)
{
	float angular_velocity = 0.0f;	//角速度(rad/ms)
	angular_velocity = pulse_cnt / Parameter_Class::wheel_resolution *M_PI_2 / time_ms;	//编码器4倍频，90°对应车轮线数

	return angular_velocity;
}

//void TIM1_BRK_TIM9_IRQHandler(void)
//{
//	if (TIM9->SR & TIM_IT_Update) //更新中断
//	{
//		TIM9->SR = ~TIM_IT_Update; //清除中断
//		//Encoder_Class::time_10us_Interrupt_cnt++;
//		Encoder_Class::time_10us_Interrupt_cnt += 6553;
//		//GPIOA->ODR ^= 1 << 15;
//	}
//}
