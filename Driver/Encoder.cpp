#include "Encoder.h"
#include <misc.h>

TIM_Base_Class Encoder_Class::Encoder_Fre_Tim = TIM_Base_Class(TIM9);
volatile unsigned long Encoder_Class::time_10us_Interrupt_cnt = 0;

void Encoder_Class::Init(bool dir)
{
	TIM_Base_Class::Init(0, 1);
	//TIM_Base_Class::Init_No_Interrupt(0, 1);			 //溢出计数为0xFFFF,分频系数1
	_TIMx->SMCR |= TIM_EncoderMode_TI12;				 //工作在编码器模式3，在TI1、TI2的边沿进行计数
	_TIMx->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //CC1、2通道为输入，IC1-TI1，IC2-TI2，无滤波器
	_TIMx->CCER |= ((uint16_t)dir) << 1;				 //根据DIR来确定CC1通道是否反向（CC1P），CC1、CC2捕获禁止
	TIM_Base_Class::Write(0x7FFF);						 //将初值计数设定为溢出计数的一半
	TIM_Base_Class::Begin();							 //开启定时器
}

void Encoder_Class::Init_Fre_TIM(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	Encoder_Fre_Tim.Init((uint16_t)65530, 1680); //TIM9的时钟为168Mhz，1680分频，最大计数时间为655.3ms
	//Encoder_Fre_Tim.Init_No_Interrupt((uint16_t)65530, 1680);	//TIM9的时钟为168Mhz，1680分频，最大计数时间为655.3ms

	//NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  //响应优先级
	//NVIC_Init(&NVIC_InitStructure);

	Encoder_Fre_Tim.Begin(); //开始计数
}

int16_t Encoder_Class::Get_Pulse(void)
{
	pulse_cnt = ((TIM_Base_Class::Read()) - 0x7FFF); //在TI1、TI2边沿采样，4倍频
	Clear();
	//TIM_Base_Class::Write(0x7FFF);					 //将初值计数设定为溢出计数的一半
	return pulse_cnt;								 //4倍频
}

void Encoder_Class::Set_Pulse(int16_t)
{
	pulse_cnt = 20;	//测试用
}

//************************************
// Method:    Update_Period
// FullName:  Encoder_Class::Update_Period
// Access:    public static
// Returns:   unsigned long 两次调用该函数的时间间隔，单位10us
// Parameter: void
// Description: 计算两次调用该函数的时间间隔，单位10us
//************************************
unsigned long Encoder_Class::Update_Period(void)
{
	unsigned long time_temp = time_10us_Interrupt_cnt + Encoder_Fre_Tim.Read(); //保存定时器计数,单位为10us
	//time_temp = time_10us_Interrupt_cnt;	//单位为10us
	//time_10us_Interrupt_cnt = 0;
	//Encoder_Class::Clear_Time_US();//重置上溢次数

	return time_temp;
}

//************************************
// Method:    Get_Palstance
// FullName:  Encoder_Class::Get_Palstance
// Access:    public
// Returns:   float	角速度
// Parameter: float 计算角速度用的时间（ms）
// Description:	以默认频率计算角速度（单位为°/ms）
//************************************
float Encoder_Class::Get_Palstance(float time_ms)
{
	//Get_Pulse();
	float palstance = 0.0f; //角速度（单位为°/ms）
#if ENCODER_FIX_WHEEL
	palstance = pulse_cnt * (90.0f / ENCODER_RESOLUTION) / time_ms; //计算角速度(°/ms)(考虑到pulse_cnt是4倍频，故90°对应编码器线数)
#else
	palstance = pulse_cnt * (90.0f / ENCODER_RESOLUTION) / time_ms / REDUCTION_RATIO; //计算角速度，该速度为电机的速度，应再除以减速比
#endif
	return palstance;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	if (TIM9->SR & TIM_IT_Update) //更新中断
	{
		TIM9->SR = ~TIM_IT_Update; //清除中断
		//Encoder_Class::time_10us_Interrupt_cnt++;
		Encoder_Class::time_10us_Interrupt_cnt += 6553;
		//GPIOA->ODR ^= 1 << 15;
	}
}
