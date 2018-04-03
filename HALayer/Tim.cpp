#include "Tim.h"

//************************************
// Method:    Init
// FullName:  TIM_Base_Class::Init
// Access:    public static 
// Returns:   void
// Parameter: TIM_TypeDef * TIM
// Parameter: uint16_t arr	溢出计数
// Parameter: uint16_t psc	分频系数
// Parameter: bool Open_Interrupt	设置是否打开中断
// Description: 设置定时器的分频系数和溢出计数，并关闭(打开)中断
//************************************
void TIM_Base_Class::Init(TIM_TypeDef * TIM, uint16_t arr, uint16_t psc, bool Open_Interrupt)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//TIM_DeInit(TIM);
	TIM_TimeBaseStructure.TIM_Period = arr - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);
	TIM->CR1 &= ~(1 << 1);	//清空UDIS,使能更新中断UEV
	TIM->CR1 |= (1 << 2);	 //置位URS,只有计数器上溢会生成UEV（更新中断）
	TIM->SR = ~TIM_IT_Update; //清零更新中断标志

	if (Open_Interrupt)
	{
		TIM->DIER |= TIM_IT_Update;	//使能更新中断
	}
}
