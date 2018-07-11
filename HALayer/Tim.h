#pragma once
#include <stm32f4xx_tim.h>

/*
* 注意STM32F4的时钟树，TIM1、TIM8、TIM9-TIM11是挂载在APB2时钟线上的
* APB2的分配系数为2，时钟频率为84Mhz
* 因此挂载在APB2上的定时器时钟频率为84Mhz*2=168Mhz
*
* 同理，挂载在APB1(42Mhz,分频系数4)上的TIM2-TIM7、TIM12-TIM15的时钟频率为84Mhz
*/

//定时器溢出频率f=fclk/(arr*psc)

class TIM_Base_Class
{
public:
	TIM_Base_Class(TIM_TypeDef *TIMx) : _TIMx(TIMx) {}
	virtual ~TIM_Base_Class() = default;

	inline void Init(uint16_t arr, uint16_t psc, bool Open_Interrupt = false) { TIM_Base_Class::Init(_TIMx, arr, psc, Open_Interrupt); }	//初始化定时器，(不)打开中断
	inline void Begin(void) { TIM_Base_Class::Begin(_TIMx); }	//开启定时器
	inline void Stop(void) { TIM_Base_Class::Stop(_TIMx); }	//关闭定时器

	void Write(uint16_t cnt) { _TIMx->CNT = cnt; } //写定时器的计数
	uint16_t Read(void) { return _TIMx->CNT; }		//返回定时器的计数
	void Init_UG(void) { _TIMx->EGR = 1; }		  //通过UG位初始化寄存器

	static void Init(TIM_TypeDef * TIM, uint16_t arr, uint16_t psc, bool Open_Interrupt = false);
	static inline void Begin(TIM_TypeDef * TIM) { TIM->CR1 |= TIM_CR1_CEN; }//开启定时器
	static inline void Stop(TIM_TypeDef * TIM) { TIM->CR1 &= ~TIM_CR1_CEN; }//关闭定时器

protected:
	TIM_TypeDef *_TIMx; //使用的定时器
};
