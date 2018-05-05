#include "PWM.h"

//************************************
// Method:    Init
// FullName:  PWM_Class::Init
// Access:    public
// Returns:   void
// Parameter: uint32_t fre	PWM波频率
// Parameter: uint16_t res	PWM波分辨率
// Parameter: uint8_t channel	PWM波使用的通道，取值1-4，使用时需注意该定时器是否有该通道
// Description:	根据频率和分辨率设置定时器
//************************************
void PWM_Class::Init(uint32_t fre, uint16_t res, uint8_t channel)
{
	uint32_t tim_fclk = 0; //定时器时钟
	uint16_t psc = 0;

	//frequency = fre;
	resolution = res;
	tim_fclk = (((uint32_t)_TIMx) > APB2PERIPH_BASE) ? 168000000UL : 84000000UL; //获取当前定时器时钟频率
	psc = tim_fclk / (fre * res);//分频系数
	TIM_Base_Class::Init(res, psc);	 //设置定时器时基单元

	CCRx = (uint32_t *)((uint32_t)(&(_TIMx->CCR1)) + ((channel - 1) << 2)); //设置通道使用的捕获比较寄存器
	Init_OC(channel);

	Begin();
}

//测试电机用代码
void PWM_Class::Set_duty_Demo(float duty)
{
	uint16_t duty_cycle = 0;
	duty_cycle = duty * (resolution >> 1);
	duty_cycle = RANGE(duty_cycle, 0, resolution >> 1);
	Set_duty(duty_cycle);
}

//************************************
// Method:    Set_duty
// FullName:  PWM_Class::Set_duty
// Access:    public
// Returns:   void
// Parameter: float duty 占空比,0-1
// Description:	设置PWM波占空比
//************************************
void PWM_Class::Set_duty(float duty)
{
	uint16_t duty_cycle = 0;
	duty_cycle = duty * resolution;
	duty_cycle = RANGE(duty_cycle, 0, resolution);
	Set_duty(duty_cycle);
}

//************************************
// Method:    Set_duty
// FullName:  PWM_Class::Set_duty
// Access:    public
// Returns:   void
// Parameter: uint16_t duty	占空比，取值为0~分辨率
// Description:	设置PWM波占空比
//************************************
void PWM_Class::Set_duty(uint16_t duty)
{
	*CCRx = duty;
}

//************************************
// Method:    Init_OC
// FullName:  PWM_Class::Init_OC
// Access:    private
// Returns:   void
// Parameter: uint8_t channel	通道，取值1-4
// Description:	设置定时器比较输出通道
//************************************
void PWM_Class::Init_OC(uint8_t channel)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure); //初始化结构体

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  //PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 //输出极性高

	switch (channel)
	{
	case 1:
		TIM_OC1Init(_TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(_TIMx, TIM_OCPreload_Enable);
		TIM_OC1FastConfig(_TIMx, TIM_OCFast_Enable);
		break;
	case 2:
		TIM_OC2Init(_TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(_TIMx, TIM_OCPreload_Enable);
		TIM_OC2FastConfig(_TIMx, TIM_OCFast_Enable);
		break;
	case 3:
		TIM_OC3Init(_TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(_TIMx, TIM_OCPreload_Enable);
		TIM_OC3FastConfig(_TIMx, TIM_OCFast_Enable);
		break;
	case 4:
		TIM_OC4Init(_TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(_TIMx, TIM_OCPreload_Enable);
		TIM_OC4FastConfig(_TIMx, TIM_OCFast_Enable);
		break;
	default:
		break;
	}
}
