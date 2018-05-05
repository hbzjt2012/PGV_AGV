#include "Motor.h"

//************************************
// Method:    Init
// FullName:  Motor_Class::Init
// Access:    public
// Returns:   void
// Parameter: uint32_t fre	电机速度PWM波的频率
// Parameter: uint16_t res	电机速度PWM波的分辨率
// Parameter: uint8_t channel	电机使用PWM波的通道
// Description:	设置电机处于待命状态(开始，刹车)
//************************************
void Motor_Class::Init(uint32_t fre, uint16_t res, uint8_t channel)
{
	_dir.Init(GPIO_Mode_OUT);
	_dir.Write(dir_init);
	_brake.Init(GPIO_Mode_OUT);
	Brake_Enable(true);
	_stop.Init(GPIO_Mode_OUT);
	Run_Enable(true);
	_speed.Init(fre, res, channel);
	_speed.Set_duty(0);
}


void Motor_Class::Set_Speed_Demo(float speed)
{

	if (speed < 0.0f)
	{
		_dir.Write(!dir_init);
		speed = -speed;
	}
	else
	{
		_dir.Write(dir_init);
	}
	_speed.Set_duty_Demo(speed);
}

void Motor_Class::Set_Speed(float speed)
{

	if (speed < 0.0f)
	{
		_dir.Write(!dir_init);
		speed = -speed;
	}
	else
	{
		_dir.Write(dir_init);
	}
	_speed.Set_duty(speed);
}

//************************************
// Method:    Run_Enable
// FullName:  Motor_Class::Run_Enable
// Access:    public
// Returns:   void
// Parameter: bool value	true 电机开始，false 电机停止
// Description: 电机运行使能
//************************************
void Motor_Class::Run_Enable(bool value)
{
	_stop.Write(!value);
}

//************************************
// Method:    Brake_Enable
// FullName:  Motor_Class::Brake_Enable
// Access:    public
// Returns:   void
// Parameter: bool value	true刹车，false运行
// Description:	电机刹车使能
//************************************
void Motor_Class::Brake_Enable(bool value)
{
	_brake.Write(value);
}

//************************************
// Method:    Set_Dir
// FullName:  Motor_Class::Set_Dir
// Access:    public
// Returns:   void
// Parameter: bool value	true前进，false后退
// Description:	设置电机方向
//************************************
void Motor_Class::Set_Dir(bool value)
{
	_dir.Write(!(value ^ dir_init)); //设置方向
}
