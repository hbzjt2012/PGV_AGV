#pragma once
/*
* 电机由驱动器控制，输入量应为模拟量，单片机需要输出超过20KHz的PWM波来模拟模拟量
* 因为不是步进电机，故不需要对脉冲数量进行控制，因此不需要使用DMA传输
*/
#include "../Configure.h"
#include "../HALayer/IO.h"
#include "../HALayer/PWM.h"

class Motor_Class
{
  public:
	Motor_Class(IO_Class Dir, IO_Class Brake, IO_Class Stop, bool Dir_Init, TIM_TypeDef *TIMx) : _dir(Dir), _brake(Brake), _stop(Stop), dir_init(Dir_Init), _speed(TIMx) {}
	~Motor_Class() = default;

	void Init(uint32_t fre, uint16_t res, uint8_t channel); //初始化电机，电机准备开始，刹车状态
	void Set_Speed_Demo(float speed);
	void Set_Speed(float speed);							//设置电机速度(设置占空比)
	void Run_Enable(bool value);							//设置电机是否运行
	void Brake_Enable(bool value);							//设置电机是否刹车
	void Set_Dir(bool value);								//设置电机方向

  private:
	bool dir_init;	//定义电机初始转向
	PWM_Class _speed; //控制电机转速用的pwm
	IO_Class _dir;	//定义电机转向

	/*
	* _stop 0	_brake 0	正常运行
	* _stop	0	_brake 1	急停
	* _stop 1	_brake 0	电机及负载时自然停车
	*/
	IO_Class _brake; //运行/刹车控制IO
	IO_Class _stop;  //开始/停止控制IO
};