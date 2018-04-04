#include "Mecanum.h"
//#include "../macros.h"

#define Velocity_RES 1000 //速度分辨率

////前左轮
//Motor_Class Mecanum_Wheel_Class::Front_Left_Wheel = \
//Motor_Class(IO_Class(GPIOE, GPIO_Pin_13), IO_Class(GPIOE, GPIO_Pin_14), IO_Class(GPIOE, GPIO_Pin_15), true, TIM2);
//
////前右轮
//Motor_Class Mecanum_Wheel_Class::Front_Right_Wheel = \
//Motor_Class(IO_Class(GPIOE, GPIO_Pin_10), IO_Class(GPIOE, GPIO_Pin_11), IO_Class(GPIOE, GPIO_Pin_12), true, TIM2);
//
////后左轮
//Motor_Class Mecanum_Wheel_Class::Back_Left_Wheel = \
//Motor_Class(IO_Class(GPIOD, GPIO_Pin_11), IO_Class(GPIOD, GPIO_Pin_12), IO_Class(GPIOD, GPIO_Pin_13), true, TIM12);
//
////后右轮
//Motor_Class Mecanum_Wheel_Class::Back_Right_Wheel = \
//Motor_Class(IO_Class(GPIOD, GPIO_Pin_8), IO_Class(GPIOD, GPIO_Pin_9), IO_Class(GPIOD, GPIO_Pin_10), true, TIM12);
//
//Encoder_Class Mecanum_Wheel_Class::Front_Left_Encoder = Encoder_Class(TIM1, 100);//前左编码器
//Encoder_Class Mecanum_Wheel_Class::Front_Right_Encoder = Encoder_Class(TIM8, 100);//前右编码器
//Encoder_Class Mecanum_Wheel_Class::Back_Left_Encoder = Encoder_Class(TIM3, 100);//后左编码器
//Encoder_Class Mecanum_Wheel_Class::Back_Right_Encoder = Encoder_Class(TIM4, 100);//后右编码器

Motor_Class Mecanum_Wheel_Class::Front_Left_Wheel = Motor_Class FRONT_LEFT_MOTOR;	 //前左轮
Motor_Class Mecanum_Wheel_Class::Front_Right_Wheel = Motor_Class FRONT_RIGHT_MOTOR;   //前右轮
Motor_Class Mecanum_Wheel_Class::Behind_Left_Wheel = Motor_Class BEHIND_LEFT_MOTOR;   //后左轮
Motor_Class Mecanum_Wheel_Class::Behind_Right_Wheel = Motor_Class BEHIND_RIGHT_MOTOR; //后右轮

Encoder_Class Mecanum_Wheel_Class::Front_Left_Encoder = Encoder_Class FRONT_LEFTT_ENCODER;	//前左编码器
Encoder_Class Mecanum_Wheel_Class::Front_Right_Encoder = Encoder_Class FRONT_RIGHT_ENCODER;   //前右编码器
Encoder_Class Mecanum_Wheel_Class::Behind_Left_Encoder = Encoder_Class BEHIND_LEFTT_ENCODER;  //后左编码器
Encoder_Class Mecanum_Wheel_Class::Behind_Right_Encoder = Encoder_Class BEHIND_RIGHT_ENCODER; //后右编码器

//************************************
// Method:    Init
// FullName:  Mecanum_Wheel_Class::Init
// Access:    public
// Returns:   void
// Parameter: void
// Description: 初始化AGV运动所需的电机和编码器（注意！此处对IO进行了复用）
//************************************
void Mecanum_Wheel_Class::Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = _BV(10) | _BV(11) | _BV(14) | _BV(15);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);

	Front_Left_Wheel.Init(84000, Velocity_RES, FRONT_LEFT_MOTOR_TIM_CHANNEL);
	Front_Right_Wheel.Init(84000, Velocity_RES, FRONT_RIGHT_MOTOR_TIM_CHANNEL);
	Behind_Left_Wheel.Init(84000, Velocity_RES, BEHIND_LEFT_MOTOR_TIM_CHANNEL);
	Behind_Right_Wheel.Init(84000, Velocity_RES, BEHIND_RIGHT_MOTOR_TIM_CHANNEL);

	GPIO_InitStructure.GPIO_Pin = _BV(8) | _BV(9);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = _BV(4) | _BV(5) | _BV(6) | _BV(7);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = _BV(6) | _BV(7);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);

	Front_Left_Encoder.Init(!FRONT_LEFTT_DEFAULT_DIR);
	Front_Right_Encoder.Init(!FRONT_RIGHT_DEFAULT_DIR);
	Behind_Left_Encoder.Init(!BEHIND_LEFTT_DEFAULT_DIR);
	Behind_Right_Encoder.Init(!BEHIND_RIGHT_DEFAULT_DIR);

	V_InAGV_Coor_InWorld.Velocity.x_velocity = 0.0f;
	V_InAGV_Coor_InWorld.Velocity.y_velocity = 0.0f;
	V_InAGV_Coor_InWorld.Velocity.yaw_velocity = 0.0f;
	V_InAGV_Coor_InWorld.Coordinate.x_coor = 0.0f;
	V_InAGV_Coor_InWorld.Coordinate.y_coor = 0.0f;
	V_InAGV_Coor_InWorld.Coordinate.angle_coor = 0.0f;
	Write_Velocity(V_InAGV_Coor_InWorld.Velocity);

	Encoder_Class::Init_Fre_TIM();
}

void Mecanum_Wheel_Class::Brake(bool value)
{
	Front_Left_Wheel.Brake_Enable(value);
	Front_Right_Wheel.Brake_Enable(value);
	Behind_Left_Wheel.Brake_Enable(value);
	Behind_Right_Wheel.Brake_Enable(value);
}

void Mecanum_Wheel_Class::Run(bool value)
{
	Front_Left_Wheel.Run_Enable(value);
	Front_Right_Wheel.Run_Enable(value);
	Behind_Left_Wheel.Run_Enable(value);
	Behind_Right_Wheel.Run_Enable(value);
}

bool Mecanum_Wheel_Class::demo(const Position_Class::Velocity_Class &Target_velocity, float &pulse, float &time_ms)
{
	//float _fre_temp = 0.0f;
	float angular_velocity_FR = 0.0f, angular_velocity_FL = 0.0f; //前右，前左轮角速度
	float angular_velocity_BL = 0.0f, angular_velocity_BR = 0.0f; //后左，后右轮角速度
	static unsigned long time_last_10us = 0, time_current_10us = 0;
	static unsigned long time_10us = 0;

	//计算时间阈值
	static unsigned long time_10us_threshold = 0; //保存编码器计数的时间阈值

	if (!time_10us_threshold) //阈值==0，计算新阈值
	{
		float abs_x = ABS(Target_velocity.x_velocity);
		float abs_y = ABS(Target_velocity.y_velocity);
		float abs = abs_x > abs_y ? abs_x : abs_y;

		if (abs < FLOAT_DELTA)
		{
			time_10us_threshold = 0;
		}
		else
		{
			time_10us_threshold = 100000UL / abs; //20个脉冲对应的us数是10^6
		}

		//若新阈值>0，重置时间
		if (time_10us_threshold > 0)
		{
			if (time_10us_threshold < 1000) //对应的是10ms
			{
				time_10us_threshold = 1000;
			}
			Encoder_Class::Clear_Time_US(); //清空计数器
			time_current_10us = 0;
			time_last_10us = 0;
			time_10us = 0;
		}
		//Encoder_Class::Update_Period();
	}

	if (time_10us_threshold) //阈值>0,获取新时间
	{
		time_current_10us = Encoder_Class::Update_Period(); //当前计数定时器计数
		time_10us += (time_current_10us - time_last_10us);

		//time_10us = Encoder_Class::Update_Period();	//上一次的定时器计数(单位10us)
		//time_10us += Encoder_Class::Update_Period();	//获取该次计算编码器旋转角度的总时间(10us)

		//time_ms_threshold = 1000.0f / (abs_x > abs_y ? abs_x : abs_y) ;	//计算时间阈值

		if (time_10us > time_10us_threshold)
		{

			Front_Left_Encoder.Get_Pulse(); //读取编码器旋转的脉冲数
			Front_Right_Encoder.Get_Pulse();
			Behind_Left_Encoder.Get_Pulse();
			Behind_Right_Encoder.Get_Pulse();

			//根据角速度和运动学计算位移
			//计算4个轮子的角速度(°/s)
			float time_ms = time_10us / 100.0f;

			angular_velocity_FR = Front_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_FL = Front_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_BR = Behind_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_BL = Behind_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;

			pulse = (angular_velocity_FR + angular_velocity_FL + angular_velocity_BL + angular_velocity_BR) * M_PI / 180 * WHEEL_DIAMETER / 4 / 2;

			//time_ms = time_10us / 100.0f;
			Encoder_Class::Clear_Time_US(); //清空计数器
			time_current_10us = 0;
			time_last_10us = 0;
			time_10us_threshold = 0;
			time_10us = 0;
			GPIOA->ODR ^= 1 << 15;
			return true;
		}
		else
		{
			time_last_10us = time_current_10us;
		}
	}

	return false;
}

void Mecanum_Wheel_Class::Write_Velocity(Position_Class::Velocity_Class &velocity_InAGV)
{
	static float duty_FR, duty_FL, duty_BR, duty_BL;
	float temp = (DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2 * velocity_InAGV.yaw_velocity;
	duty_FR = (-velocity_InAGV.x_velocity + velocity_InAGV.y_velocity + temp) / WHEEL_MAX_LINE_VELOCITY;
	duty_FL = (velocity_InAGV.x_velocity + velocity_InAGV.y_velocity - temp) / WHEEL_MAX_LINE_VELOCITY;
	duty_BL = (-velocity_InAGV.x_velocity + velocity_InAGV.y_velocity - temp) / WHEEL_MAX_LINE_VELOCITY;
	//duty_BL = (velocity.x_speed + velocity.y_speed + temp) / WHEEL_MAX_LINE_VELOCITY;
	duty_FR = RANGE(duty_FR, -1.0f, 1.0f);
	duty_FL = RANGE(duty_FL, -1.0f, 1.0f);
	duty_BL = RANGE(duty_BL, -1.0f, 1.0f);
	duty_BR = duty_FR + duty_FL - duty_BL;

	//velocity_InAGV.x_velocity = 0.0f;
	//velocity_InAGV.y_velocity = 0.0f;
	//velocity_InAGV.yaw_velocity = 0.0f;

	Front_Left_Wheel.Set_Speed(duty_FL);
	Front_Right_Wheel.Set_Speed(duty_FR);
	Behind_Left_Wheel.Set_Speed(duty_BL);
	Behind_Right_Wheel.Set_Speed(duty_BR);
}

Position_Class &Mecanum_Wheel_Class::Update_Post(Position_Class &Current_Position, const Position_Class::Velocity_Class &Target_velocity)
{
	//float _fre_temp = 0.0f;
	float angular_velocity_FR = 0.0f, angular_velocity_FL = 0.0f;   //前右，前左轮角速度
	float angular_velocity_BL = 0.0f, angular_velocity_BR = 0.0f;   //后左，后右轮角速度
	static unsigned long time_last_10us = 0, time_current_10us = 0; //上一次时间计数，当前时间计数
	static unsigned long time_10us = 0;

	//计算时间阈值
	static unsigned long time_10us_threshold = 0; //保存编码器计数的时间阈值

	if (!time_10us_threshold) //阈值==0，计算新阈值
	{
		float abs_x = ABS(Target_velocity.x_velocity);
		float abs_y = ABS(Target_velocity.y_velocity);
		float abs = abs_x > abs_y ? abs_x : abs_y;

		if (abs < FLOAT_DELTA)
		{
			time_10us_threshold = 0;
		}
		else
		{
			time_10us_threshold = 100000UL / abs; //20个脉冲对应的us数是10^6
		}

		//若新阈值>0，重置时间
		if (time_10us_threshold > 0)
		{
			//if (time_10us_threshold < 1000)	//对应的是10ms
			//{
			//	time_10us_threshold = 1000;
			//}
			Encoder_Class::Clear_Time_US(); //清空计数器
			time_current_10us = 0;
			time_last_10us = 0;
			time_10us = 0;
		}
		//Encoder_Class::Update_Period();
	}

	if (time_10us_threshold > 0) //阈值>0,获取新时间
	{
		time_current_10us = Encoder_Class::Update_Period(); //当前计数定时器计数
		time_10us += (time_current_10us - time_last_10us);
		//time_10us += Encoder_Class::Update_Period();	//获取该次计算编码器旋转角度的总时间(10us)
		//time_ms_threshold = 1000.0f / (abs_x > abs_y ? abs_x : abs_y) ;	//计算时间阈值

		if (time_10us > time_10us_threshold)
		{

			Front_Left_Encoder.Get_Pulse(); //读取编码器旋转的脉冲数
			Front_Right_Encoder.Get_Pulse();
			Behind_Left_Encoder.Get_Pulse();
			Behind_Right_Encoder.Get_Pulse();

			//根据角速度和运动学计算位移
			//计算4个轮子的角速度(°/s)
			float time_ms = time_10us / 100.0f;

			angular_velocity_FR = Front_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_FL = Front_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_BR = Behind_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_BL = Behind_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;

			//PI* WHEEL_DIAMETER/1440=PI / 180 * WHEEL_DIAMETER /2/ 4
			//PI/180为转换成弧度
			//WHEEL_DIAMETER /2为半径
			Current_Position.Velocity.x_velocity = (-angular_velocity_FR + angular_velocity_FL - angular_velocity_BL + angular_velocity_BR) * M_PI * WHEEL_DIAMETER / 1440;
			Current_Position.Velocity.y_velocity = (angular_velocity_FR + angular_velocity_FL + angular_velocity_BL + angular_velocity_BR) * M_PI * WHEEL_DIAMETER / 1440;
			Current_Position.Velocity.yaw_velocity = (angular_velocity_FR - angular_velocity_FL - angular_velocity_BL + angular_velocity_BR) * M_PI * WHEEL_DIAMETER / 1440 / (DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES);
					
			Current_Position.Coordinate.x_coor += Current_Position.Velocity.x_velocity * time_ms / 1000.0f;
			Current_Position.Coordinate.y_coor += Current_Position.Velocity.y_velocity * time_ms / 1000.0f;
			Current_Position.Coordinate.angle_coor += Current_Position.Velocity.yaw_velocity * time_ms / 1000.0f;

			Encoder_Class::Clear_Time_US(); //清空计数器
			time_current_10us = 0;
			time_last_10us = 0;
			time_10us_threshold = 0;
			time_10us = 0;

			GPIOA->ODR ^= 1 << 15;
		}
		else
		{
			time_last_10us = time_current_10us;
		}
	}

	else
	{
		Current_Position.Velocity.x_velocity = 0.0f;
		Current_Position.Velocity.y_velocity = 0.0f;
		Current_Position.Velocity.yaw_velocity = 0.0f;
	}
	return Current_Position;
}
