#include "Mecanum.h"
#include "../Math/Trigonometric.h"
#include <cmath>

#define Velocity_RES 10000 //速度分辨率
//使用Lyapunov方法根据误差更新速度
#define C1 2.0f
#define C2 C1
#define C3 20.0f

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

	//Front_Left_Wheel.Init(84000, Velocity_RES, FRONT_LEFT_MOTOR_TIM_CHANNEL);
	//Front_Right_Wheel.Init(84000, Velocity_RES, FRONT_RIGHT_MOTOR_TIM_CHANNEL);
	//Behind_Left_Wheel.Init(84000, Velocity_RES, BEHIND_LEFT_MOTOR_TIM_CHANNEL);
	//Behind_Right_Wheel.Init(84000, Velocity_RES, BEHIND_RIGHT_MOTOR_TIM_CHANNEL);

	//测试用，PWM频率2K
	Front_Left_Wheel.Init(2000, Velocity_RES, FRONT_LEFT_MOTOR_TIM_CHANNEL);
	Front_Right_Wheel.Init(2000, Velocity_RES, FRONT_RIGHT_MOTOR_TIM_CHANNEL);
	Behind_Left_Wheel.Init(2000, Velocity_RES, BEHIND_LEFT_MOTOR_TIM_CHANNEL);
	Behind_Right_Wheel.Init(2000, Velocity_RES, BEHIND_RIGHT_MOTOR_TIM_CHANNEL);

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

	//Displacement_Velocity_InAGV_By_Encoder.Velocity.x_velocity = 0.0f;
	//Displacement_Velocity_InAGV_By_Encoder.Velocity.y_velocity = 0.0f;
	//Displacement_Velocity_InAGV_By_Encoder.Velocity.angle_velocity = 0.0f;
	//Displacement_Velocity_InAGV_By_Encoder.Coordinate.x_coor = 0.0f;
	//Displacement_Velocity_InAGV_By_Encoder.Coordinate.y_coor = 0.0f;
	//Displacement_Velocity_InAGV_By_Encoder.Coordinate.angle_coor = 0.0f;
	Write_Velocity(Displacement_Velocity_InAGV_By_Encoder.Velocity);

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

Position_Class::Velocity_Class & Mecanum_Wheel_Class::Update_Velocity_By_ErrorCoor(const Position_Class::Coordinate_Class & Error_Coor_InAGV, Position_Class::Velocity_Class & AGV_Velocity_InAGV)
{
	//此处算法应改进
	if (ABS(Error_Coor_InAGV.x_coor) > 0.1f)
	{
		AGV_Velocity_InAGV.x_velocity += C1*Error_Coor_InAGV.x_coor;
	}
	if (ABS(Error_Coor_InAGV.y_coor) > 0.1f)
	{
		AGV_Velocity_InAGV.y_velocity += C2*Error_Coor_InAGV.y_coor;
	}
	if (ABS(Error_Coor_InAGV.angle_coor) > 0.1f)
	{
		AGV_Velocity_InAGV.angle_velocity += C3*Sin_Lookup(Error_Coor_InAGV.angle_coor);
	}
	return AGV_Velocity_InAGV;
}

//依照麦克纳姆轮的物理限制，更新速度
Position_Class::Velocity_Class & Mecanum_Wheel_Class::Update_Velocity_By_Limit(Position_Class::Velocity_Class & Velocity)
{
	//根据麦克纳姆轮的运动学关系式以及不打滑的约束方程
	//Vx，Vy，(Lx+Ly)W(弧度)的约束面为正八面体的表面,即|Vx|+|Vy|+|(Lx+Ly)W(弧度)|<=V_wheel

	//|Velocity.x_velocity|+|Velocity.y_velocity|+|angular_velocity|<=WHEEL_MAX_LINE_VELOCITY
	float angular_velocity = Velocity.angle_velocity*M_PI / 180.0f*(DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2.0f;
	float abs_temp = ABS(Velocity.x_velocity) + ABS(Velocity.y_velocity) + ABS(angular_velocity);
	if (abs_temp > WHEEL_MAX_LINE_VELOCITY)
	{
		float k = WHEEL_MAX_LINE_VELOCITY / abs_temp;
		Velocity *= k;
	}
	return Velocity;
}

//************************************
// Method:    Write_Velocity
// FullName:  Mecanum_Wheel_Class::Write_Velocity
// Access:    public 
// Returns:   void
// Parameter: Position_Class::Velocity_Class & AGV_Velocity_InAGV
// Description: 将AGV速度转换为车轮速度
//************************************
void Mecanum_Wheel_Class::Write_Velocity(Position_Class::Velocity_Class &AGV_Velocity_InAGV)
{
	float duty_FR, duty_FL, duty_BR, duty_BL;
	float yaw_temp = (DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2 * AGV_Velocity_InAGV.angle_velocity;
	float temp = yaw_temp* M_PI / 180.0f;	//单位为mm/s

	float x_velocity_abs, y_velocity_abs, yaw_velocity_abs;

	x_velocity_abs = ABS(AGV_Velocity_InAGV.x_velocity);
	y_velocity_abs = ABS(AGV_Velocity_InAGV.y_velocity);
	yaw_velocity_abs = ABS(yaw_temp);

	velocity = sqrtf(x_velocity_abs*x_velocity_abs + y_velocity_abs*y_velocity_abs + temp*temp);	//更新AGV的质心速度2范数

	duty_FR = (-AGV_Velocity_InAGV.x_velocity + AGV_Velocity_InAGV.y_velocity + temp) / WHEEL_MAX_LINE_VELOCITY;
	duty_FL = (AGV_Velocity_InAGV.x_velocity + AGV_Velocity_InAGV.y_velocity - temp) / WHEEL_MAX_LINE_VELOCITY;
	duty_BL = (-AGV_Velocity_InAGV.x_velocity + AGV_Velocity_InAGV.y_velocity - temp) / WHEEL_MAX_LINE_VELOCITY;
	//duty_BL = (velocity.x_speed + velocity.y_speed + temp) / WHEEL_MAX_LINE_VELOCITY;
	duty_FR = RANGE(duty_FR, -1.0f, 1.0f);
	duty_FL = RANGE(duty_FL, -1.0f, 1.0f);
	duty_BL = RANGE(duty_BL, -1.0f, 1.0f);
	duty_BR = duty_FR + duty_FL - duty_BL;

	Front_Left_Wheel.Set_Speed(duty_FL);
	Front_Right_Wheel.Set_Speed(duty_FR);
	Behind_Left_Wheel.Set_Speed(duty_BL);
	Behind_Right_Wheel.Set_Speed(duty_BR);
}

//************************************
// Method:    Update_Post_By_Encoder
// FullName:  Mecanum_Wheel_Class::Update_Post_By_Encoder
// Access:    public 
// Returns:   Position_Class & 返回世界坐标系下的AGV坐标和速度
// Parameter: Position_Class & Current_InWorld 世界坐标系下当前AGV的坐标和速度
// Description: 根据编码器更新世界坐标系下的坐标和速度
//************************************
Position_Class & Mecanum_Wheel_Class::Update_Post_By_Encoder(Position_Class & Current_InWorld, bool &update_by_extern)
{
	static unsigned short time_10us_threshold = 0;
	static unsigned long time_last_10us = 0, time_current_10us = 0; //上一次时间计数，当前时间计数
	static unsigned long time_10us = 0;


	float angular_velocity_FR, angular_velocity_FL;   //前右，前左轮角速度
	float angular_velocity_BL, angular_velocity_BR;   //后左，后右轮角速度

	if (update_by_extern)	//坐标由外部更新
	{
		time_10us_threshold = 0;	//重新计算阈值
		update_by_extern = false;
	}

	if (!time_10us_threshold) //阈值==0，计算新阈值
	{
		time_10us_threshold = Cal_Cycle();	//计算时间周期


		//若新阈值>0，重置时间
		if (time_10us_threshold > 0)
		{
			Encoder_Class::Clear_Time_US(); //清空计数器
			time_current_10us = 0;
			time_last_10us = 0;
			time_10us = 0;
		}
		else
		{
			Front_Right_Encoder.Clear();
			Front_Left_Encoder.Clear();
			Behind_Right_Encoder.Clear();
			Behind_Left_Encoder.Clear();
		}
	}

	if (time_10us_threshold > 0) //阈值>0,获取新时间
	{
		time_current_10us = Encoder_Class::Update_Period(); //当前计数定时器计数
		time_10us += (time_current_10us - time_last_10us);
		//time_10us = time_10us_threshold +1;
		if (time_10us > time_10us_threshold)
		{
			//读取编码器脉冲数
			Front_Left_Encoder.Get_Pulse(); //读取编码器旋转的脉冲数
			Front_Right_Encoder.Get_Pulse();
			Behind_Left_Encoder.Get_Pulse();
			Behind_Right_Encoder.Get_Pulse();

			////测试用
			//Front_Left_Encoder.Set_Pulse(20); //读取编码器旋转的脉冲数
			//Front_Right_Encoder.Set_Pulse(20);
			//Behind_Left_Encoder.Set_Pulse(20);
			//Behind_Right_Encoder.Set_Pulse(20);


			//根据角速度和运动学计算位移
			//计算4个轮子的角速度(°/s)
			float time_ms = time_10us / 100.0f;

			angular_velocity_FR = Front_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_FL = Front_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_BR = Behind_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
			angular_velocity_BL = Behind_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;

			//PI* WHEEL_DIAMETER/1440=M_PI / 180 * WHEEL_DIAMETER /2/ 4
			//PI/180为转换成弧度
			//WHEEL_DIAMETER /2为半径
			//获取AGV坐标系下AGV的速度和位移

			Displacement_Velocity_InAGV_By_Encoder.Velocity.x_velocity = (-angular_velocity_FR + angular_velocity_FL - angular_velocity_BL + angular_velocity_BR) * M_PI * WHEEL_DIAMETER / 1440;
			Displacement_Velocity_InAGV_By_Encoder.Velocity.y_velocity = (angular_velocity_FR + angular_velocity_FL + angular_velocity_BL + angular_velocity_BR) * M_PI * WHEEL_DIAMETER / 1440;
			Displacement_Velocity_InAGV_By_Encoder.Velocity.angle_velocity = (angular_velocity_FR - angular_velocity_FL - angular_velocity_BL + angular_velocity_BR) * WHEEL_DIAMETER / 8 / ((DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2);

			Displacement_Velocity_InAGV_By_Encoder.Coordinate.x_coor = Displacement_Velocity_InAGV_By_Encoder.Velocity.x_velocity * time_ms / 1000.0f;
			Displacement_Velocity_InAGV_By_Encoder.Coordinate.y_coor = Displacement_Velocity_InAGV_By_Encoder.Velocity.y_velocity * time_ms / 1000.0f;
			Displacement_Velocity_InAGV_By_Encoder.Coordinate.angle_coor = Displacement_Velocity_InAGV_By_Encoder.Velocity.angle_velocity * time_ms / 1000.0f;

			Position_Class Absolute_coor_Inworld_temp;

			//将AGV坐标系中的速度和位移变换至世界坐标系
			Absolute_coor_Inworld_temp = Position_Class::Relative_To_Absolute(Absolute_coor_Inworld_temp, Displacement_Velocity_InAGV_By_Encoder, Current_InWorld.Coordinate);

			Current_InWorld.Coordinate = Absolute_coor_Inworld_temp.Coordinate;
			Current_InWorld.Velocity = Absolute_coor_Inworld_temp.Velocity;

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
		Current_InWorld.Velocity.x_velocity = 0.0f;
		Current_InWorld.Velocity.y_velocity = 0.0f;
		Current_InWorld.Velocity.angle_velocity = 0.0f;
	}
	return Current_InWorld;
}

float Mecanum_Wheel_Class::Get_theta_rate(float time_ms)
{
	float angular_velocity_FR, angular_velocity_FL;   //前右，前左轮角速度
	float angular_velocity_BL, angular_velocity_BR;   //后左，后右轮角速度

	float theta_rate = 0.0f;	//角速度（°/s）

	Front_Left_Encoder.Get_Pulse(); //读取编码器旋转的脉冲数
	Front_Right_Encoder.Get_Pulse();
	Behind_Left_Encoder.Get_Pulse();
	Behind_Right_Encoder.Get_Pulse();

	//根据角速度和运动学计算位移
	//计算4个轮子的角速度(°/s)

	angular_velocity_FR = Front_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
	angular_velocity_FL = Front_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;
	angular_velocity_BR = Behind_Right_Encoder.Get_Palstance(time_ms) * 1000.0f;
	angular_velocity_BL = Behind_Left_Encoder.Get_Palstance(time_ms) * 1000.0f;

	//PI* WHEEL_DIAMETER/1440=M_PI / 180 * WHEEL_DIAMETER /2/ 4
	//PI/180为转换成弧度
	//WHEEL_DIAMETER /2为半径
	//获取AGV坐标系下AGV的速度和位移

	theta_rate = (angular_velocity_FR - angular_velocity_FL - angular_velocity_BL + angular_velocity_BR) * WHEEL_DIAMETER / 8 / ((DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2);

	return theta_rate;
}

