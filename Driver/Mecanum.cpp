#include "Mecanum.h"
#include "../Math/Trigonometric.h"
#include <cmath>


#define Velocity_RES 10000 //速度分辨率
////使用Lyapunov方法根据误差更新速度
//#define C1 2.0f
//#define C2 C1
//#define C3 20.0f

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

	AGV_Velocity_InAGV.velocity = 0.0f;
	AGV_Velocity_InAGV.velocity_angle = 0.0f;
	AGV_Velocity_InAGV.angular_velocity = 0.0f;
	Write_Velocity(AGV_Velocity_InAGV);

	Encoder_Class::Init_Period_TIM();
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


//Coordinate_Class & Mecanum_Wheel_Class::Update_Coor_By_Encoder_demo(const Coordinate_Class Coor_InWorld_Last)
//{
//	float delta_x = 0.0f, delta_y = 0.0f, delta_theta = AGV_Velocity_InAGV.angular_velocity*0.01f;
//
//	float velocity_theta_t_1 = Coor_InWorld_Last.angle_coor + AGV_Velocity_InAGV.velocity_angle;	//t-1时刻的速度方向（世界坐标系）
//	float velocity_theta_t = velocity_theta_t_1 + delta_theta;	//t时刻的速度方向(世界坐标系)(计算周期是20ms)
//
//	float angular_velocity = AGV_Velocity_InAGV.angular_velocity / 180.0*M_PI;	//将角度转换为弧度
//
//	if (ABS(angular_velocity) < FLOAT_DELTA)	//角速度小于阈值，认为是直线运动
//	{
//		delta_x = AGV_Velocity_InAGV.velocity*Cos_Lookup(velocity_theta_t_1)*0.01f;
//		delta_y = AGV_Velocity_InAGV.velocity*Sin_Lookup(velocity_theta_t_1)*0.01f;
//	}
//	else  //圆弧运动
//	{
//		delta_x = AGV_Velocity_InAGV.velocity / angular_velocity*(Sin_Lookup(velocity_theta_t) - Sin_Lookup(velocity_theta_t_1));
//		delta_y = AGV_Velocity_InAGV.velocity / angular_velocity*(Cos_Lookup(velocity_theta_t_1) - Cos_Lookup(velocity_theta_t));
//	}
//	AGV_Coor_InWorld.x_coor += delta_x;
//	AGV_Coor_InWorld.y_coor += delta_y;
//	AGV_Coor_InWorld.angle_coor += delta_theta;
//	return AGV_Coor_InWorld;
//	// TODO: 在此处插入 return 语句
//}
//
//Velocity_Class & Mecanum_Wheel_Class::Update_Velocity_By_ErrorCoor(const Coordinate_Class & Error_Coor_InAGV, Velocity_Class & AGV_Velocity_InAGV)
//{
//	//此处算法应改进
//	//if (ABS(Error_Coor_InAGV.x_coor) > 0.1f)
//	//{
//	//	AGV_Velocity_InAGV.x_velocity += C1*Error_Coor_InAGV.x_coor;
//	//}
//	//if (ABS(Error_Coor_InAGV.y_coor) > 0.1f)
//	//{
//	//	AGV_Velocity_InAGV.y_velocity += C2*Error_Coor_InAGV.y_coor;
//	//}
//	//if (ABS(Error_Coor_InAGV.angle_coor) > 0.1f)
//	//{
//	//	AGV_Velocity_InAGV.angle_velocity += C3*Sin_Lookup(Error_Coor_InAGV.angle_coor);
//	//}
//	return AGV_Velocity_InAGV;
//}
//
////依照麦克纳姆轮的物理限制，更新速度
//Velocity_Class & Mecanum_Wheel_Class::Constraint_Velocity_By_Limit(Velocity_Class & Velocity)
//{
//	//根据麦克纳姆轮的运动学关系式以及不打滑的约束方程
//	//Vx，Vy，(Lx+Ly)W(弧度)的约束面为正八面体的表面,即|Vx|+|Vy|+|(Lx+Ly)W(弧度)|<=V_wheel
//
//	//|Velocity.x_velocity|+|Velocity.y_velocity|+|angular_velocity|<=WHEEL_MAX_LINE_VELOCITY
//	float angular_velocity = Velocity.angular_velocity*M_PI / 180.0f*(DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2.0f;
//	float x_velocity = Velocity.velocity*Cos_Lookup(Velocity.velocity_angle);
//	float y_velocity = Velocity.velocity*Sin_Lookup(Velocity.velocity_angle);
//	float abs_temp = ABS(x_velocity) + ABS(y_velocity) + ABS(angular_velocity);
//
//	float k = 1.0f;
//
//	if (abs_temp > Parameter_Class::wheel_max_line_velocity)
//	{
//		k = Parameter_Class::wheel_max_line_velocity / abs_temp;
//	}
//	//else if (abs_temp < Parameter_Class::wheel_min_line_velocity)
//	//{
//	//	k = Parameter_Class::wheel_min_line_velocity / abs_temp;
//	//}
//	Velocity *= k;
//
//	velocity = abs_temp*k;
//	return Velocity;
//}

//************************************
// Method:    Write_Velocity
// FullName:  Mecanum_Wheel_Class::Write_Velocity
// Access:    public 
// Returns:   void
// Parameter: Position_Class::Velocity_Class & AGV_Velocity_InAGV
// Description: 将AGV速度转换为车轮速度
//************************************
void Mecanum_Wheel_Class::Write_Velocity(Velocity_Class &AGV_Velocity_InAGV)
{
	float duty_FR, duty_FL, duty_BR, duty_BL;
	float k = 1.0f;

	float angle_velocity = AGV_Velocity_InAGV.angular_velocity*Parameter_Class::wheel_lx_ly_distance;

	float x_velocity = AGV_Velocity_InAGV.velocity*Cos_Lookup(AGV_Velocity_InAGV.velocity_angle);
	float y_velocity = AGV_Velocity_InAGV.velocity*Sin_Lookup(AGV_Velocity_InAGV.velocity_angle);

	float velocity = ABS(x_velocity) + ABS(y_velocity) + ABS(angle_velocity);

	if (velocity > Parameter_Class::wheel_max_line_velocity)
	{
		k = (Parameter_Class::wheel_max_line_velocity / velocity);
		AGV_Velocity_InAGV *= k;
	}

	x_velocity *= k;
	y_velocity *= k;
	angle_velocity *= k;

	duty_FR = (-x_velocity + y_velocity + angle_velocity) / Parameter_Class::wheel_max_line_velocity_hard;
	duty_FL = (x_velocity + y_velocity - angle_velocity) / Parameter_Class::wheel_max_line_velocity_hard;
	duty_BL = (-x_velocity + y_velocity - angle_velocity) / Parameter_Class::wheel_max_line_velocity_hard;
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
// Method:    Cal_Velocity_By_Encoder
// FullName:  Mecanum_Wheel_Class::Cal_Velocity_By_Encoder
// Access:    public 
// Returns:   float	离上一次运算的时间间隔
// Parameter: Velocity_Class & AGV_Velocity	车体速度
// Description: 使用编码器计算车身速度
//************************************
float Mecanum_Wheel_Class::Cal_Velocity_By_Encoder(Velocity_Class & AGV_Velocity)
{
	float line_velocity_FR, line_velocity_FL;   //前右，前左轮线速度
	float line_velocity_BL, line_velocity_BR;   //后左，后右轮线速度

	float time_ms = Get_Time_ms(); //获取时间间隔，清除计数器

	Front_Left_Encoder.Get_Pulse(); //读取编码器旋转的脉冲数
	Front_Right_Encoder.Get_Pulse();
	Behind_Left_Encoder.Get_Pulse();
	Behind_Right_Encoder.Get_Pulse();

	//获取轮子的线速度(mm/s)
	line_velocity_FR = Front_Right_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;
	line_velocity_FL = Front_Left_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;
	line_velocity_BR = Behind_Right_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;
	line_velocity_BL = Behind_Left_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;

	//将轮子的线速度转化为车身的速度
	float x_velocity_temp = (-line_velocity_FR + line_velocity_FL - line_velocity_BL + line_velocity_BR) / 4;
	float y_velocity_temp = (line_velocity_FR + line_velocity_FL + line_velocity_BL + line_velocity_BR) / 4;
	float angle_velocity_temp = (line_velocity_FR - line_velocity_FL - line_velocity_BL + line_velocity_BR) / 4;	//此时单位还是mm/s

																													//转化为rad/s
	angle_velocity_temp = angle_velocity_temp / (Parameter_Class::wheel_lx_ly_distance);

	float velocity_temp = sqrtf(x_velocity_temp*x_velocity_temp + y_velocity_temp*y_velocity_temp);	//线速度大小
	float angle_temp = ArcTan_Lookup(x_velocity_temp, y_velocity_temp) / 10.0f;	//线速度方向

	AGV_Velocity.velocity = velocity_temp;
	AGV_Velocity.velocity_angle = angle_temp;
	AGV_Velocity.angular_velocity = angle_velocity_temp;

	GPIOA->ODR ^= 1 << 15;
	return time_ms;
}


float Mecanum_Wheel_Class::Get_Time_ms(void)
{
	float time_10us = Encoder_Class::Read_Time();	//获取时间
	Encoder_Class::Clear_Time_US(); //清空计数器
	return  time_10us / 100.0f;
}

void Mecanum_Wheel_Class::Update_Velocity_By_ErrorCoor(const Coordinate_Class & Error_Coor_InAGV, Velocity_Class Target_Velocity)
{
	//使用Lyapunov方法根据误差更新速度，未完成
}
