#include "Mecanum.h"
#include "../Math/Trigonometric.h"
#include "../HardwareDefine/Version_Boards.h"
#include <math.h>


#define Velocity_RES 10000 //速度分辨率
////使用Lyapunov方法根据误差更新速度
//#define C1 2.0f
//#define C2 C1
//#define C3 20.0f

Motor_Class Mecanum_Wheel_Class::Front_Right_Wheel = Motor_Class(IO_Class(FRONT_RIGHT_MOTOR_DIR_Port, FRONT_RIGHT_MOTOR_DIR_Pin), \
	IO_Class(FRONT_RIGHT_MOTOR_BRAKE_Port, FRONT_RIGHT_MOTOR_BRAKE_Pin), IO_Class(FRONT_RIGHT_MOTOR_STOP_Port, FRONT_RIGHT_MOTOR_STOP_Pin), \
	FRONT_RIGHT_DEFAULT_DIR, FRONT_RIGHT_MOTOR_TIM);	//前右轮

Motor_Class Mecanum_Wheel_Class::Front_Left_Wheel = Motor_Class(IO_Class(FRONT_LEFT_MOTOR_DIR_Port, FRONT_LEFT_MOTOR_DIR_Pin), \
	IO_Class(FRONT_LEFT_MOTOR_BRAKE_Port, FRONT_LEFT_MOTOR_BRAKE_Pin), IO_Class(FRONT_LEFT_MOTOR_STOP_Port, FRONT_LEFT_MOTOR_STOP_Pin), \
	FRONT_LEFT_DEFAULT_DIR, FRONT_LEFT_MOTOR_TIM);	//前左轮

Motor_Class Mecanum_Wheel_Class::Behind_Right_Wheel = Motor_Class(IO_Class(BEHIND_RIGHT_MOTOR_DIR_Port, BEHIND_RIGHT_MOTOR_DIR_Pin), \
	IO_Class(BEHIND_RIGHT_MOTOR_BRAKE_Port, BEHIND_RIGHT_MOTOR_BRAKE_Pin), IO_Class(BEHIND_RIGHT_MOTOR_STOP_Port, BEHIND_RIGHT_MOTOR_STOP_Pin), \
	BEHIND_RIGHT_DEFAULT_DIR, BEHIND_RIGHT_MOTOR_TIM);	//后右轮

Motor_Class Mecanum_Wheel_Class::Behind_Left_Wheel = Motor_Class(IO_Class(BEHIND_LEFT_MOTOR_DIR_Port, BEHIND_LEFT_MOTOR_DIR_Pin), \
	IO_Class(BEHIND_LEFT_MOTOR_BRAKE_Port, BEHIND_LEFT_MOTOR_BRAKE_Pin), IO_Class(BEHIND_LEFT_MOTOR_STOP_Port, BEHIND_LEFT_MOTOR_STOP_Pin), \
	BEHIND_LEFT_DEFAULT_DIR, BEHIND_LEFT_MOTOR_TIM);	//后左轮

Encoder_Class Mecanum_Wheel_Class::Front_Left_Encoder = Encoder_Class FRONT_LEFT_ENCODER_TIM;	//前左编码器
Encoder_Class Mecanum_Wheel_Class::Front_Right_Encoder = Encoder_Class FRONT_RIGHT_ENCODER_TIM;  //前右编码器
Encoder_Class Mecanum_Wheel_Class::Behind_Left_Encoder = Encoder_Class BEHIND_LEFT_ENCODER_TIM;  //后左编码器
Encoder_Class Mecanum_Wheel_Class::Behind_Right_Encoder = Encoder_Class BEHIND_RIGHT_ENCODER_TIM; //后右编码器

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

	//电机设置
	GPIO_InitStructure.GPIO_Pin = FRONT_LEFT_MOTOR_SPEED_Pin;
	GPIO_Init(FRONT_LEFT_MOTOR_SPEED_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(FRONT_LEFT_MOTOR_SPEED_Port, FRONT_LEFT_MOTOR_SPEED_PinSource, FRONT_LEFT_MOTOR_SPEED_AF);

	GPIO_InitStructure.GPIO_Pin = FRONT_RIGHT_MOTOR_SPEED_Pin;
	GPIO_Init(FRONT_RIGHT_MOTOR_SPEED_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(FRONT_RIGHT_MOTOR_SPEED_Port, FRONT_RIGHT_MOTOR_SPEED_PinSource, FRONT_RIGHT_MOTOR_SPEED_AF);

	GPIO_InitStructure.GPIO_Pin = BEHIND_LEFT_MOTOR_SPEED_Pin;
	GPIO_Init(BEHIND_LEFT_MOTOR_SPEED_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(BEHIND_LEFT_MOTOR_SPEED_Port, BEHIND_LEFT_MOTOR_SPEED_PinSource, BEHIND_LEFT_MOTOR_SPEED_AF);

	GPIO_InitStructure.GPIO_Pin = BEHIND_RIGHT_MOTOR_SPEED_Pin;
	GPIO_Init(BEHIND_RIGHT_MOTOR_SPEED_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(BEHIND_RIGHT_MOTOR_SPEED_Port, BEHIND_RIGHT_MOTOR_SPEED_PinSource, BEHIND_RIGHT_MOTOR_SPEED_AF);

	Front_Left_Wheel.Init(2000, Velocity_RES, FRONT_LEFT_MOTOR_TIM_CHANNEL);
	Front_Right_Wheel.Init(2000, Velocity_RES, FRONT_RIGHT_MOTOR_TIM_CHANNEL);
	Behind_Left_Wheel.Init(2000, Velocity_RES, BEHIND_LEFT_MOTOR_TIM_CHANNEL);
	Behind_Right_Wheel.Init(2000, Velocity_RES, BEHIND_RIGHT_MOTOR_TIM_CHANNEL);


	//编码器设置
	GPIO_InitStructure.GPIO_Pin = FRONT_RIGHT_ENCODER_A_Pin;
	GPIO_Init(FRONT_RIGHT_ENCODER_A_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(FRONT_RIGHT_ENCODER_A_Port, FRONT_RIGHT_ENCODER_A_PinSource, FRONT_RIGHT_ENCODER_A_AF);
	GPIO_InitStructure.GPIO_Pin = FRONT_RIGHT_ENCODER_B_Pin;
	GPIO_Init(FRONT_RIGHT_ENCODER_B_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(FRONT_RIGHT_ENCODER_B_Port, FRONT_RIGHT_ENCODER_B_PinSource, FRONT_RIGHT_ENCODER_B_AF);

	GPIO_InitStructure.GPIO_Pin = FRONT_LEFT_ENCODER_A_Pin;
	GPIO_Init(FRONT_LEFT_ENCODER_A_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(FRONT_LEFT_ENCODER_A_Port, FRONT_LEFT_ENCODER_A_PinSource, FRONT_LEFT_ENCODER_A_AF);
	GPIO_InitStructure.GPIO_Pin = FRONT_LEFT_ENCODER_B_Pin;
	GPIO_Init(FRONT_LEFT_ENCODER_B_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(FRONT_LEFT_ENCODER_B_Port, FRONT_LEFT_ENCODER_B_PinSource, FRONT_LEFT_ENCODER_B_AF);

	GPIO_InitStructure.GPIO_Pin = BEHIND_RIGHT_ENCODER_A_Pin;
	GPIO_Init(BEHIND_RIGHT_ENCODER_A_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(BEHIND_RIGHT_ENCODER_A_Port, BEHIND_RIGHT_ENCODER_A_PinSource, BEHIND_RIGHT_ENCODER_A_AF);
	GPIO_InitStructure.GPIO_Pin = BEHIND_RIGHT_ENCODER_B_Pin;
	GPIO_Init(BEHIND_RIGHT_ENCODER_B_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(BEHIND_RIGHT_ENCODER_B_Port, BEHIND_RIGHT_ENCODER_B_PinSource, BEHIND_RIGHT_ENCODER_B_AF);

	GPIO_InitStructure.GPIO_Pin = BEHIND_LEFT_ENCODER_A_Pin;
	GPIO_Init(BEHIND_LEFT_ENCODER_A_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(BEHIND_LEFT_ENCODER_A_Port, BEHIND_LEFT_ENCODER_A_PinSource, BEHIND_LEFT_ENCODER_A_AF);
	GPIO_InitStructure.GPIO_Pin = BEHIND_LEFT_ENCODER_B_Pin;
	GPIO_Init(BEHIND_LEFT_ENCODER_B_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(BEHIND_LEFT_ENCODER_B_Port, BEHIND_LEFT_ENCODER_B_PinSource, BEHIND_LEFT_ENCODER_B_AF);


	Front_Left_Encoder.Init(FRONT_LEFT_DEFAULT_DIR);
	Front_Right_Encoder.Init(FRONT_RIGHT_DEFAULT_DIR);
	Behind_Left_Encoder.Init(BEHIND_LEFT_DEFAULT_DIR);
	Behind_Right_Encoder.Init(BEHIND_RIGHT_DEFAULT_DIR);

	//AGV_Velocity_InAGV.velocity = 0.0f;
	//AGV_Velocity_InAGV.velocity_angle = 0.0f;
	//AGV_Velocity_InAGV.angular_velocity = 0.0f;
	AGV_Velocity_InAGV.velocity_x = 0.0f;
	AGV_Velocity_InAGV.velocity_y = 0.0f;
	AGV_Velocity_InAGV.angular_velocity_mm = 0.0f;
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

	float velocity_temp = ABS(AGV_Velocity_InAGV.velocity_x) + ABS(AGV_Velocity_InAGV.velocity_y) + ABS(AGV_Velocity_InAGV.angular_velocity_mm);
	if (velocity_temp > Parameter_Class::wheel_max_line_velocity)
	{
		k = (Parameter_Class::wheel_max_line_velocity / velocity_temp);
		AGV_Velocity_InAGV *= k;
	}
	//else if ((velocity_temp < Parameter_Class::wheel_min_line_velocity) && (velocity_temp > Parameter_Class::wheel_min_line_velocity / 10.0f))	//确实存在速度，且速度小于电机响应
	//{
	//	k = (Parameter_Class::wheel_min_line_velocity / velocity_temp);
	//	AGV_Velocity_InAGV *= k;
	//}

	duty_FR = (-AGV_Velocity_InAGV.velocity_x + AGV_Velocity_InAGV.velocity_y + AGV_Velocity_InAGV.angular_velocity_mm) / Parameter_Class::wheel_max_line_velocity_hard;
	duty_FL = (AGV_Velocity_InAGV.velocity_x + AGV_Velocity_InAGV.velocity_y - AGV_Velocity_InAGV.angular_velocity_mm) / Parameter_Class::wheel_max_line_velocity_hard;
	duty_BL = (-AGV_Velocity_InAGV.velocity_x + AGV_Velocity_InAGV.velocity_y - AGV_Velocity_InAGV.angular_velocity_mm) / Parameter_Class::wheel_max_line_velocity_hard;
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
// Returns:   float	离上一次运算的时间间隔(ms)
// Parameter: Velocity_Class & AGV_Velocity	车体速度
// Description: 使用编码器计算车身速度
//************************************
float Mecanum_Wheel_Class::Cal_Velocity_By_Encoder(Velocity_Class & AGV_Velocity)
{
	float line_velocity_FR, line_velocity_FL;   //前右，前左轮线速度
	float line_velocity_BL, line_velocity_BR;   //后左，后右轮线速度

	float time_ms = Get_Time_ms(); //获取时间间隔，清除计数器

	//此处可利用脉冲数直接计算车体的速度，可简化
	Front_Left_Encoder.Get_Pulse(); //读取编码器旋转的脉冲数
	Front_Right_Encoder.Get_Pulse();
	Behind_Left_Encoder.Get_Pulse();
	Behind_Right_Encoder.Get_Pulse();

	//获取轮子的线速度(mm/s)
	line_velocity_FR = Front_Right_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;
	line_velocity_FL = Front_Left_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;
	line_velocity_BR = Behind_Right_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;
	line_velocity_BL = Behind_Left_Encoder.Cal_Angular_Velocity(time_ms) * 1000.0f*MECANUM_WHEEL_DIAMETER / 2;


	AGV_Velocity.velocity_x = (-line_velocity_FR + line_velocity_FL - line_velocity_BL + line_velocity_BR) / 4;
	AGV_Velocity.velocity_y = (line_velocity_FR + line_velocity_FL + line_velocity_BL + line_velocity_BR) / 4;
	AGV_Velocity.angular_velocity_mm = (line_velocity_FR - line_velocity_FL - line_velocity_BL + line_velocity_BR) / 4;
	AGV_Velocity.angular_velocity_rad = AGV_Velocity.angular_velocity_mm / (Parameter_Class::wheel_lx_ly_distance);
	AGV_Velocity.angular_velocity_angle = AGV_Velocity.angular_velocity_rad / M_PI * 180;

	return time_ms;
}

Coordinate_Class & Mecanum_Wheel_Class::Update_Coor_demo(Coordinate_Class & Coor_Current, Velocity_Class & Velocity, float time_s)
{
	Coordinate_Class Coor_Delta;
	Coor_Delta.x_coor = Velocity.velocity_x*time_s;
	Coor_Delta.y_coor = Velocity.velocity_y*time_s;
	Coor_Delta.angle_coor = Velocity.angular_velocity_angle*time_s;

	Coor_Current = Coor_Current + Coor_Delta;

	return Coor_Current;
}


float Mecanum_Wheel_Class::Get_Time_ms(void)
{
	float time_10us = Encoder_Class::Read_Time();	//获取时间
	Encoder_Class::Clear_Time_US(); //清空计数器
	return  time_10us / 100.0f;
}

void Mecanum_Wheel_Class::Update_Velocity_By_ErrorCoor(const Coordinate_Class & Error_Coor_InAGV, Velocity_Class& Target_Velocity, const Coordinate_Class& Base)
{
#define C1 3.0f	
#define C2 C1
#define C3 5.0f

	//Velocity_Class Target_Velocity_InWorld;
	////使用绝对坐标容易计算
	//Target_Velocity_InWorld = Velocity_Class::Relative_To_Absolute(Target_Velocity_InWorld, Target_Velocity, Base);

	//此处算法应改进，修改为和打滑系数相关的C1 C2 C3
	if (ABS(Error_Coor_InAGV.x_coor) > 0.5f)
	{
		Target_Velocity.velocity_x += C1*Error_Coor_InAGV.x_coor;
	}
	if (ABS(Error_Coor_InAGV.y_coor) > 0.5f)
	{
		Target_Velocity.velocity_y += C2*Error_Coor_InAGV.y_coor;
	}
	if (ABS(Error_Coor_InAGV.angle_coor) > 0.5f)
	{
		Target_Velocity.angular_velocity_angle += C3*Sin_Lookup(Error_Coor_InAGV.angle_coor);
	}

	//Target_Velocity = Velocity_Class::Absolute_To_Relative(Target_Velocity_InWorld, Target_Velocity, Base);


	Target_Velocity.angular_velocity_rad = Target_Velocity.angular_velocity_angle / 180.0f*M_PI;
	Target_Velocity.angular_velocity_mm = Target_Velocity.angular_velocity_rad*Parameter_Class::wheel_lx_ly_distance;

#undef C1
#undef C2
#undef C3
}
