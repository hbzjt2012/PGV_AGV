#pragma once
/*
* 使用麦克纳姆轮的驱动类
*/
#include "AGV_Control.h"
#include "Motor.h"
#include "Encoder.h"
#include "../Configure.h"
#include "../macros.h"

/*
* 麦克纳姆轮正逆运动学模型见公式计算.doc
*/

class Mecanum_Wheel_Class final : public AGV_Control_Class
{
  public:
	Mecanum_Wheel_Class() = default;
	~Mecanum_Wheel_Class() = default;

	void Init(void) override;
	void Brake(bool value) override;
	void Run(bool value) override;

	bool demo(const Position_Class::Velocity_Class &Target_velocity, float &pulse, float &time);

	void Write_Velocity(Position_Class::Velocity_Class &velocity_InAGV) override;												   //根据输入速度控制小车
	Position_Class &Update_Post(Position_Class &Current_Position, const Position_Class::Velocity_Class &Target_velocity) override; //更新坐标和速度

  private:
	static Motor_Class Front_Left_Wheel, Front_Right_Wheel;			//左、右前轮
	static Motor_Class Behind_Left_Wheel, Behind_Right_Wheel;		//左、右后轮
	static Encoder_Class Front_Left_Encoder, Front_Right_Encoder;   //左、右前轮编码器
	static Encoder_Class Behind_Left_Encoder, Behind_Right_Encoder; //左、右后轮编码器
};
