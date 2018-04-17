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

	void Write_Velocity(Position_Class::Velocity_Class &AGV_Velocity_InAGV) override;	//将AGV速度转换为车轮速度，更新velocity
	Position_Class &Update_Post_By_Encoder(Position_Class &Current_InWorld) override; //根据编码器更新世界坐标系下的坐标和速度
	

private:
	static Motor_Class Front_Left_Wheel, Front_Right_Wheel;			//左、右前轮
	static Motor_Class Behind_Left_Wheel, Behind_Right_Wheel;		//左、右后轮
	static Encoder_Class Front_Left_Encoder, Front_Right_Encoder;   //左、右前轮编码器
	static Encoder_Class Behind_Left_Encoder, Behind_Right_Encoder; //左、右后轮编码器

	
};
