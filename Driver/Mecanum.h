#pragma once
/*
* 使用麦克纳姆轮的驱动类
*/
#include "AGV_Control.h"
#include "Motor.h"
#include "Encoder.h"
#include "../Configure.h"
#include "../macros.h"
#include "../parameter_define.h"

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

	Velocity_Class & Update_Velocity_By_ErrorCoor(const Coordinate_Class&Error_Coor_InAGV, Velocity_Class &AGV_Velocity_InAGV) override;	//根据位姿误差更新期望速度
	Velocity_Class& Update_Velocity_By_Limit(Velocity_Class&Velocity) override;	//对速度限幅
	void Write_Velocity(Velocity_Class &AGV_Velocity_InAGV) override;	//将AGV速度转换为车轮速度，更新velocity
	Velocity_Class &Cal_Velocity_By_Encoder(void) override;	//根据编码器获取AGV速度

private:
	static Motor_Class Front_Left_Wheel, Front_Right_Wheel;			//左、右前轮
	static Motor_Class Behind_Left_Wheel, Behind_Right_Wheel;		//左、右后轮
	static Encoder_Class Front_Left_Encoder, Front_Right_Encoder;   //左、右前轮编码器
	static Encoder_Class Behind_Left_Encoder, Behind_Right_Encoder; //左、右后轮编码器


};
