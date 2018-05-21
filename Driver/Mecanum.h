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


	//根据物理约束限幅速度,将车体速度转换为轮速，控制电机
	void Write_Velocity(Velocity_Class &AGV_Velocity_InAGV) override;
	//由编码器计算AGV车体速度
	float Cal_Velocity_By_Encoder(Velocity_Class & AGV_Velocity) override;

	//测试程序，随便写的一个计算坐标方法
	Coordinate_Class &Update_Coor_demo(Coordinate_Class &Coor_Current, Velocity_Class&Velocity, float time_s);

private:
	float Get_Time_ms(void);
	//根据跟踪误差更新期望速度
	void Update_Velocity_By_ErrorCoor(const Coordinate_Class&Error_Coor_InAGV, Velocity_Class &Target_Velocity) override;

	static Motor_Class Front_Left_Wheel, Front_Right_Wheel;			//左、右前轮
	static Motor_Class Behind_Left_Wheel, Behind_Right_Wheel;		//左、右后轮
	static Encoder_Class Front_Left_Encoder, Front_Right_Encoder;   //左、右前轮编码器
	static Encoder_Class Behind_Left_Encoder, Behind_Right_Encoder; //左、右后轮编码器


};
