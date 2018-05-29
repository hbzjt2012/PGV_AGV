#pragma once
//加速度计
//获取平面上的x，y加速度
class Accelerometer_Class
{
public:
	Accelerometer_Class() = default;
	virtual ~Accelerometer_Class() = default;

	virtual void Cal_Accelerometer_Data(void) = 0;	//计算加速度数据
	float forward_accel;	//前向加速度(mm/s2)
	float transverse_accel;	//横向加速度(mm/s2)

protected:

};