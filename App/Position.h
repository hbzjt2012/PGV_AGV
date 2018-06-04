#pragma once
#include "../Math/Trigonometric.h"
#include "../DSP_Lib/arm_math.h"

class Coordinate_Class
{
public:
	Coordinate_Class(void) {};

	float x_coor;	//x坐标(mm)
	float y_coor;   //y坐标(mm)
	float angle_coor; //角度坐标(-180°~+180°)
	float angle_rad;	//角度坐标(弧度)

	void Clear(void);
	void Transform_Angle(void);	//将角度转换为-180~+180，也修改弧度
	static float Transform_Angle(float angle);//将角度转换为-180~+180
											  //void Truncation_Coor(void);	//将角度变换至-180~+180°，坐标保留0.1精度

	static Coordinate_Class &Relative_To_Absolute(Coordinate_Class &Absolute_Coor, const Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor); //相对坐标转换为绝对坐标
	static Coordinate_Class& Absolute_To_Relative(const Coordinate_Class &Absolute_Coor, Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor);	//从绝对坐标转换为相对坐标
}; //坐标

class Velocity_Class
{
public:
	Velocity_Class(void) {}

	float velocity_x;	//x方向速度 mm/s
	float velocity_y;	//y方向速度 mm/s
	float velocity;			//线速度速度,mm/s
	float velocity_angle;   //线速度与x轴的夹角 °
	float angular_velocity_rad; //旋转角速度,rad/s
	float angular_velocity_angle;	//旋转角速度 °/s
	float angular_velocity_mm;	//旋转角速度，转化为线速度 mm/s

	Velocity_Class &operator+=(const Velocity_Class &addend);
	Velocity_Class &operator-=(const Velocity_Class &subtrahend);
	Velocity_Class &operator*=(const float factor);
	Velocity_Class &operator/=(const float divisor);

	void Clear(void);

	static Velocity_Class &Relative_To_Absolute(Velocity_Class &Absolute_Velocity, const Velocity_Class &Relative_Velocity, const Coordinate_Class &Base_Coor); //从相对速度转换为绝对速度
	static Velocity_Class &Absolute_To_Relative(const Velocity_Class &Absolute_Velocity, Velocity_Class &Relative_Velocity, const Coordinate_Class &Base_Coor);	//从绝对速度转换为相对速度

}; //速度


Velocity_Class operator+(const Velocity_Class &summand, const Velocity_Class &addend);
Velocity_Class operator-(const Velocity_Class &minuend, const Velocity_Class &subtrahend);
Coordinate_Class operator+(const Coordinate_Class &summand, const Coordinate_Class &addend);
Coordinate_Class operator-(const Coordinate_Class &minuend, const Coordinate_Class &subtrahend);
Coordinate_Class operator*(const Coordinate_Class &source, const float factor);
