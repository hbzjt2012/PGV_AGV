#pragma once
#include "../Math/Trigonometric.h"
#include "../DSP_Lib/arm_math.h"

class Coordinate_Class
{
public:
	Coordinate_Class(void) {};

	float x_coor;	//x坐标(mm)
	float y_coor;   //y坐标(mm)
	float angle_coor; //角度坐标(°)
	float angle_rad;	//角度坐标(弧度)

	void Clear(void);	//坐标清零
	float Rad2Angle(void) { angle_coor = angle_rad / M_PI*180.0f; return angle_coor; }	//弧度转角度
	float Angle2Rad(void) { angle_rad = angle_coor / 180.0f*M_PI; return angle_rad; }	//角度转弧度
	void Coor_Trans(const float base_angle);	//转换角度

	Coordinate_Class &operator+=(const Coordinate_Class &addend);	//被加数坐标系中，相对坐标为加数的  绝对坐标
	Coordinate_Class &operator-=(const Coordinate_Class &subtrahend);	//减数坐标系中，绝对坐标为被减数的  相对坐标

	//相对坐标转换为绝对坐标
	static Coordinate_Class &Relative_To_Absolute(Coordinate_Class &Absolute_Coor, const Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor);
	//从绝对坐标转换为相对坐标
	static Coordinate_Class& Absolute_To_Relative(const Coordinate_Class &Absolute_Coor, Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor);
	//将输入角度转换至base角度-180°~+180°范围内
	static float Angle_Trans(float angle, const float base);
}; //坐标

class Velocity_Class
{
public:
	Velocity_Class(void) {}

	float velocity_x;	//x方向速度 mm/s
	float velocity_y;	//y方向速度 mm/s
	float angular_velocity_rad; //旋转角速度,rad/s
	float angular_velocity_angle;	//旋转角速度 °/s
	float angular_velocity_mm;	//旋转角速度，转化为线速度 mm/s

	Velocity_Class &operator+=(const Velocity_Class &addend);
	Velocity_Class &operator-=(const Velocity_Class &subtrahend);
	Velocity_Class &operator*=(const float factor);
	Velocity_Class &operator/=(const float divisor);

	void Clear(void);
	float Rad2Angle(void) { angular_velocity_angle = angular_velocity_rad / M_PI*180.0f; return angular_velocity_angle; }	//弧度转角度
	float Angle2Rad(void) { angular_velocity_rad = angular_velocity_angle / 180.0f*M_PI; return angular_velocity_angle; }	//角度转弧度

	static Velocity_Class &Relative_To_Absolute(Velocity_Class &Absolute_Velocity, const Velocity_Class &Relative_Velocity, const Coordinate_Class &Base_Coor); //从相对速度转换为绝对速度
	static Velocity_Class &Absolute_To_Relative(const Velocity_Class &Absolute_Velocity, Velocity_Class &Relative_Velocity, const Coordinate_Class &Base_Coor);	//从绝对速度转换为相对速度

}; //速度


Velocity_Class operator+(const Velocity_Class &summand, const Velocity_Class &addend);
Velocity_Class operator-(const Velocity_Class &minuend, const Velocity_Class &subtrahend);
Coordinate_Class operator+(const Coordinate_Class &summand, const Coordinate_Class &addend);
Coordinate_Class operator-(const Coordinate_Class &minuend, const Coordinate_Class &subtrahend);
Coordinate_Class operator*(const Coordinate_Class &source, const float factor);
