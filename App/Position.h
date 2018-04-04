#pragma once
#include "../Math/Trigonometric.h"

class Position_Class
{
  public:
	class Velocity_Class
	{
	  public:
		float x_velocity;   //x轴速度,mm/s
		float y_velocity;   //y轴速度,mm/s
		float yaw_velocity; //旋转角速度,°/s
		//Velocity_StructTypedef() :x_velocity(0.0f), y_velocity(0.0f), yaw_velocity(0.0f) {};
		Velocity_Class &operator+=(const Velocity_Class &addend);
		Velocity_Class &operator-=(const Velocity_Class &subtrahend);
	}; //速度

	class Coordinate_Class
	{
	  public:
		float x_coor;	 //x轴坐标(mm)，精确到0.1mm
		float y_coor;	 //y轴坐标(mm)，精确到0.1mm
		float angle_coor; //角度坐标(°),精确到0.1°
		//Coordinate_StructTypedef() :x_coor(0.0f), y_coor(0.0f), angle_coor(0.0f) {};
		Coordinate_Class &operator+=(const Coordinate_Class &addend);
		Coordinate_Class &operator-=(const Coordinate_Class &subtrahend);
	}; //坐标

	Velocity_Class Velocity;	 //速度
	Coordinate_Class Coordinate; //坐标

	Position_Class &operator+=(const Position_Class &addend);
	Position_Class &operator-=(const Position_Class &subtrahend);

	static Coordinate_Class &Absolute_To_Relative(const Coordinate_Class &Absolute_Coor, Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor); //绝对坐标转换为相对坐标
	static Coordinate_Class &Relative_To_Absolute(Coordinate_Class &Absolute_Coor, const Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor); //相对坐标转换为绝对坐标

	static Coordinate_Class &Truncation_Coor(Coordinate_Class &Source_Coor);

  private:
};

static Position_Class::Velocity_Class operator+(const Position_Class::Velocity_Class &summand, const Position_Class::Velocity_Class &addend);
static Position_Class::Velocity_Class operator-(const Position_Class::Velocity_Class &minuend, const Position_Class::Velocity_Class &subtrahend);
static Position_Class::Coordinate_Class operator+(const Position_Class::Coordinate_Class &summand, const Position_Class::Coordinate_Class &addend);
static Position_Class::Coordinate_Class operator-(const Position_Class::Coordinate_Class &minuend, const Position_Class::Coordinate_Class &subtrahend);
static Position_Class operator+(const Position_Class &summand, const Position_Class &addend);
static Position_Class operator-(const Position_Class &minuend, const Position_Class &subtrahend);
