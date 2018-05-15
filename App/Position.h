#pragma once
#include "../Math/Trigonometric.h"
#include "../DSP_Lib/arm_math.h"

class Velocity_Class
{
public:
	//速度为一个3*1的列向量
	Velocity_Class(void) :velocity(0.0f), velocity_angle(0.0f), angular_velocity(0.0f) {
		velocity_matrix.numRows = 3;
		velocity_matrix.numCols = 1;
		velocity_matrix.pData = velocity_array;
	}
	union
	{
		struct
		{
			float32_t velocity;			//线速度速度,mm/s
			float32_t velocity_angle;   //线速度与x轴的夹角 °
			float32_t angular_velocity; //旋转角速度,°/s
		};
		float32_t velocity_array[3];
	};

	arm_matrix_instance_f32 velocity_matrix;	//速度矩阵(3*1列向量)

	Velocity_Class &operator+=(const Velocity_Class &addend);
	Velocity_Class &operator-=(const Velocity_Class &subtrahend);
	Velocity_Class &operator*=(const float factor);
	Velocity_Class &operator/=(const float divisor);

}; //速度

class Coordinate_Class
{
public:
	//坐标为一个3*1的列向量
	Coordinate_Class(void) :x_coor(0.0f), y_coor(0.0f), angle_coor(0.0f) {
		coor_matrix.numRows = 3;
		coor_matrix.numCols = 1;
		coor_matrix.pData = coor_array;
	}
	//坐标为一个3*1的列向量
	Coordinate_Class(float x, float y, float angle) :x_coor(x), y_coor(y), angle_coor(angle) {
		coor_matrix.numRows = 3;
		coor_matrix.numCols = 1;
		coor_matrix.pData = coor_array;
	}
	union
	{
		struct
		{
			float32_t x_coor;	//x坐标(mm)
			float32_t y_coor;   //y坐标(mm)
			float32_t angle_coor; //角度坐标(°)
		};
		float32_t coor_array[3];
	};

	arm_matrix_instance_f32 coor_matrix;	//速度矩阵(3*1列向量)

	Coordinate_Class &operator*=(const float factor);
	Coordinate_Class &operator/=(const float divisor);

	void Clear(void);
	void Truncation_Coor(void);	//将角度变换至0-360°，保留0.1精度

	static Coordinate_Class &Relative_To_Absolute(Coordinate_Class &Absolute_Coor, const Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor); //相对坐标转换为绝对坐标
	static Coordinate_Class& Absolute_To_Relative(const Coordinate_Class &Absolute_Coor, Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor);	//从绝对坐标转换为相对坐标
}; //坐标


Velocity_Class operator+(const Velocity_Class &summand, const Velocity_Class &addend);
Velocity_Class operator-(const Velocity_Class &minuend, const Velocity_Class &subtrahend);
Coordinate_Class operator+(const Coordinate_Class &summand, const Coordinate_Class &addend);
Coordinate_Class operator-(const Coordinate_Class &minuend, const Coordinate_Class &subtrahend);
