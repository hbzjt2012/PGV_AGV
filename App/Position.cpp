#include "Position.h"

//************************************
// Method:    operator+=
// FullName:  Velocity_Class::operator+=
// Access:    public 
// Returns:   Velocity_Class &
// Parameter: const Velocity_Class & addend
// Description: 重载+=运算符，计算两个向量的和
//************************************
Velocity_Class &Velocity_Class::operator+=(const Velocity_Class &addend)
{
	float32_t x_velocity_source = velocity*Cos_Lookup(this->velocity_angle);	//源向量的x轴速度
	float32_t y_velocity_source = velocity*Sin_Lookup(this->velocity_angle);	//源向量的y轴速度
	float32_t x_velocity_addend = velocity*Cos_Lookup(addend.velocity_angle);	//加数的x轴速度
	float32_t y_velocity_addend = velocity*Sin_Lookup(addend.velocity_angle);	//加数的y轴速度
	float32_t x_velocity_temp = x_velocity_source + x_velocity_addend;	//新的x轴速度
	float32_t y_velocity_temp = y_velocity_source + y_velocity_addend;	//新的y轴速度

	float32_t velocity_temp = x_velocity_temp*x_velocity_temp + y_velocity_temp*y_velocity_temp;
	arm_sqrt_f32(velocity_temp, &(this->velocity));	//计算速度大小
	this->velocity_angle = ArcTan_Lookup(x_velocity_temp, y_velocity_temp) / 10.0f;	//计算速度方向
	this->angular_velocity += addend.angular_velocity;	//角速度大小
	return *this;
}

//************************************
// Method:    operator-=
// FullName:  Velocity_Class::operator-=
// Access:    public 
// Returns:   Velocity_Class &
// Parameter: const Velocity_Class & subtrahend
// Description: 重载-=运算符，转化为+=
//************************************
Velocity_Class &Velocity_Class::operator-=(const Velocity_Class &subtrahend)
{
	//转化为加法
	Velocity_Class addend_temp = subtrahend;
	addend_temp.angular_velocity = -addend_temp.angular_velocity;
	addend_temp.velocity = -addend_temp.velocity;
	this->operator+=(addend_temp);	//转换为加法
	return *this;
}

//************************************
// Method:    operator*=
// FullName:  Velocity_Class::operator*=
// Access:    public 
// Returns:   Velocity_Class&
// Parameter: const float factor
// Description: 重载*=运算符
//************************************
Velocity_Class&Velocity_Class::operator*=(const float factor)
{
	this->angular_velocity *= factor;
	this->velocity *= factor;
	return *this;
}

//************************************
// Method:    operator/=
// FullName:  Velocity_Class::operator/=
// Access:    public 
// Returns:   Velocity_Class &
// Parameter: const float divisor
// Description: 重载/=运算符，转化为*=处理
//************************************
Velocity_Class &Velocity_Class::operator/=(const float divisor)
{
	this->operator*=(1.0f / divisor);
	return *this;
}


//************************************
// Method:    operator+=
// FullName:  Coordinate_Class::operator+=
// Access:    public 
// Returns:   Coordinate_Class & 在世界坐标系中的坐标
// Parameter: const Coordinate_Class & addend_relative 加数，在*this坐标系中的相对坐标
// Description: 重载+=运算符，映射为坐标变换，从相对坐标（加数相对于被加数）变换为绝对坐标（加数）
//************************************
Coordinate_Class &Coordinate_Class::operator+=(const Coordinate_Class &addend_relative)
{
	Coordinate_Class::Relative_To_Absolute(*this, addend_relative, *this);
	return *this;
}

//************************************
// Method:    operator-=
// FullName:  Coordinate_Class::operator-=
// Access:    public 
// Returns:   Coordinate_Class & 在*this坐标系中的相对坐标
// Parameter: const Coordinate_Class & subtrahend_absolute 减数，世界坐标系中的坐标
// Description: 重载为-=运算符，映射坐标变换，从绝对坐标（减数）变换为相对坐标（相对于被减数）
//************************************
Coordinate_Class &Coordinate_Class::operator-=(const Coordinate_Class &subtrahend_absolute)
{
	Coordinate_Class::Absolute_To_Relative(subtrahend_absolute, *this, *this);
	return *this;
}

//************************************
// Method:    operator*=
// FullName:  Coordinate_Class::operator*=
// Access:    public 
// Returns:   Coordinate_Class &
// Parameter: const float factor
// Description: 重载*=运算符，只放大x，y
//************************************
Coordinate_Class &Coordinate_Class::operator*=(const float factor)
{
	this->x_coor *= factor;
	this->y_coor *= factor;
	return *this;
}

//************************************
// Method:    operator/=
// FullName:  Coordinate_Class::operator/=
// Access:    public 
// Returns:   Coordinate_Class &
// Parameter: const float divisor
// Description: 重载/=运算符，映射为*=
//************************************
Coordinate_Class &Coordinate_Class::operator/=(const float divisor)
{
	this->operator*=(1.0f / divisor);
	return *this;
}

Coordinate_Class & Coordinate_Class::Relative_To_Absolute(Coordinate_Class & Absolute_Coor, const Coordinate_Class & Relative_Coor, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);
	const float x_temp = Base_Coor.x_coor;
	const float y_temp = Base_Coor.y_coor;
	const float angle_temp = Base_Coor.angle_coor;

	Absolute_Coor.x_coor = cos_Angle * (Relative_Coor.x_coor) - sin_Angle * (Relative_Coor.y_coor) + x_temp;
	Absolute_Coor.y_coor = sin_Angle * (Relative_Coor.x_coor) + cos_Angle * (Relative_Coor.y_coor) + y_temp;
	Absolute_Coor.angle_coor = Relative_Coor.angle_coor + angle_temp;

	return Absolute_Coor;
}

//************************************
// Method:    Absolute_To_Relative
// FullName:  Coordinate_Class::Absolute_To_Relative
// Access:    public static 
// Returns:   Coordinate_Class &
// Parameter: const Coordinate_Class & Absolute_Coor
// Parameter: Coordinate_Class & Relative_Coor
// Parameter: const Coordinate_Class & Base_Coor
// Description: 从绝对坐标转换为相对坐标
//************************************
Coordinate_Class & Coordinate_Class::Absolute_To_Relative(const Coordinate_Class & Absolute_Coor, Coordinate_Class & Relative_Coor, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	const float x_temp = Base_Coor.x_coor;
	const float y_temp = Base_Coor.y_coor;
	const float angle_temp = Base_Coor.angle_coor;

	Relative_Coor.x_coor = cos_Angle * (Absolute_Coor.x_coor - x_temp) + sin_Angle * (Absolute_Coor.y_coor - Base_Coor.y_coor);
	Relative_Coor.y_coor = (-sin_Angle) * (Absolute_Coor.x_coor - x_temp) + cos_Angle * (Absolute_Coor.y_coor - Base_Coor.y_coor);
	Relative_Coor.angle_coor = (Absolute_Coor.angle_coor - angle_temp);

	return Relative_Coor;
}
