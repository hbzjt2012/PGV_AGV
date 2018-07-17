#include "Position.h"

void Coordinate_Class::Clear(void)
{
	x_coor = 0.0f;
	y_coor = 0.0f;
	angle_coor = 0.0f;
	angle_rad = 0.0f;
}

//将角度转换至base的同一个周期内(+180~-180)
void Coordinate_Class::Coor_Trans(const float base_angle)
{
	angle_coor = Angle_Trans(angle_coor, base_angle);
	Angle2Rad();
}

//基坐标为*this,相对坐标为addend，求绝对坐标
Coordinate_Class & Coordinate_Class::operator+=(const Coordinate_Class & addend)
{
	Coordinate_Class temp;
	Coordinate_Class::Relative_To_Absolute(temp, addend, *this);
	*this = temp;
	return *this;
}

//基坐标为subtrahend，绝对坐标为*this，求相对坐标
Coordinate_Class & Coordinate_Class::operator-=(const Coordinate_Class & subtrahend)
{
	Coordinate_Class temp;
	Coordinate_Class::Absolute_To_Relative(*this, temp, subtrahend);
	*this = temp;
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

	Absolute_Coor.Angle2Rad();
	//Absolute_Coor.Coor_Trans(angle_temp);	//转换角度

	return Absolute_Coor;
}

Coordinate_Class & Coordinate_Class::Absolute_To_Relative(const Coordinate_Class & Absolute_Coor, Coordinate_Class & Relative_Coor, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	const float x_temp = Base_Coor.x_coor;
	const float y_temp = Base_Coor.y_coor;
	const float angle_temp = Base_Coor.angle_coor;

	Relative_Coor.x_coor = cos_Angle * (Absolute_Coor.x_coor - x_temp) + sin_Angle * (Absolute_Coor.y_coor - y_temp);
	Relative_Coor.y_coor = (-sin_Angle) * (Absolute_Coor.x_coor - x_temp) + cos_Angle * (Absolute_Coor.y_coor - y_temp);
	Relative_Coor.angle_coor = Absolute_Coor.angle_coor - angle_temp;

	Relative_Coor.Angle2Rad();
	//Relative_Coor.Coor_Trans(angle_temp);

	return Relative_Coor;
}

//************************************
// Method:    Angle_Trans
// FullName:  Coordinate_Class::Angle_Trans
// Access:    public static 
// Returns:   float
// Parameter: float angle
// Parameter: const float base
// Description: 将angle缩放至base同一周期内(+180°~-180°)
//************************************
float Coordinate_Class::Angle_Trans(float angle, const float base)
{
	int k = (int)((angle - base) / 360.0f);
	angle -= k * 360;
	if (angle - base >= 180.0f)
	{
		angle -= 360.0f;
	}
	else if (angle - base < -180.0f)
	{
		angle += 360.0f;
	}
	return angle;
}

//************************************
// Method:    operator+=
// FullName:  Velocity_Class::operator+=
// Access:    public 
// Returns:   Velocity_Class &
// Parameter: const Velocity_Class & addend
// Description: 求两个速度合
//************************************
Velocity_Class &Velocity_Class::operator+=(const Velocity_Class &addend)
{
	velocity_x += addend.velocity_x;
	velocity_y += addend.velocity_y;
	angular_velocity_rad += addend.angular_velocity_rad;
	angular_velocity_angle += addend.angular_velocity_angle;
	angular_velocity_mm += addend.angular_velocity_mm;

	return *this;
}

//求两速度差
Velocity_Class & Velocity_Class::operator-=(const Velocity_Class & subtrahend)
{
	velocity_x -= subtrahend.velocity_x;
	velocity_y -= subtrahend.velocity_y;
	angular_velocity_rad -= subtrahend.angular_velocity_rad;
	angular_velocity_angle -= subtrahend.angular_velocity_angle;
	angular_velocity_mm -= subtrahend.angular_velocity_mm;

	return *this;
}

Velocity_Class & Velocity_Class::operator*=(const float factor)
{
	//线速度、角速度乘因子
	velocity_x *= factor;
	velocity_y *= factor;
	angular_velocity_rad *= factor;
	angular_velocity_angle *= factor;
	angular_velocity_mm *= factor;
	return *this;
}

Velocity_Class & Velocity_Class::operator/=(const float divisor)
{
	this->operator*=(1.0f / divisor);
	return *this;
}

void Velocity_Class::Clear(void)
{
	velocity_x = 0.0f;
	velocity_y = 0.0f;
	angular_velocity_angle = 0.0f;
	angular_velocity_mm = 0.0f;
	angular_velocity_rad = 0.0f;
}

Velocity_Class & Velocity_Class::Relative_To_Absolute(Velocity_Class & Absolute_Velocity, const Velocity_Class & Relative_Velocity, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);
	const float x_temp = Base_Coor.x_coor;
	const float y_temp = Base_Coor.y_coor;

	Absolute_Velocity.velocity_x = cos_Angle * (Relative_Velocity.velocity_x) - sin_Angle * (Relative_Velocity.velocity_y);
	Absolute_Velocity.velocity_y = sin_Angle * (Relative_Velocity.velocity_x) + cos_Angle * (Relative_Velocity.velocity_y);
	Absolute_Velocity.angular_velocity_angle = Relative_Velocity.angular_velocity_angle;
	Absolute_Velocity.angular_velocity_rad = Relative_Velocity.angular_velocity_rad;
	Absolute_Velocity.angular_velocity_mm = Relative_Velocity.angular_velocity_mm;

	return Absolute_Velocity;
}

Velocity_Class & Velocity_Class::Absolute_To_Relative(const Velocity_Class & Absolute_Velocity, Velocity_Class & Relative_Velocity, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	Relative_Velocity.velocity_x = cos_Angle * (Absolute_Velocity.velocity_x) + sin_Angle * (Absolute_Velocity.velocity_y);
	Relative_Velocity.velocity_y = (-sin_Angle) * (Absolute_Velocity.velocity_x) + cos_Angle * (Absolute_Velocity.velocity_y);
	Relative_Velocity.angular_velocity_angle = (Absolute_Velocity.angular_velocity_angle);
	Relative_Velocity.angular_velocity_rad = (Absolute_Velocity.angular_velocity_rad);
	Relative_Velocity.angular_velocity_mm = (Absolute_Velocity.angular_velocity_mm);
}

Velocity_Class operator+(const Velocity_Class & summand, const Velocity_Class & addend)
{
	Velocity_Class temp = summand;
	temp += addend;
	return temp;
}

Velocity_Class operator-(const Velocity_Class & minuend, const Velocity_Class & subtrahend)
{
	Velocity_Class temp = minuend;
	temp -= subtrahend;
	return temp;
}

//不符合交换律
Coordinate_Class operator+(const Coordinate_Class & summand, const Coordinate_Class & addend)
{
	Coordinate_Class temp = summand;
	temp += addend;
	return temp;
}

Coordinate_Class operator-(const Coordinate_Class & minuend, const Coordinate_Class & subtrahend)
{
	Coordinate_Class temp = minuend;
	temp -= subtrahend;
	return temp;
}

Coordinate_Class operator*(const Coordinate_Class & source, const float factor)
{
	Coordinate_Class temp;
	temp.x_coor = source.x_coor*factor;
	temp.y_coor = source.y_coor*factor;
	temp.angle_coor = source.angle_coor*factor;
	temp.angle_rad = source.angle_rad*factor;
	return temp;
}