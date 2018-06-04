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
	//float32_t x_velocity_source = velocity*Cos_Lookup(this->velocity_angle);	//源向量的x轴速度
	//float32_t y_velocity_source = velocity*Sin_Lookup(this->velocity_angle);	//源向量的y轴速度
	//float32_t x_velocity_addend = velocity*Cos_Lookup(addend.velocity_angle);	//加数的x轴速度
	//float32_t y_velocity_addend = velocity*Sin_Lookup(addend.velocity_angle);	//加数的y轴速度
	//float32_t x_velocity_temp = x_velocity_source + x_velocity_addend;	//新的x轴速度
	//float32_t y_velocity_temp = y_velocity_source + y_velocity_addend;	//新的y轴速度

	velocity_x += addend.velocity_x;
	velocity_y += addend.velocity_y;

	float velocity_temp = velocity_x*velocity_x + velocity_y*velocity_y;
	arm_sqrt_f32(velocity_temp, &(this->velocity));	//计算速度大小
	this->velocity_angle = ArcTan_Lookup(velocity_x, velocity_y) / 10.0f;	//计算速度方向
	angular_velocity_rad += addend.angular_velocity_rad;
	angular_velocity_angle += addend.angular_velocity_angle;
	angular_velocity_mm += addend.angular_velocity_mm;


	//float32_t velocity_temp = x_velocity_temp*x_velocity_temp + y_velocity_temp*y_velocity_temp;
	//arm_sqrt_f32(velocity_temp, &(this->velocity));	//计算速度大小
	//this->velocity_angle = ArcTan_Lookup(x_velocity_temp, y_velocity_temp) / 10.0f;	//计算速度方向
	//this->angular_velocity += addend.angular_velocity;	//角速度大小
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
	//大小相反，角度不变，角速度不变
	//Velocity_Class addend_temp = subtrahend;
	////addend_temp.angular_velocity = addend_temp.angular_velocity;
	//addend_temp.velocity = -addend_temp.velocity;
	//this->operator+=(addend_temp);	//转换为加法
	//return *this;

	velocity_x -= subtrahend.velocity_x;
	velocity_y -= subtrahend.velocity_y;

	float velocity_temp = velocity_x*velocity_x + velocity_y*velocity_y;
	arm_sqrt_f32(velocity_temp, &(this->velocity));	//计算速度大小
	this->velocity_angle = ArcTan_Lookup(velocity_x, velocity_y) / 10.0f;	//计算速度方向
	angular_velocity_rad -= subtrahend.angular_velocity_rad;
	angular_velocity_angle -= subtrahend.angular_velocity_angle;
	angular_velocity_mm -= subtrahend.angular_velocity_mm;
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
	//线速度、角速度乘因子
	velocity_x *= factor;
	velocity_y *= factor;
	velocity *= factor;
	angular_velocity_rad *= factor;
	angular_velocity_angle *= factor;
	angular_velocity_mm *= factor;
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

void Velocity_Class::Clear(void)
{
	velocity_x = 0.0f;
	velocity_y = 0.0f;
	velocity = 0.0f;
	velocity_angle = 0.0f;
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
	const float angle_temp = Base_Coor.angle_coor;

	Absolute_Velocity.velocity_x = cos_Angle * (Relative_Velocity.velocity_x) - sin_Angle * (Relative_Velocity.velocity_y);
	Absolute_Velocity.velocity_y = sin_Angle * (Relative_Velocity.velocity_x) + cos_Angle * (Relative_Velocity.velocity_y);
	Absolute_Velocity.angular_velocity_angle = Relative_Velocity.angular_velocity_angle;



	return Absolute_Velocity;
	// TODO: 在此处插入 return 语句
}

Velocity_Class & Velocity_Class::Absolute_To_Relative(const Velocity_Class & Absolute_Velocity, Velocity_Class & Relative_Velocity, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	Relative_Velocity.velocity_x = cos_Angle * (Absolute_Velocity.velocity_x) + sin_Angle * (Absolute_Velocity.velocity_y);
	Relative_Velocity.velocity_y = (-sin_Angle) * (Absolute_Velocity.velocity_x) + cos_Angle * (Absolute_Velocity.velocity_y);
	Relative_Velocity.angular_velocity_angle = (Absolute_Velocity.angular_velocity_angle);
	// TODO: 在此处插入 return 语句
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

////************************************
//// Method:    operator*=
//// FullName:  Coordinate_Class::operator*=
//// Access:    public 
//// Returns:   Coordinate_Class &
//// Parameter: const float factor
//// Description: 重载*=运算符，只放大x，y
////************************************
//Coordinate_Class &Coordinate_Class::operator*=(const float factor)
//{
//	this->x_coor *= factor;
//	this->y_coor *= factor;
//	return *this;
//}
//
////************************************
//// Method:    operator/=
//// FullName:  Coordinate_Class::operator/=
//// Access:    public 
//// Returns:   Coordinate_Class &
//// Parameter: const float divisor
//// Description: 重载/=运算符，映射为*=
////************************************
//Coordinate_Class &Coordinate_Class::operator/=(const float divisor)
//{
//	this->operator*=(1.0f / divisor);
//	return *this;
//}


void Coordinate_Class::Clear(void)
{
	x_coor = 0.0f;
	y_coor = 0.0f;
	angle_coor = 0.0f;
	angle_rad = 0.0f;
}

void Coordinate_Class::Transform_Angle(void)
{
	angle_coor = Transform_Angle(angle_coor);
	angle_rad = angle_coor / 180.0f*M_PI;
}

//将角度变换至-180~+180
float Coordinate_Class::Transform_Angle(float angle)
{
	int k = (long)(angle) / 360;
	angle = angle - k * 360;

	if (angle <= -180.0f)
	{
		angle += 360.0f;
	}
	else if (angle >= 180.0f)
	{
		angle -= 360.0f;
	}
	return angle;
}

//void Coordinate_Class::Truncation_Coor(void)
//{
//	long angle_temp = ((long)(angle_coor * 10.0f) % 3600);
//	x_coor = (long)(x_coor * 10.0f) / 10.0f;
//	y_coor = (long)(y_coor * 10.0f) / 10.0f;
//	angle_coor = ((angle_temp + 3600) % 3600) / 10.0f;
//	if (angle_coor > 180.0f)
//	{
//		angle_coor -= 360.0f;	//改变范围
//	}
//}

//不符合交换律
Coordinate_Class operator+(const Coordinate_Class & summand, const Coordinate_Class & addend)
{
	Coordinate_Class temp = summand;

	temp = Coordinate_Class::Relative_To_Absolute(temp, addend, summand);

	return temp;
}

Coordinate_Class operator-(const Coordinate_Class & minuend, const Coordinate_Class & subtrahend)
{
	Coordinate_Class temp;

	Coordinate_Class::Absolute_To_Relative(minuend, temp, subtrahend);

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

	Absolute_Coor.Transform_Angle();

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

	Relative_Coor.x_coor = cos_Angle * (Absolute_Coor.x_coor - x_temp) + sin_Angle * (Absolute_Coor.y_coor - y_temp);
	Relative_Coor.y_coor = (-sin_Angle) * (Absolute_Coor.x_coor - x_temp) + cos_Angle * (Absolute_Coor.y_coor - y_temp);
	Relative_Coor.angle_coor = (Absolute_Coor.angle_coor - angle_temp);

	//if (Relative_Coor.angle_coor > 180.0f)
	//{
	//	Relative_Coor.angle_coor -= 360.0f;	//改变范围
	//}

	Relative_Coor.Transform_Angle();

	return Relative_Coor;
}
