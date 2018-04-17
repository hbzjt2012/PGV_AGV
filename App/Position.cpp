#include "Position.h"

Position_Class::Velocity_Class &Position_Class::Velocity_Class::operator+=(const Velocity_Class &addend)
{
	x_velocity += addend.x_velocity;
	y_velocity += addend.y_velocity;
	angle_velocity += addend.angle_velocity;
	return *this;
}

Position_Class::Velocity_Class &Position_Class::Velocity_Class::operator-=(const Velocity_Class &subtrahend)
{
	x_velocity -= subtrahend.x_velocity;
	y_velocity -= subtrahend.y_velocity;
	angle_velocity -= subtrahend.angle_velocity;
	return *this;
}

Position_Class::Coordinate_Class &Position_Class::Coordinate_Class::operator+=(const Coordinate_Class &addend)
{
	x_coor += addend.x_coor;
	y_coor += addend.y_coor;
	angle_coor += addend.angle_coor;
	return *this;
}

Position_Class::Coordinate_Class &Position_Class::Coordinate_Class::operator-=(const Coordinate_Class &subtrahend)
{
	x_coor -= subtrahend.x_coor;
	y_coor -= subtrahend.y_coor;
	angle_coor -= subtrahend.angle_coor;
	return *this;
}

Position_Class &Position_Class::operator+=(const Position_Class &addend)
{
	Velocity += addend.Velocity;
	Coordinate += addend.Coordinate;
	return *this;
}

Position_Class &Position_Class::operator-=(const Position_Class &subtrahend)
{
	Velocity -= subtrahend.Velocity;
	Coordinate -= subtrahend.Coordinate;
	return *this;
}



Position_Class::Velocity_Class & Position_Class::Absolute_To_Relative(const Velocity_Class & Absolute_Velocity, Velocity_Class & Relative_Velocity, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	Relative_Velocity.x_velocity = cos_Angle * (Absolute_Velocity.x_velocity) + sin_Angle * (Absolute_Velocity.y_velocity);
	Relative_Velocity.y_velocity = (-sin_Angle) * (Absolute_Velocity.x_velocity) + cos_Angle * (Absolute_Velocity.y_velocity);
	Relative_Velocity.angle_velocity = (Absolute_Velocity.angle_velocity);

	return Relative_Velocity;
}

//************************************
// Method:    Absolute_To_Relative
// FullName:  Position_Class::Absolute_To_Relative
// Access:    public static
// Returns:   Position_Class::Coordinate_StructTypedef &
// Parameter: const Coordinate_StructTypedef & Absolute_Coor
// Parameter: Coordinate_StructTypedef & Relative_Coor
// Parameter: const Coordinate_StructTypedef & Base_Coor 相对坐标系在绝对坐标系中的坐标
// Description: 从绝对坐标变化为相对坐标
//************************************
Position_Class::Coordinate_Class &Position_Class::Absolute_To_Relative(const Coordinate_Class &Absolute_Coor, Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	Relative_Coor.x_coor = cos_Angle * (Absolute_Coor.x_coor - Base_Coor.x_coor) + sin_Angle * (Absolute_Coor.y_coor - Base_Coor.y_coor);
	Relative_Coor.y_coor = (-sin_Angle) * (Absolute_Coor.x_coor - Base_Coor.x_coor) + cos_Angle * (Absolute_Coor.y_coor - Base_Coor.y_coor);
	Relative_Coor.angle_coor = (Absolute_Coor.angle_coor - Base_Coor.angle_coor);

	return Relative_Coor;
	// TODO: 在此处插入 return 语句
}

Position_Class & Position_Class::Absolute_To_Relative(const Position_Class & Absolute_Position, Position_Class & Relative_Position, const Coordinate_Class & Base_Coor)
{
	Relative_Position.Coordinate = Absolute_To_Relative(Absolute_Position.Coordinate, Relative_Position.Coordinate, Base_Coor);
	Relative_Position.Velocity = Absolute_To_Relative(Absolute_Position.Velocity, Relative_Position.Velocity, Base_Coor);
	return Relative_Position;
	// TODO: 在此处插入 return 语句
}

Position_Class::Velocity_Class & Position_Class::Relative_To_Absolute(Velocity_Class & Absolute_Velocity, const Velocity_Class & Relative_Velocity, const Coordinate_Class & Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	Absolute_Velocity.x_velocity = cos_Angle * (Relative_Velocity.x_velocity) - sin_Angle * (Relative_Velocity.y_velocity);
	Absolute_Velocity.y_velocity = sin_Angle * (Relative_Velocity.x_velocity) + cos_Angle * (Relative_Velocity.y_velocity);
	Absolute_Velocity.angle_velocity = (Relative_Velocity.angle_velocity);

	return Absolute_Velocity;
}

//************************************
// Method:    Relative_To_Absolute
// FullName:  Position_Class::Relative_To_Absolute
// Access:    public static
// Returns:   Position_Class::Coordinate_StructTypedef &
// Parameter: Coordinate_StructTypedef & Absolute_Coor
// Parameter: const Coordinate_StructTypedef & Relative_Coor
// Parameter: const Coordinate_StructTypedef & Base_Coor
// Description: 从相对坐标变化为绝对坐标
//************************************
Position_Class::Coordinate_Class &Position_Class::Relative_To_Absolute(Coordinate_Class &Absolute_Coor, const Coordinate_Class &Relative_Coor, const Coordinate_Class &Base_Coor)
{
	float cos_Angle = Cos_Lookup(Base_Coor.angle_coor);
	float sin_Angle = Sin_Lookup(Base_Coor.angle_coor);

	Absolute_Coor.x_coor = cos_Angle * (Relative_Coor.x_coor) - sin_Angle * (Relative_Coor.y_coor);
	Absolute_Coor.y_coor = sin_Angle * (Relative_Coor.x_coor) + cos_Angle * (Relative_Coor.y_coor);
	Absolute_Coor.angle_coor = Relative_Coor.angle_coor;
	Absolute_Coor += Base_Coor;

	return Absolute_Coor;
	// TODO: 在此处插入 return 语句
}

Position_Class & Position_Class::Relative_To_Absolute(Position_Class & Absolute_Position, const Position_Class & Relative_Position, const Coordinate_Class & Base_Coor)
{
	Absolute_Position.Coordinate = Relative_To_Absolute(Absolute_Position.Coordinate, Relative_Position.Coordinate, Base_Coor);
	Absolute_Position.Velocity = Relative_To_Absolute(Absolute_Position.Velocity, Relative_Position.Velocity, Base_Coor);
	return Absolute_Position;
	// TODO: 在此处插入 return 语句
}


//保留0.1的精度
//角度缩小至360°内
Position_Class::Coordinate_Class &Position_Class::Truncation_Coor(Coordinate_Class &Source_Coor)
{
	Source_Coor.x_coor = (long)(Source_Coor.x_coor * 10.0f) / 10.0f;
	Source_Coor.y_coor = (long)(Source_Coor.y_coor * 10.0f) / 10.0f;
	Source_Coor.angle_coor = ((long)(Source_Coor.angle_coor * 10.0f) % 3600) / 10.0f;

	return Source_Coor;
	// TODO: 在此处插入 return 语句
}

Position_Class::Velocity_Class operator+(const Position_Class::Velocity_Class &summand, const Position_Class::Velocity_Class &addend)
{
	Position_Class::Velocity_Class temp = summand;
	temp += addend;
	return temp;
}

Position_Class::Velocity_Class operator-(const Position_Class::Velocity_Class &minuend, const Position_Class::Velocity_Class &subtrahend)
{
	Position_Class::Velocity_Class temp = minuend;
	temp -= subtrahend;
	return temp;
}

Position_Class::Coordinate_Class operator+(const Position_Class::Coordinate_Class &summand, const Position_Class::Coordinate_Class &addend)
{
	Position_Class::Coordinate_Class temp = summand;
	temp += addend;
	return temp;
}

Position_Class::Coordinate_Class operator-(const Position_Class::Coordinate_Class &minuend, const Position_Class::Coordinate_Class &subtrahend)
{
	Position_Class::Coordinate_Class temp = minuend;
	temp -= subtrahend;
	return temp;
}

Position_Class operator+(const Position_Class &summand, const Position_Class &addend)
{
	Position_Class temp = summand;
	temp += addend;
	return temp;
}

Position_Class operator-(const Position_Class &minuend, const Position_Class &subtrahend)
{
	Position_Class temp = minuend;
	temp -= subtrahend;
	return temp;
}
