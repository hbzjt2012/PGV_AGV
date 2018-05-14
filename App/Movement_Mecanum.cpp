#include "Movement_Mecanum.h"

//麦克纳姆轮的实现



//************************************
// Method:    Cal_Displacement
// FullName:  Movemeng_Mecanum_Class::Cal_Displacement
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Destination_Coor_InOrigin
// Description: 根据终点坐标在起点坐标中的坐标计算插补距离
//************************************
float Movemeng_Mecanum_Class::Cal_Displacement(const Coordinate_Class Destination_Coor_InOrigin)
{
	x_temp_InOrigin = Destination_Coor_InOrigin.x_coor;
	y_temp_InOrigin = Destination_Coor_InOrigin.y_coor;
	angle_equivalent_temp_InOrigin = Destination_Coor_InOrigin.angle_coor / 180.0f*M_PI*(DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2;

	distance_InOrigin = ABS(x_temp_InOrigin) + ABS(y_temp_InOrigin) + ABS(angle_equivalent_temp_InOrigin);

	return distance_InOrigin;
}

//************************************
// Method:    Cal_Velocity
// FullName:  Movemeng_Mecanum_Class::Cal_Velocity
// Access:    private 
// Returns:   Velocity_Class&
// Parameter: const Coordinate_Class & Destination_Coor_InOrigin
// Parameter: const float velocity
// Description: 根据终点坐标在起点坐标中的坐标，将和速度分配给各个轴
//************************************
Velocity_Class &Movemeng_Mecanum_Class::Cal_Velocity(const Coordinate_Class & Destination_Coor_InOrigin, const float velocity)
{
	float k = x_temp_InOrigin / distance_InOrigin;
	float x_velocity = k*velocity;
	k = y_temp_InOrigin / distance_InOrigin;
	float y_velocity = k*velocity;
	k = angle_equivalent_temp_InOrigin / distance_InOrigin;
	float angle_equivalent_velocity = k*velocity;

	Target_Velocity_InAGV.velocity_angle = ArcTan_Lookup(x_velocity, y_velocity);
	x_velocity = ABS(x_velocity);
	y_velocity = ABS(y_velocity);

	Target_Velocity_InAGV.velocity = sqrtf(x_velocity*x_velocity + y_velocity*y_velocity);

	Target_Velocity_InAGV.angular_velocity = angle_equivalent_velocity / M_PI*180.0f / ((DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2);

	return Target_Velocity_InAGV;
}

//************************************
// Method:    Cal_Current_Coor_InOrigin
// FullName:  Movemeng_Mecanum_Class::Cal_Current_Coor_InOrigin
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Current_Coor_InOrigin
// Description: 根据当前坐标计算移动的距离
//************************************
float Movemeng_Mecanum_Class::Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin)
{
	float x_temp, y_temp, angle_equivalent_temp, distance_temp;
	x_temp = Current_Coor_InOrigin.x_coor;
	y_temp = Current_Coor_InOrigin.y_coor;
	angle_equivalent_temp = Current_Coor_InOrigin.angle_coor / 180.0f*M_PI*(DISTANCE_OF_WHEEL_X_AXES + DISTANCE_OF_WHEEL_Y_AXES) / 2;


	x_temp = x_temp_InOrigin > 0.0f ? x_temp : -x_temp;
	y_temp = y_temp_InOrigin > 0.0f ? y_temp : -y_temp;
	angle_equivalent_temp = angle_equivalent_temp_InOrigin > 0.0f ? angle_equivalent_temp : -angle_equivalent_temp;


	distance_temp = x_temp + y_temp + angle_equivalent_temp;

	return distance_temp;
}
