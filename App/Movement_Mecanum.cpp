#include "Movement_Mecanum.h"

//麦克纳姆轮的实现

Coordinate_Class & Movement_Mecanum_Class::Cal_Projection_Coor(const Coordinate_Class & Current_Coor_InOrigin)
{
	Coordinate_Class Projection_Coor;
	//注意角度正负(需注意)
	float X_H_mul_y = x_temp_InOrigin*Current_Coor_InOrigin.x_coor \
		+ y_temp_InOrigin*Current_Coor_InOrigin.y_coor \
		+ angle_equivalent_temp_InOrigin*(Current_Coor_InOrigin.angle_rad*Parameter_Class::wheel_lx_ly_distance);

	float k = X_H_mul_y / X_H_mul_X;

	//获取当前向量在终点向量上的投影向量，详情见《矩阵论》
	Projection_Coor = Destination_Coor_InOrigin*k;
}

//************************************
// Method:    Cal_Destination_Displacement
// FullName:  Movement_Mecanum_Class::Cal_Destination_Displacement
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Destination_Coor_InOrigin
// Description: 根据终点坐标在起点坐标中的坐标计算插补距离
//************************************
float Movement_Mecanum_Class::Cal_Destination_Displacement(const Coordinate_Class Destination_Coor_InOrigin)
{
	//储存各个轴在终点坐标系上的距离
	x_temp_InOrigin = Destination_Coor_InOrigin.x_coor;
	y_temp_InOrigin = Destination_Coor_InOrigin.y_coor;
	angle_equivalent_temp_InOrigin = Destination_Coor_InOrigin.angle_rad *Parameter_Class::wheel_lx_ly_distance;

	X_H_mul_X = x_temp_InOrigin*x_temp_InOrigin \
		+ y_temp_InOrigin*y_temp_InOrigin\
		+ angle_equivalent_temp_InOrigin*angle_equivalent_temp_InOrigin;

	distance_InOrigin_ABS = ABS(x_temp_InOrigin) + ABS(y_temp_InOrigin) + ABS(angle_equivalent_temp_InOrigin);

	return distance_InOrigin_ABS;
}

//************************************
// Method:    Assign_Velocity
// FullName:  Movement_Mecanum_Class::Assign_Velocity
// Access:    private 
// Returns:   Velocity_Class &
// Parameter: const Coordinate_Class & Destination_Coor_InOrigin
// Parameter: const float velocity
// Description: 根据终点在起点坐标系下的坐标，将速度分配个各个轴
//************************************
Velocity_Class & Movement_Mecanum_Class::Assign_Velocity(const Coordinate_Class & Destination_Coor_InOrigin, const float velocity)
{
	float k = 0.0f;

	k = x_temp_InOrigin / distance_InOrigin_ABS;
	Target_Velocity_InAGV.velocity_x = k*velocity;
	
	k = y_temp_InOrigin / distance_InOrigin_ABS;
	Target_Velocity_InAGV.velocity_y = k*velocity;
	
	k = angle_equivalent_temp_InOrigin / distance_InOrigin_ABS;
	Target_Velocity_InAGV.angular_velocity_mm = k*velocity;

	Target_Velocity_InAGV.angular_velocity_rad = Target_Velocity_InAGV.angular_velocity_mm / (Parameter_Class::wheel_lx_ly_distance);
	Target_Velocity_InAGV.angular_velocity_angle = Target_Velocity_InAGV.angular_velocity_rad / M_PI * 180;

	return Target_Velocity_InAGV;

}

//************************************
// Method:    Cal_Current_Coor_InOrigin
// FullName:  Movement_Mecanum_Class::Cal_Current_Coor_InOrigin
// Access:    private 
// Returns:   float
// Parameter: const Coordinate_Class Current_Coor_InOrigin
// Description: 根据当前坐标计算移动的距离
//************************************
float Movement_Mecanum_Class::Cal_Current_Coor_InOrigin(const Coordinate_Class Current_Coor_InOrigin)
{
	float angle_equivalent_temp, distance_temp;
	angle_equivalent_temp = Current_Coor_InOrigin.angle_rad*Parameter_Class::wheel_lx_ly_distance;

	distance_temp = ABS(Current_Coor_InOrigin.x_coor) + ABS(Current_Coor_InOrigin.y_coor) + ABS(angle_equivalent_temp);

	return distance_temp;
}
