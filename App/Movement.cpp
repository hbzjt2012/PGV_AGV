#include "Movement.h"

//因为同一时间只会执行一条运动指令，故为静态变量
Velocity_Class Movement_Class::Target_Velocity_InAGV;	//目标速度
Coordinate_Class Movement_Class::Target_Coor_InWorld;	//目标坐标

bool Movement_Class::Init(const Coordinate_Class & Origin_Coor, const Interpolation_Parameter_TypedefStructure & Input_Para)
{
	Origin_Coor_InWorld = Origin_Coor;	//保存起点坐标
	Destination_Coor_InOrigin = Destination_Coor_InWorld - Origin_Coor_InWorld;	//计算终点在起点坐标系上的相对坐标
	float input_distance_abs = Cal_Destination_Displacement(Destination_Coor_InOrigin);	//计算待插补的距离
	Update_Interpolation_Parameter(Input_Para);	//更新插补参数
	//此处可以修改插补阈值
	return Interpolation_Class::Init(input_distance_abs);	//返回插补结果
}

//************************************
// Method:    Cal_Velocity
// FullName:  Movement_Class::Cal_Velocity
// Access:    public 
// Returns:   bool 若插补完成，返回false
// Parameter: const Coordinate_Class Current_Coor_InWorld 当前坐标
// Description: 根据当前坐标，计算期望速度，期望坐标
//************************************
bool Movement_Class::Cal_Velocity(const Coordinate_Class Current_Coor_InWorld)
{
	Coordinate_Class Current_Coor_InOrigin = Current_Coor_InWorld - Origin_Coor_InWorld;	//获取当前坐标在起点坐标系中的坐标
	Coordinate_Class Target_Coor_InOrigin = Cal_Projection_Coor(Current_Coor_InOrigin);	//计算当前坐标向量在终点坐标向量上的投影（即目标坐标）

	float current_distance = Cal_Current_Coor_InOrigin(Target_Coor_InOrigin);	//获取期望坐标在起点坐标系上的位移
	bool Inter_result = Interpolation_Class::Cal_Velocity(current_distance);	//计算插补结果

	Velocity_Class Target_Velocity_InOrigin;	//期望坐标坐标系中的速度

	Target_Velocity_InOrigin = Assign_Velocity(Target_Coor_InOrigin, target_velocity);	//将速度分配给各个轴

	//将起点坐标系中的速度旋转至AGV坐标系
	Target_Velocity_InAGV = Velocity_Class::Absolute_To_Relative(Target_Velocity_InOrigin, Target_Velocity_InAGV, Target_Coor_InOrigin);

	//计算期望坐标在世界坐标系上的坐标
	Target_Coor_InWorld = Origin_Coor_InWorld + Target_Coor_InOrigin;

	return Inter_result;	//返回插补结果
}
