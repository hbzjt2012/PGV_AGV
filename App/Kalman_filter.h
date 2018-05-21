#pragma once
#include "Position.h"

namespace Kalman_Filter
{
	//使用卡尔曼滤波更新角度、角速度
	void Cal_Theta_Omega(const float omega_encoder, const float omega_gyro, float &theta_gyro, float &omega, float time_s, const float theta);
	//编码器更新角度、角速度后，更新xy坐标
	void Cal_XY_By_Gyro_Encoder(Coordinate_Class&Coor, const Velocity_Class &Velocity, const float theta_now, float time_s);

	//根据编码器更新坐标
	void Cal_X_Y_Theta_By_Encoder_Gyro(Coordinate_Class&Coor, const Velocity_Class &Velocity, float time_s);

	Coordinate_Class Cal_Coor_By_PGV(const Coordinate_Class&Coor, const Coordinate_Class&Coor_PGV);
}
