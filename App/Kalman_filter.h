#pragma once
#include "Position.h"

namespace Kalman_Filter
{
	//使用卡尔曼滤波更新角度、角速度
	void Cal_Theta_Omega(float omega_encoder, float &theta_gyro, float &omega_gyro, float time_s, const float theta);

	//根据角度，角速度计算坐标
	void Cal_X_Y_Theta_By_Encoder_Gyro(Coordinate_Class&Coor, const Velocity_Class &Velocity, float time_s, bool theta_updated = false, float theta_now = 0.0f);
}