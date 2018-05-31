#pragma once
#include "Position.h"

//namespace Kalman_Filter
//{
//	//使用陀螺仪和编码器更新角度,返回更新后的角度
//	float Kalman_Cal_Theta(const float theta_encoder_delta, const float theta_previous, const float theta_gyo);
//
//	//使用陀螺仪和编码器更新角速度，返回更新后的角速度
//	float Kalman_Cal_Omega(const float omega_encoder, const float omega_gyro);
//
//	//使用加速度计和编码器更新位移、速度增量
//	void Kalman_Cal(const float acc_line_accelerometer, const float velocity_previous, float &velocity_delta, float&distance_delta);
//
//
//	//使用卡尔曼滤波更新角度、角速度
//	void Cal_Theta_Omega(const float omega_encoder, const float omega_gyro, float &theta_gyro, float &omega, float time_s, const float theta);
//	//编码器更新角度、角速度后，更新xy坐标
//	void Cal_XY_By_Gyro_Encoder(Coordinate_Class&Coor, const Velocity_Class &Velocity, const float theta_now, float time_s);
//
//	//根据编码器更新坐标
//	void Cal_X_Y_Theta_By_Encoder_Gyro(Coordinate_Class&Coor, const Velocity_Class &Velocity, float time_s);
//
//	Coordinate_Class Cal_Coor_By_PGV(const Coordinate_Class&Coor, const Coordinate_Class&Coor_PGV);
//}
