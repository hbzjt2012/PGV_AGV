#include "Kalman_filter.h"

//static float theta_variance = 0.0f;	//角度协方差
//
////计算执行器噪声，需实验测量,单位为°
//float Noise_Theta_Encoder(void) { return 2.0f; }
////计算陀螺仪测量噪声，需实验测量,单位为°
//float Noise_Theta_Gyro(void) { return 1.0f; }
//
//
////计算执行器噪声，需实验测量,单位为°/s
//float Noise_Omega_Encoder(void) { return 0.2f; }
////计算陀螺仪测量噪声，需实验测量,单位为°/s
//float Noise_Omega_Gyro(void) { return 0.05f; }
//
//static float omega_variance = 0.0f;	//初始角速度协方差
//
////************************************
//// Method:    Kalman_Cal_Theta
//// FullName:  Kalman_Filter::Kalman_Cal_Theta
//// Access:    public 
//// Returns:   float 使用卡尔曼滤波计算得出的新的角度(°)
//// Parameter: const float theta_encoder_delta 编码器计算得出的角度增量(°)
//// Parameter: const float theta_previous 上一时刻的角度值(°)
//// Parameter: const float theta_gyo 陀螺仪测量得到的角度(°)
//// Description: 编码器为执行机构，陀螺仪为测量机构，使用卡尔曼滤波计算新的角度
////************************************
//float Kalman_Filter::Kalman_Cal_Theta(const float theta_encoder_delta, const float theta_previous, const float theta_gyo)
//{
//	float theta_current_temp = theta_encoder_delta + theta_previous;	//当前角度预测值
//	float theta_variance_temp = theta_variance + Noise_Theta_Encoder();	//协方差预测值
//	float gain = theta_variance_temp / (theta_variance_temp + Noise_Theta_Gyro());	//计算增益系数
//
//	theta_current_temp = theta_current_temp + gain*(theta_gyo - theta_current_temp);	//更新角度估计值
//	theta_variance = (1 - gain)*theta_variance_temp;	//更新协方差预测值
//	return theta_current_temp;
//}
//
////************************************
//// Method:    Kalman_Cal_Omega
//// FullName:  Kalman_Filter::Kalman_Cal_Omega
//// Access:    public 
//// Returns:   float 新的角速度
//// Parameter: const float omega_encoder 使用编码器获取的角速度(°/s)
//// Parameter: const float omega_gyro 陀螺仪测量获取的角速度(°/s)
//// Description: 编码器为执行机构，陀螺仪为测量机构，使用卡尔曼滤波计算角速度
////************************************
//float Kalman_Filter::Kalman_Cal_Omega(const float omega_encoder, const float omega_gyro)
//{
//	float omega_temp = omega_encoder;
//	float omega_variance_temp = Noise_Omega_Encoder();
//	float gain = omega_variance_temp / (omega_variance_temp + Noise_Omega_Gyro());
//
//	omega_temp = omega_temp + gain*(omega_gyro - omega_temp);
//	omega_variance = (1 - gain)*omega_variance_temp;
//
//	return omega_temp;
//}
//
//void Kalman_Filter::Kalman_Cal(const float acc_line_accelerometer, const float velocity_previous, float & velocity_delta, float & distance_delta)
//{
//	static float acc_line_previous = 0.0f;	//上一时刻的加速度
//}
