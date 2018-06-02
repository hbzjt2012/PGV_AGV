#pragma once
#include "../DSP_Lib/arm_math.h"
//卡尔曼滤波算法
class Kalman_Class
{
public:
	Kalman_Class() = default;
	virtual ~Kalman_Class() = default;

	virtual void Init(void) = 0;
	virtual void Set_Noise(float noise) = 0;	//设置卡尔曼滤波器的执行噪声、测量噪声
	//根据控制量和测量量输出新的状态量
	virtual arm_matrix_instance_f32 &Kalman_Filter(const arm_matrix_instance_f32& input, const arm_matrix_instance_f32& measurement) = 0;
protected:

	//根据上一时刻的状态量和控制量预测这一时刻的状态量
	virtual void Forecast_State_Variable_Process_Model(const arm_matrix_instance_f32& input) = 0;
	virtual void Forecast_Covariance(void) = 0;	//预测协方差矩阵
	virtual void Cal_Kalman_Gain(void) = 0;	//计算卡尔曼增益矩阵
	virtual void Update_State_Variable(const arm_matrix_instance_f32& measurement) = 0;	//使用测量值更新状态量
	virtual void Update_Covariance(void) = 0;	//更新协方差

	float matrix_data_temp[100];	//矩阵处理数据暂存，大小为100，需事先给定一个足够大的空间
};