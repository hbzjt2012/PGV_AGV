#pragma once

//平面陀螺仪
//获取偏航角，偏航角速度
class Gyro_Class
{
public:
	Gyro_Class() = default;
	//Gyro_Class() : Uart_Base_Class(USART2), TX_DMA(DMA1_Stream6), data_OK(false), z_heading_bias(0.0f) {}
	virtual ~Gyro_Class() = default;

	virtual void Cal_Gyro_Data(void) = 0;	//计算陀螺仪数据
	void Set_Bias(float bias) { z_heading_bias = bias; }	//设置陀螺仪角度

	float z_rate;		 //Z轴角速率(°/s)
	float z_heading;	 //Z轴方位角，-180°~+180°

protected:
	float z_heading_bias;	//Z轴角度偏置(°)
};