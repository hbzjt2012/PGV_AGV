#include "Kalman_filter.h"
#include "../macros.h"
#include "../DSP_Lib/arm_math.h"

float Cal_Encoder_Omega_Noise(float omega_encoder);

float variance_omega = 0.0f;	//角速度方差
float variance_x = 0.0f, variance_y = 0.0f, variance_theta = 0.0f;	//x,y,theta方差

float encoder_velocity_x_noise = 2.0f, encoder_velocity_y_noise = 2.0f, encoder_omega_noise;

float gyro_theta_measure_noise = 0.1f, gyro_omega_measure_noise = 0.05f;


//************************************
// Method:    Cal_Theta_Omega
// FullName:  Kalman_Filter::Cal_Theta_Omega
// Access:    public 
// Returns:   void
// Parameter: float omega_encoder 角速度控制量(rad/s)
// Parameter: float & theta_gyro 输入为陀螺仪测量值，输出为新的估计值(°)
// Parameter: float & omega_gyro 输入为陀螺仪测量值，输出为新的估计值(rad/s)
// Parameter: float time_s 间隔时间(s)
// Parameter: const float theta 上一时刻的角度(°)
// Description: 使用卡尔曼滤波融合编码器和陀螺仪的数据，计算出角度和角速度
//************************************
void Kalman_Filter::Cal_Theta_Omega(float omega_encoder, float & theta_gyro, float & omega_gyro, float time_s, const float theta)
{
	float omega_encoder_temp = omega_encoder * 180 / M_PI, omega_gyro_temp = omega_gyro * 180 / M_PI;
	float theta_temp, omega_temp;

	if (theta_gyro > 360.0f)
	{
		theta_gyro -= 360.0f;
	}
	else if (theta_gyro < 0.0f)
	{
		theta_gyro += 360.0f;
	}

	theta_temp = theta + omega_encoder_temp * time_s;	//计算预测角度(°)
	omega_temp = omega_encoder_temp;	//计算预测角速度(°/s)
	encoder_omega_noise = Cal_Encoder_Omega_Noise(omega_encoder_temp);	//计算运动噪声

	//预测协方差
	variance_omega += encoder_omega_noise;
	variance_theta += encoder_omega_noise;

	//计算增益矩阵
	float K_theta = variance_theta / variance_theta + gyro_theta_measure_noise;
	float K_omega = variance_omega / variance_omega + gyro_omega_measure_noise;

	//float32_t variance_numerator[4] = { variance_theta ,0.0f,0.0f,variance_omega };
	//float32_t variance_denominator[4] = { variance_theta + gyro_theta_measure_noise ,0.0f,0.0f,variance_omega + gyro_omega_measure_noise };
	//float32_t variance_denominator_inv[4];

	//arm_matrix_instance_f32 numerator, denominator, denominator_inv;
	//numerator.numCols = 2;
	//numerator.numRows = 2;
	//numerator.pData = variance_numerator;

	//denominator.numCols = 2;
	//denominator.numRows = 2;
	//denominator.pData = variance_denominator;

	//denominator_inv.numCols = 2;
	//denominator_inv.numRows = 2;
	//denominator_inv.pData = variance_denominator_inv;

	//arm_mat_inverse_f32(&denominator, &denominator_inv);	//求矩阵逆

	theta_gyro = theta_temp + K_theta*(theta_gyro - theta_temp);	//更新角度估计值(°)
	omega_gyro = omega_temp + K_omega*(omega_gyro_temp - omega_temp);	//更新角速度估计值(°/s)
	omega_gyro = omega_gyro / 180 * M_PI;	//转化为(rad/s)

	//更新方差
	variance_omega = (1 - K_omega)*variance_omega;
	variance_theta = (1 - K_theta)*variance_theta;
}

void Kalman_Filter::Cal_X_Y_Theta_By_Encoder_Gyro(Coordinate_Class&Coor, const Velocity_Class &Velocity, float time_s, bool theta_updated, float theta_now)
{
	float delta_theta = 0.0f;	//角度更新
	theta_now = Coordinate_Class::Transform_Angle(theta_now);	//变换角度
	if (!theta_updated)	//当前角度还未更新
	{
		variance_theta += Cal_Encoder_Omega_Noise(Velocity.angular_velocity * 180 / M_PI);	//更新协方差

		delta_theta = Velocity.angular_velocity*time_s * 180 / M_PI;	//一个控制周期内角度的增加量(°)

		theta_now = Coor.angle_coor + delta_theta;	//计算新角度
		theta_now = Coordinate_Class::Transform_Angle(theta_now);	//变换角度
	}

	delta_theta = theta_now - Coor.angle_coor;	//求出角度的增加值
	delta_theta = Coordinate_Class::Transform_Angle(delta_theta);	//变换角度

	float sin_last = Sin_Lookup(Coor.angle_coor + Velocity.velocity_angle);
	float sin_current = Sin_Lookup(theta_now + Velocity.velocity_angle);
	float cos_last = Cos_Lookup(Coor.angle_coor + Velocity.velocity_angle);
	float cos_current = Cos_Lookup(theta_now + Velocity.velocity_angle);

	if (ABS(delta_theta) < 0.1f)	//直线运动 角度增加值小于0.1°，认为是直线运动
	{
		Coor.x_coor += Velocity.velocity*cos_last*time_s;
		Coor.y_coor += Velocity.velocity*sin_last*time_s;
		Coor.angle_coor = theta_now;
	}
	else
	{
		float radius = Velocity.velocity / Velocity.angular_velocity;	//半径
		Coor.x_coor = Coor.x_coor - radius*(sin_current - sin_last);
		Coor.y_coor = Coor.y_coor + radius*(cos_last - cos_current);
		Coor.angle_coor = theta_now;
	}

	//更新方差
	variance_x += encoder_velocity_x_noise;
	variance_y += encoder_velocity_y_noise;
}

//根据角速度计算执行噪声
//输入为编码器测量的角速度(°/s)
//需由实验测得，待修改
float Cal_Encoder_Omega_Noise(float omega_encoder)
{
	if (ABS(omega_encoder) < 5.0f)
	{
		return 1.0f;
	}
	else
	{
		return 2.0f;
	}
}
