#include "Kalman_filter.h"
#include "../macros.h"
#include "../DSP_Lib/arm_math.h"

//以下函数均需要通过实验得出规律，此处为了方便调试，先假设
//角度单位一律为°
//角速度单位一律为°/s
float Return_Encoder_Omega_Noise(float omega_encoder);	//返回角速度运动噪声
float Return_Encoder_Velocity_Noise(float velocity);	//返回线速度运动噪声
float Return_Encoder_Velocity_Theta_Noise(float velocity_theta);	//返回线速度夹角运动噪声
float Return_Gyro_Theta_Measure_Noise(float theta_encoder);	//返回陀螺仪测量角度噪声
float Return_Gyro_Omega_Measure_Noise(float omega_encoder);	//返回陀螺仪测量角速度噪声


float variance_coor[9] = { 100.0f,0.0f,0.0f,\
							0.0f,100.0f,0.0f,\
							0.0f,0.0f,20.0f };	//坐标协方差，含角度
float variance_coor_xy[4];	//坐标协方差，不含角度
arm_matrix_instance_f32 variance_coor_matrix = { 3,3,variance_coor };	//坐标协方差矩阵，含角度
arm_matrix_instance_f32 variance_coor_xy_matrix = { 2,2,variance_coor_xy };	//坐标协方差矩阵，不含角度


float variance_coor_PGV[9] = { 0.01f,0.0f,0.0f,\
0.0f, 0.01f,0.0f,\
0.0f,0.0f, 0.01f };	//PGV传感器的测量噪声
arm_matrix_instance_f32 variance_coor_PGV_matrix = { 3,3,variance_coor_PGV };	//坐标协方差矩阵，含角度

float variance_coor_temp1[9];	//坐标协方差暂存区，含角度
float variance_coor_xy_temp1[4];	//坐标协方差暂存区，不含角度
arm_matrix_instance_f32 variance_coor_matrix_temp1 = { 3,3,variance_coor_temp1 };	//坐标协方差矩阵暂存，含角度
arm_matrix_instance_f32 variance_coor_xy_matrix_temp1 = { 2,2,variance_coor_xy_temp1 };	//坐标协方差矩阵暂存，不含角度

float variance_coor_temp2[9];	//坐标协方差暂存区，含角度
float variance_coor_xy_temp2[4];	//坐标协方差暂存区，不含角度
arm_matrix_instance_f32 variance_coor_matrix_temp2 = { 3,3,variance_coor_temp2 };	//坐标协方差矩阵暂存，含角度
arm_matrix_instance_f32 variance_coor_xy_matrix_temp2 = { 2,2,variance_coor_xy_temp2 };	//坐标协方差矩阵暂存，不含角度

float variance_coor_temp3[9];	//坐标协方差暂存区，含角度
float variance_coor_xy_temp3[4];	//坐标协方差暂存区，不含角度
arm_matrix_instance_f32 variance_coor_matrix_temp3 = { 3,3,variance_coor_temp3 };	//坐标协方差矩阵暂存，含角度
arm_matrix_instance_f32 variance_coor_xy_matrix_temp3 = { 2,2,variance_coor_xy_temp3 };	//坐标协方差矩阵暂存，不含角度

float variance_velocity_temp1[9];	//坐标协方差暂存区，含角度
arm_matrix_instance_f32 variance_velocity_matrix_temp1 = { 3,3,variance_velocity_temp1 };	//坐标协方差矩阵暂存，含角度

float variance_velocity_temp2[9];	//坐标协方差暂存区，含角度
arm_matrix_instance_f32 variance_velocity_matrix_temp2 = { 3,3,variance_velocity_temp2 };	//坐标协方差矩阵暂存，含角度

float variance_velocity_temp3[9];	//坐标协方差暂存区，含角度
arm_matrix_instance_f32 variance_velocity_matrix_temp3 = { 3,3,variance_velocity_temp3 };	//坐标协方差矩阵暂存，含角度

float &variance_theta = variance_coor[8];	//角度方差
float &variance_x = variance_coor[4];	//x坐标方差
float &variance_y = variance_coor[3];	//y坐标方差
float variance_omega = 0.0f;	//角速度方差


//************************************
// Method:    Cal_Theta_Omega
// FullName:  Kalman_Filter::Cal_Theta_Omega
// Access:    public 
// Returns:   void
// Parameter: const float omega_encoder 编码器获取的角速度控制量(°/s)
// Parameter: const float omega_gyro 陀螺仪角速度测量量(°/s)
// Parameter: float & theta_gyro 输入为陀螺仪角度测量量(°),输出为新的角度(°)
// Parameter: float & omega 输出为新的角速度(°/s)
// Parameter: float time_s 间隔时间(s)
// Parameter: const float theta 上一时刻的角度(°)
// Description: 使用卡尔曼滤波融合编码器和陀螺仪的数据，计算出角度和角速度
//************************************
void Kalman_Filter::Cal_Theta_Omega(const float omega_encoder, const float omega_gyro, float & theta_gyro, float &omega, float time_s, const float theta)
{
	//float omega_gyro_temp = omega_gyro;	//陀螺仪测得的车体角速度,单位为°/s
	float theta_gyro_temp = theta_gyro;	//陀螺仪测得的车体偏转角,单位为°，范围为-180°~+180°

	//float theta_temp, omega_temp;

	theta_gyro = theta + omega_encoder * time_s;	//计算预测角度(°)
	omega = omega_encoder;	//计算预测角速度(°/s)

	float omega_encoder_noise = Return_Encoder_Omega_Noise(omega_encoder);	//运动(角速度)噪声

	variance_omega = omega_encoder_noise;	//预测角速度方差
	variance_theta += omega_encoder_noise*time_s;	//预测角度方差

	//计算测量噪声
	float gyro_omega_measure_noise = Return_Gyro_Omega_Measure_Noise(omega_gyro);
	float gyro_theta_measure_noise = Return_Gyro_Theta_Measure_Noise(theta_gyro_temp);
	//计算增益矩阵
	float K_theta = variance_theta / (variance_theta + gyro_theta_measure_noise);
	float K_omega = variance_omega / (variance_omega + gyro_omega_measure_noise);

	//arm_mat_inverse_f32(&denominator, &denominator_inv);	//求矩阵逆

	theta_gyro = theta_gyro + K_theta*(theta_gyro_temp - theta_gyro);	//更新角度估计值(°)
	omega = omega + K_omega*(omega_gyro - omega);	//更新角速度估计值(°/s)
	//omega_gyro = omega_gyro / 180 * M_PI;	//转化为(rad/s)
	theta_gyro = Coordinate_Class::Transform_Angle(theta_gyro);

	variance_omega = (1 - K_omega)*variance_omega;	//更新角速度方差
	variance_theta = (1 - K_theta)*variance_theta;	//更新角度方差
}

//************************************
// Method:    Cal_XY_By_Gyro_Encoder
// FullName:  Kalman_Filter::Cal_XY_By_Gyro_Encoder
// Access:    public 
// Returns:   void
// Parameter: Coordinate_Class & Coor 输入为上一时刻坐标，输出为当前时刻的坐标
// Parameter: const Velocity_Class & Velocity 速度输入
// Parameter: const float theta_now 这一时刻的角度(°)
// Parameter: float time_s 时间间隔(s)
// Description: 更新已由陀螺仪更新过角度、角速度的xy坐标
//************************************
void Kalman_Filter::Cal_XY_By_Gyro_Encoder(Coordinate_Class & Coor, const Velocity_Class & Velocity, const float theta_now, float time_s)
{
	static float encoder_noise[4] = { 0.0f,0.0f,0.0f,0.0f };
	static arm_matrix_instance_f32 encoder_noise_matrix = { 2,2,encoder_noise };	//运动噪声

	float omega_temp = Velocity.angular_velocity * 180 / M_PI;	//将角速度转化为°/s

	float velocity_theta_last = Coor.angle_coor + Velocity.velocity_angle;	//上一时刻速度夹角
	float velocity_theta_current = theta_now + Velocity.velocity_angle;	//这一时刻速度夹角
	float delta_theta = theta_now - Coor.angle_coor;	//偏转角差值，用于判断当前是直线运动还是圆弧运动

	float sin_last = Sin_Lookup(velocity_theta_last);
	float sin_current = Sin_Lookup(velocity_theta_current);
	float cos_last = Cos_Lookup(velocity_theta_last);
	float cos_current = Cos_Lookup(velocity_theta_current);

	//计算运动噪声
	encoder_noise[0] = Return_Encoder_Velocity_Noise(Velocity.velocity);
	encoder_noise[3] = Return_Encoder_Velocity_Theta_Noise(Velocity.velocity_angle);

	//保存协方差
	variance_coor_xy[0] = variance_coor[0];
	variance_coor_xy[1] = variance_coor[1];
	variance_coor_xy[2] = variance_coor[3];
	variance_coor_xy[3] = variance_coor[4];

	if (ABS(delta_theta) < 0.1f || (ABS(omega_temp) < (0.1 / time_s)))	//直线运动 角度增加值小于0.1°，认为是直线运动
	{
		//更新坐标
		Coor.x_coor += Velocity.velocity*cos_last*time_s;
		Coor.y_coor += Velocity.velocity*sin_last*time_s;
		//计算运动噪声与状态空间的映射关系V
		variance_coor_xy_temp1[0] = cos_last*time_s;
		variance_coor_xy_temp1[1] = -Velocity.velocity*sin_last*time_s;
		variance_coor_xy_temp1[2] = sin_last*time_s;
		variance_coor_xy_temp1[3] = Velocity.velocity*cos_last*time_s;
	}
	else
	{
		float radius = Velocity.velocity / Velocity.angular_velocity;	//半径
		Coor.x_coor += Coor.x_coor - radius*(sin_last - sin_current);
		Coor.y_coor += Coor.y_coor + radius*(cos_last - cos_current);
		//计算运动噪声与状态空间的映射关系V
		variance_coor_xy_temp1[0] = -(sin_last - sin_current) / Velocity.angular_velocity;
		variance_coor_xy_temp1[1] = -radius*(cos_last - cos_current);
		variance_coor_xy_temp1[2] = (cos_last - cos_current) / Velocity.angular_velocity;
		variance_coor_xy_temp1[3] = -radius*(sin_last - sin_current);
	}
	Coor.angle_coor = theta_now;

	//更新协方差
	arm_mat_trans_f32(&variance_coor_xy_matrix_temp1, &variance_coor_xy_matrix_temp2);	//求转置

	//计算V*M*V^T,结果存放在variance_coor_xy_matrix_temp1中
	arm_mat_mult_f32(&variance_coor_xy_matrix_temp1, &encoder_noise_matrix, &variance_coor_xy_matrix_temp3);
	arm_mat_mult_f32(&variance_coor_xy_matrix_temp3, &variance_coor_xy_matrix_temp2, &variance_coor_xy_matrix_temp1);

	//将新的协方差保存在variance_coor_xy_matrix_temp2中
	arm_mat_add_f32(&variance_coor_xy_matrix, &variance_coor_xy_matrix_temp1, &variance_coor_xy_matrix_temp2);

	//保存新的协方差
	variance_coor_xy[0] = variance_coor[0] = variance_coor_xy_temp2[0];
	variance_coor_xy[1] = variance_coor[1] = variance_coor_xy_temp2[1];
	variance_coor_xy[2] = variance_coor[3] = variance_coor_xy_temp2[2];
	variance_coor_xy[3] = variance_coor[4] = variance_coor_xy_temp2[3];
}

void Kalman_Filter::Cal_X_Y_Theta_By_Encoder_Gyro(Coordinate_Class & Coor, const Velocity_Class & Velocity, float time_s)
{
	static float encoder_noise[9] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
	static arm_matrix_instance_f32 encoder_noise_matrix = { 3,3,encoder_noise };	//运动噪声

	float omega_temp = Velocity.angular_velocity * 180 / M_PI;	//将角速度转化为°/s

	float velocity_theta_last = Coor.angle_coor + Velocity.velocity_angle;	//上一时刻速度夹角
	float delta_theta = omega_temp*time_s;	//偏转角差值，用于判断当前是直线运动还是圆弧运动
	float velocity_theta_current = velocity_theta_last + delta_theta;	//这一时刻速度夹角

	float sin_last = Sin_Lookup(velocity_theta_last);
	float sin_current = Sin_Lookup(velocity_theta_current);
	float cos_last = Cos_Lookup(velocity_theta_last);
	float cos_current = Cos_Lookup(velocity_theta_current);

	//计算运动噪声
	encoder_noise[0] = Return_Encoder_Velocity_Noise(Velocity.velocity);
	encoder_noise[4] = Return_Encoder_Velocity_Theta_Noise(Velocity.velocity_angle);
	encoder_noise[8] = Return_Encoder_Omega_Noise(omega_temp);



	if (ABS(delta_theta) < 0.1f)	//直线运动 角度增加值小于0.1°，认为是直线运动
	{
		//更新坐标
		Coor.x_coor += Velocity.velocity*cos_last*time_s;
		Coor.y_coor += Velocity.velocity*sin_last*time_s;

		//计算更新协方差的雅克比矩阵G
		variance_coor_temp1[0] = 1.0f;
		variance_coor_temp1[1] = 0.0f;
		variance_coor_temp1[2] = -Velocity.velocity*sin_last*time_s;
		variance_coor_temp1[3] = 0.0f;
		variance_coor_temp1[4] = 1.0f;
		variance_coor_temp1[5] = Velocity.velocity*cos_last*time_s;
		variance_coor_temp1[6] = 0.0f;
		variance_coor_temp1[7] = 0.0f;
		variance_coor_temp1[8] = 1.0f;


		//计算运动噪声与状态空间的映射关系V
		variance_velocity_temp1[0] = cos_last*time_s;
		variance_velocity_temp1[1] = -Velocity.velocity*sin_last*time_s;
		variance_velocity_temp1[3] = 0.0f;
		variance_velocity_temp1[4] = sin_last*time_s;
		variance_velocity_temp1[5] = Velocity.velocity*cos_last*time_s;
		variance_velocity_temp1[5] = 0.0f;
		variance_velocity_temp1[6] = 0.0f;
		variance_velocity_temp1[7] = 0.0f;
		variance_velocity_temp1[8] = 0.0f;
	}
	else
	{
		//更新坐标
		float radius = Velocity.velocity / Velocity.angular_velocity;	//半径
		Coor.x_coor += Coor.x_coor - radius*(sin_last - sin_current);
		Coor.y_coor += Coor.y_coor + radius*(cos_last - cos_current);
		Coor.angle_coor += delta_theta;


		//计算更新协方差的雅克比矩阵G
		variance_coor_temp1[0] = 1.0f;
		variance_coor_temp1[1] = 0.0f;
		variance_coor_temp1[2] = -radius*(cos_last - cos_current);
		variance_coor_temp1[3] = 0.0f;
		variance_coor_temp1[4] = 1.0f;
		variance_coor_temp1[5] = -radius*(sin_last - sin_current);
		variance_coor_temp1[6] = 0.0f;
		variance_coor_temp1[7] = 0.0f;
		variance_coor_temp1[8] = 1.0f;


		//计算运动噪声与状态空间的映射关系V
		variance_velocity_temp1[0] = -(sin_last - sin_current) / Velocity.angular_velocity;
		variance_velocity_temp1[1] = -radius*(cos_last - cos_current);
		variance_velocity_temp1[3] = radius / Velocity.angular_velocity*(sin_last - sin_current) + radius*cos_current*time_s;
		variance_velocity_temp1[4] = (cos_last - cos_current) / Velocity.angular_velocity;
		variance_velocity_temp1[5] = -radius*(sin_last - sin_current);
		variance_velocity_temp1[5] = -radius / Velocity.angular_velocity*(cos_last - cos_current) + radius*sin_current*time_s;
		variance_velocity_temp1[6] = 0.0f;
		variance_velocity_temp1[7] = 0.0f;
		variance_velocity_temp1[8] = time_s;
	}

	//更新协方差
	arm_mat_trans_f32(&variance_coor_matrix_temp1, &variance_coor_matrix_temp2);	//求转置

	arm_mat_mult_f32(&variance_coor_matrix_temp1, &variance_coor_matrix, &variance_coor_matrix_temp3);
	arm_mat_mult_f32(&variance_coor_matrix_temp3, &variance_coor_matrix_temp2, &variance_coor_matrix_temp1);//计算G*协方差*G^T,结果存放在variance_coor_matrix_temp1中

	arm_mat_trans_f32(&variance_velocity_matrix_temp1, &variance_velocity_matrix_temp2);	//求转置

	arm_mat_mult_f32(&variance_velocity_matrix_temp1, &encoder_noise_matrix, &variance_velocity_matrix_temp3);
	arm_mat_mult_f32(&variance_velocity_matrix_temp3, &variance_velocity_matrix_temp2, &variance_velocity_matrix_temp1);//计算G*协方差*G^T,结果存放在variance_coor_matrix_temp1中

	arm_mat_add_f32(&variance_velocity_matrix_temp1, &variance_coor_matrix_temp1, &variance_coor_matrix);

}

Coordinate_Class Kalman_Filter::Cal_Coor_By_PGV(const Coordinate_Class & Coor, const Coordinate_Class & Coor_PGV)
{
	static float coor_temp1[3] = { 0.0f,0.0f,0.0f }, coor_temp2[3] = { 0.0f,0.0f,0.0f };
	static arm_matrix_instance_f32 coor_temp1_matrix = { 3,1,coor_temp1 }, coor_temp2_matrix = { 3,1,coor_temp2 };	//运动噪声

	Coordinate_Class Coor_temp;

	arm_mat_add_f32(&variance_coor_matrix, &variance_coor_PGV_matrix, &variance_coor_matrix_temp1);
	arm_mat_inverse_f32(&variance_coor_matrix_temp1, &variance_velocity_matrix_temp2);	//求逆

	arm_mat_mult_f32(&variance_coor_matrix, &variance_velocity_matrix_temp2, &variance_velocity_matrix_temp1);//将增益矩阵存放在variance_velocity_matrix_temp1中

	arm_mat_sub_f32(&Coor_PGV.coor_matrix, &Coor.coor_matrix, &coor_temp1_matrix);	//将测量值和估计值的差保存在coor_temp1_matrix中
	arm_mat_mult_f32(&variance_velocity_matrix_temp1, &coor_temp1_matrix, &coor_temp2_matrix);	//乘以增益系数

	arm_mat_add_f32(&Coor.coor_matrix, &coor_temp2_matrix, &Coor_temp.coor_matrix);	//得到新的估计值

	for (int i = 0; i < 9; i++)
	{
		variance_velocity_temp1[i] = -variance_velocity_temp1[i];
	}
	variance_velocity_temp1[0] += 1.0f;
	variance_velocity_temp1[4] += 1.0f;
	variance_velocity_temp1[8] += 1.0f;

	arm_mat_mult_f32(&variance_velocity_matrix_temp1, &variance_coor_matrix, &variance_velocity_matrix_temp2);//将更新后的协方差矩阵存放在variance_velocity_matrix_temp2中

	//更新协方差矩阵
	for (int i = 0; i < 9; i++)
	{
		variance_coor[i] = variance_velocity_temp2[i];
	}

	return Coor_temp;
}


//根据角速度计算执行噪声
//输入为编码器测得的车体角速度(°/s)
//返回为执行噪声，具体返回值需由实验测得
float Return_Encoder_Omega_Noise(float omega_encoder)
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

float Return_Encoder_Velocity_Noise(float velocity)
{
	return 1.0f;
}

float Return_Encoder_Velocity_Theta_Noise(float velocity_theta)
{
	return 1.0f;
}

float Return_Gyro_Theta_Measure_Noise(float theta_encoder)
{
	return 0.1f;
}

float Return_Gyro_Omega_Measure_Noise(float omega_encoder)
{
	return 0.05f;
}
