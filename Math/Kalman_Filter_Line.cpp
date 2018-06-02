#include "Kalman_Filter_Line.h"

void Kalman_Line_Class::Init(void)
{
	arm_mat_init_f32(&process_matrix, 2, 1, process_data);
	arm_mat_init_f32(&measurement_matrix, 2, 1, measurement_data);

	arm_mat_init_f32(&state_variable_matrix, 2, 1, state_variable_data);
	arm_mat_init_f32(&vcovariance_matrix, 2, 2, vcovariance_data);

	arm_mat_init_f32(&A_matrix, 2, 2, A_data);
	arm_mat_init_f32(&B_matrix, 2, 2, B_data);

	arm_mat_init_f32(&process_noise_covariance_matrix, 2, 2, process_noise_covariance_matrix_data);
	arm_mat_init_f32(&measurement_noise_covariance_matrix, 2, 2, measurement_noise_covariance_matrix_data);

	arm_mat_init_f32(&gain_matrix, 2, 2, gain_matrix_data);

	A_data[0] = 0.0f;
	A_data[2] = 0.0f;
	A_data[3] = 1.0f;


	//初始置信度
	for (int i = 0; i < 4; i++)
	{
		vcovariance_data[i] = 0.0f;
	}

	state_variable_data[0] = state_variable_data[1] = 0.0f;

}


//************************************
// Method:    Set_Noise
// FullName:  Kalman_Line_Class::Set_Noise
// Access:    private 
// Returns:   void
// Parameter: float time_s 时间间隔，涉及到A、B
// Parameter: float noise_encoder_theta 打滑程度的度量
// Description: 更新滤波器参数
//************************************
void Kalman_Line_Class::Set_Noise(float time_s, float noise_encoder_theta)
{
	measurement_noise_covariance_matrix_data[3] = noise_encoder_theta*noise_encoder_theta;
	Set_Noise(time_s);
}

arm_matrix_instance_f32 & Kalman_Line_Class::Kalman_Filter(const arm_matrix_instance_f32 & input, const arm_matrix_instance_f32 & measurement)
{
	Forecast_State_Variable_Process_Model(input);	//预测状态量
	Forecast_Covariance();	//预测协方差矩阵
	Cal_Kalman_Gain();	//计算卡尔曼增益矩阵
	Update_State_Variable(measurement);	//更新状态量
	Update_Covariance();	//更新协方差矩阵

	return state_variable_matrix;
}

//************************************
// Method:    Update_Stae_Variable_No_Process
// FullName:  Kalman_Line_Class::Update_Stae_Variable_No_Process
// Access:    public 
// Returns:   arm_matrix_instance_f32 &
// Parameter: const arm_matrix_instance_f32 & measurement
// Parameter: const float time_s
// Description: 只使用测量量直接更新状态量
//************************************
arm_matrix_instance_f32 & Kalman_Line_Class::Update_Stae_Variable_No_Process(const arm_matrix_instance_f32 & measurement, const float time_s)
{
	Set_Noise(time_s, noise_measurement_velocity);	//设置噪声

	arm_matrix_instance_f32 matrix_temp1;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放vcovariance+measurement_noise_covariance
	arm_mat_init_f32(&matrix_temp1, vcovariance_matrix.numRows, measurement_noise_covariance_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_add_f32(&vcovariance_matrix, &measurement_noise_covariance_matrix, &matrix_temp1);

	//保存新的协方差
	int data_size = matrix_temp1.numRows*matrix_temp1.numCols;
	float *source_data = vcovariance_matrix.pData;
	float *pdata = matrix_temp1.pData;

	for (int i = 0; i < data_size; i++)
	{
		*source_data = *pdata;
		source_data++;
		pdata++;
	}

	data_size = measurement.numRows*measurement.numCols;
	source_data = state_variable_matrix.pData;
	pdata = measurement.pData;

	for (int i = 0; i < data_size; i++)
	{
		*source_data = *pdata;
		source_data++;
		pdata++;
	}

	return state_variable_matrix;
}


//执行噪声和测量噪声都与time有关
void Kalman_Line_Class::Set_Noise(float noise)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2, matrix_temp3;
	float *matrix_data_temp = this->matrix_data_temp;

	A_data[1] = noise;
	B_data[0] = B_data[1] = noise*noise / 4;
	B_data[2] = B_data[3] = noise / 2;

	//初始化matrix_temp1,用于存放B^T
	arm_mat_init_f32(&matrix_temp1, B_matrix.numCols, B_matrix.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_trans_f32(&B_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放控制量协方差
	arm_mat_init_f32(&matrix_temp2, B_matrix.numCols, B_matrix.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	*matrix_temp2.pData = noise_process_acc;
	*(matrix_temp2.pData + 1) = 0.0f;
	*(matrix_temp2.pData + 2) = 0.0f;
	*(matrix_temp2.pData + 3) = noise_process_acc;

	//初始化matrix_temp3,用于存放B*noise
	arm_mat_init_f32(&matrix_temp3, B_matrix.numRows, matrix_temp2.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp3.numRows*matrix_temp3.numCols;
	arm_mat_mult_f32(&B_matrix, &matrix_temp2, &matrix_temp3);

	//更新执行噪声
	arm_mat_mult_f32(&matrix_temp3, &matrix_temp1, &process_noise_covariance_matrix);

	//更新测量噪声
	measurement_noise_covariance_matrix_data[0] = measurement_noise_covariance_matrix_data[3] * noise*noise;
	measurement_noise_covariance_matrix_data[1] = measurement_noise_covariance_matrix_data[3] * noise;
	measurement_noise_covariance_matrix_data[2] = measurement_noise_covariance_matrix_data[3] * noise;
}

void Kalman_Line_Class::Forecast_State_Variable_Process_Model(const arm_matrix_instance_f32 & input)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放A^state
	arm_mat_init_f32(&matrix_temp1, A_matrix.numRows, state_variable_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_mult_f32(&A_matrix, &state_variable_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放B*input
	arm_mat_init_f32(&matrix_temp2, B_matrix.numRows, input.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_mult_f32(&B_matrix, &input, &matrix_temp2);

	//预测状态量
	arm_mat_add_f32(&matrix_temp1, &matrix_temp2, &state_variable_matrix);

}

void Kalman_Line_Class::Forecast_Covariance(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2, matrix_temp3;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放A^T
	arm_mat_init_f32(&matrix_temp1, A_matrix.numCols, A_matrix.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_trans_f32(&A_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放A*vcovariance
	arm_mat_init_f32(&matrix_temp2, A_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_mult_f32(&A_matrix, &vcovariance_matrix, &matrix_temp2);

	//初始化matrix_temp3,用于存放A*vcovariance*A^T
	arm_mat_init_f32(&matrix_temp3, matrix_temp2.numRows, matrix_temp1.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp3.numRows*matrix_temp3.numCols;
	arm_mat_mult_f32(&matrix_temp2, &matrix_temp1, &matrix_temp3);

	//预测协方差
	arm_mat_add_f32(&matrix_temp3, &process_noise_covariance_matrix, &vcovariance_matrix);

}

void Kalman_Line_Class::Cal_Kalman_Gain(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放vcovariance+measurement_noise_covariance
	arm_mat_init_f32(&matrix_temp1, vcovariance_matrix.numRows, measurement_noise_covariance_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_add_f32(&vcovariance_matrix, &measurement_noise_covariance_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放(vcovariance+measurement_noise_covariance)^(-1)
	arm_mat_init_f32(&matrix_temp2, matrix_temp1.numRows, matrix_temp1.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_inverse_f32(&matrix_temp1, &matrix_temp2);

	//计算卡尔曼增益矩阵
	arm_mat_mult_f32(&vcovariance_matrix, &matrix_temp2, &gain_matrix);

}

void Kalman_Line_Class::Update_State_Variable(const arm_matrix_instance_f32 & measurement)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2, matrix_temp3;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放measurement-state
	arm_mat_init_f32(&matrix_temp1, measurement.numRows, measurement.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_sub_f32(&measurement, &state_variable_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放gain*(measurement-state)
	arm_mat_init_f32(&matrix_temp2, gain_matrix.numRows, matrix_temp1.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_mult_f32(&gain_matrix, &matrix_temp1, &matrix_temp2);

	//初始化matrix_temp3,用于存放state+gain*(measurement-state)
	arm_mat_init_f32(&matrix_temp3, state_variable_matrix.numRows, matrix_temp2.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp3.numRows*matrix_temp3.numCols;
	arm_mat_add_f32(&state_variable_matrix, &matrix_temp2, &matrix_temp3);

	//保存新的状态量
	int data_size = matrix_temp3.numRows*matrix_temp3.numCols;
	float *source_data = state_variable_matrix.pData;
	float *pdata = matrix_temp3.pData;

	for (int i = 0; i < data_size; i++)
	{
		*source_data = *pdata;
		source_data++;
		pdata++;
	}
}

void Kalman_Line_Class::Update_Covariance(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放gain*vcovariance
	arm_mat_init_f32(&matrix_temp1, gain_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_mult_f32(&gain_matrix, &vcovariance_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放vcovariance-gain*vcovariance
	arm_mat_init_f32(&matrix_temp2, vcovariance_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_sub_f32(&vcovariance_matrix, &matrix_temp1, &matrix_temp2);

	//保存新的协方差
	int data_size = matrix_temp2.numRows*matrix_temp2.numCols;
	float *source_data = vcovariance_matrix.pData;
	float *pdata = matrix_temp2.pData;

	for (int i = 0; i < data_size; i++)
	{
		*source_data = *pdata;
		source_data++;
		pdata++;
	}
}
