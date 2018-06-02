#include "Kalman_Filter_Angle.h"

void Kalman_Angle_Class::Init(void)
{
	arm_mat_init_f32(&process_matrix, 1, 1, &process_data);
	arm_mat_init_f32(&measurement_matrix, 2, 1, measurement_data);

	arm_mat_init_f32(&state_variable_matrix, 2, 1, state_variable_data);
	arm_mat_init_f32(&vcovariance_matrix, 2, 2, vcovariance_data);

	arm_mat_init_f32(&A_matrix, 2, 2, A_data);
	arm_mat_init_f32(&B_matrix, 2, 1, B_data);

	arm_mat_init_f32(&process_noise_covariance_matrix, 2, 2, process_noise_covariance_matrix_data);
	arm_mat_init_f32(&measurement_noise_covariance_matrix, 2, 2, measurement_noise_covariance_matrix_data);

	arm_mat_init_f32(&gain_matrix, 2, 2, gain_matrix_data);

	A_data[0] = 0.0f;
	A_data[1] = 0.0f;
	A_data[2] = 0.0f;
	A_data[3] = -1.0f;

	B_data[1] = 2.0f;

	//初始置信度
	for (int i = 0; i < 4; i++)
	{
		vcovariance_data[i] = 0.0f;
	}

	state_variable_data[0] = state_variable_data[1] = 0.0f;
}

//************************************
// Method:    Set_Noise
// FullName:  Kalman_Angle_Class::Set_Noise
// Access:    public 
// Returns:   void
// Parameter: float noise 噪声度量，在此处为时间
// Description: 更新滤波器的执行、测量噪声
//************************************
void Kalman_Angle_Class::Set_Noise(float noise)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	float *matrix_data_temp = this->matrix_data_temp;

	arm_matrix_instance_f32 noise_matrix;

	arm_mat_init_f32(&noise_matrix, 1, 1, &noise_process_omega);

	B_data[0] = noise;
	//初始化matrix_temp1,用于存放B^T
	arm_mat_init_f32(&matrix_temp1, B_matrix.numCols, B_matrix.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_trans_f32(&B_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放B*noise
	arm_mat_init_f32(&matrix_temp2, B_matrix.numRows, noise_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_mult_f32(&B_matrix, &noise_matrix, &matrix_temp2);
	//matrix_temp2.pData[0] = B_matrix.pData[0] * noise_process_omega;
	//matrix_temp2.pData[1] = B_matrix.pData[1] * noise_process_omega;

	//更新执行噪声
	arm_mat_mult_f32(&matrix_temp2, &matrix_temp1, &process_noise_covariance_matrix);

	float noise_temp = noise_measurement_omega*noise*noise/4;

	//更新测量噪声
	measurement_noise_covariance_matrix_data[0] = noise_temp;
	measurement_noise_covariance_matrix_data[1] = noise_measurement_omega*noise/2;
	measurement_noise_covariance_matrix_data[2] = noise_measurement_omega*noise/2;
	measurement_noise_covariance_matrix_data[3] = noise_measurement_omega;
}

//************************************
// Method:    Kalman_Filter
// FullName:  Kalman_Angle_Class::Kalman_Filter
// Access:    public 
// Returns:   arm_matrix_instance_f32 &
// Parameter: const arm_matrix_instance_f32 & input 执行量
// Parameter: const arm_matrix_instance_f32 & measurement 测量量
// Description: 使用卡尔曼滤波器计算新的状态量
//************************************
arm_matrix_instance_f32 & Kalman_Angle_Class::Kalman_Filter(const arm_matrix_instance_f32 & input, const arm_matrix_instance_f32 & measurement)
{
	Forecast_State_Variable_Process_Model(input);	//预测状态量
	Forecast_Covariance();	//预测协方差矩阵
	Cal_Kalman_Gain();	//计算卡尔曼增益矩阵
	Update_State_Variable(measurement);	//更新状态量
	Update_Covariance();	//更新协方差矩阵

	return state_variable_matrix;
}


//************************************
// Method:    Forecast_State_Variable_Process_Model
// FullName:  Kalman_Angle_Class::Forecast_State_Variable_Process_Model
// Access:    private 
// Returns:   void
// Parameter: const arm_matrix_instance_f32 & input
// Description: 预测状态量
//************************************
void Kalman_Angle_Class::Forecast_State_Variable_Process_Model(const arm_matrix_instance_f32 & input)
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

//************************************
// Method:    Forecast_Covariance
// FullName:  Kalman_Angle_Class::Forecast_Covariance
// Access:    private 
// Returns:   void
// Parameter: void
// Description: 预测协方差矩阵
//************************************
void Kalman_Angle_Class::Forecast_Covariance(void)
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

//************************************
// Method:    Cal_Kalman_Gain
// FullName:  Kalman_Angle_Class::Cal_Kalman_Gain
// Access:    private 
// Returns:   void
// Parameter: void
// Description: 计算卡尔曼增益矩阵
//************************************
void Kalman_Angle_Class::Cal_Kalman_Gain(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放vcovariance+measurement_noise_covariance
	arm_mat_init_f32(&matrix_temp1, vcovariance_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_add_f32(&vcovariance_matrix, &measurement_noise_covariance_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放(vcovariance+measurement_noise_covariance)^(-1)
	arm_mat_init_f32(&matrix_temp2, matrix_temp1.numCols, matrix_temp1.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_inverse_f32(&matrix_temp1, &matrix_temp2);

	//计算卡尔曼增益矩阵
	arm_mat_mult_f32(&vcovariance_matrix, &matrix_temp2, &gain_matrix);
}

//************************************
// Method:    Update_State_Variable
// FullName:  Kalman_Angle_Class::Update_State_Variable
// Access:    private 
// Returns:   void
// Parameter: const arm_matrix_instance_f32 & measurement
// Description: 更新状态量
//************************************
void Kalman_Angle_Class::Update_State_Variable(const arm_matrix_instance_f32 & measurement)
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
	arm_mat_init_f32(&matrix_temp3, state_variable_matrix.numRows, state_variable_matrix.numCols, matrix_data_temp);
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

void Kalman_Angle_Class::Update_Covariance(void)
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
