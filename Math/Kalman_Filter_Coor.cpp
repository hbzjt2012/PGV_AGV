#include "Kalman_Filter_Coor.h"
#include "Trigonometric.h"

void Kalman_Coor_Class::Init(void)
{
	arm_mat_init_f32(&process_matrix, 3, 1, process_data);
	arm_mat_init_f32(&measurement_matrix, 3, 1, measurement_data);

	arm_mat_init_f32(&state_variable_matrix, 3, 1, state_variable_data);
	arm_mat_init_f32(&vcovariance_matrix, 3, 3, vcovariance_data);

	arm_mat_init_f32(&B_matrix, 3, 3, B_data);

	arm_mat_init_f32(&process_noise_covariance_matrix, 3, 3, process_noise_covariance_matrix_data);
	arm_mat_init_f32(&measurement_noise_covariance_matrix, 3, 3, measurement_noise_covariance_matrix_data);

	arm_mat_init_f32(&gain_matrix, 3, 3, gain_matrix_data);

	B_data[2] = 0.0f;
	B_data[5] = 0.0f;
	B_data[6] = 0.0f;
	B_data[7] = 0.0f;
	B_data[8] = 1.0f;

	for (int i = 0; i < 9; i++)
	{
		vcovariance_data[i] = 0.0f;
		measurement_noise_covariance_matrix_data[i] = 0.0f;
	}

	//初始置信度
	vcovariance_data[0] = 10000.0f;
	vcovariance_data[4] = 10000.0f;
	vcovariance_data[8] = 10000.0f;

	measurement_noise_covariance_matrix_data[0] = noise_measurement_coor;
	measurement_noise_covariance_matrix_data[4] = noise_measurement_coor;
	measurement_noise_covariance_matrix_data[8] = noise_measurement_coor;

	state_variable_data[0] = state_variable_data[1] = state_variable_data[2] = 0.0f;

}

void Kalman_Coor_Class::Update_Process_Noise(const arm_matrix_instance_f32 & input_noise)
{
	Set_Noise(angle_coor);	//更新控制转移矩阵B

	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放B^T
	arm_mat_init_f32(&matrix_temp1, B_matrix.numCols, B_matrix.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	arm_mat_trans_f32(&B_matrix, &matrix_temp1);

	//初始化matrix_temp2,用于存放B*noise
	arm_mat_init_f32(&matrix_temp2, B_matrix.numRows, input_noise.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_mult_f32(&B_matrix, &input_noise, &matrix_temp2);

	//更新执行噪声
	arm_mat_mult_f32(&matrix_temp2, &matrix_temp1, &process_noise_covariance_matrix);
}

arm_matrix_instance_f32 & Kalman_Coor_Class::Kalman_Filter(const arm_matrix_instance_f32 & input, const arm_matrix_instance_f32 & measurement)
{
	Forecast_State_Variable_Process_Model(input);	//预测状态量
	Forecast_Covariance();	//预测协方差矩阵
	Cal_Kalman_Gain();	//计算卡尔曼增益矩阵
	Update_State_Variable(measurement);	//更新状态量
	Update_Covariance();	//更新协方差矩阵

	return state_variable_matrix;
}

arm_matrix_instance_f32 & Kalman_Coor_Class::Update_Stae_Variable_No_Measurement(const arm_matrix_instance_f32 & input)
{
	Forecast_State_Variable_Process_Model(input);	//预测状态量
	Forecast_Covariance();	//预测协方差
	return state_variable_matrix;	//返回状态量
}

void Kalman_Coor_Class::Set_Noise(float noise)
{
	float sin_temp = Sin_Lookup(noise);
	float cos_temp = Cos_Lookup(noise);

	B_data[0] = cos_temp;
	B_data[1] = -sin_temp;
	B_data[3] = sin_temp;
	B_data[4] = cos_temp;
}

void Kalman_Coor_Class::Forecast_State_Variable_Process_Model(const arm_matrix_instance_f32 & input)
{
	arm_matrix_instance_f32  matrix_temp2, matrix_temp3;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp2,用于存放B*input
	arm_mat_init_f32(&matrix_temp2, B_matrix.numRows, input.numCols, matrix_data_temp);
	matrix_data_temp += matrix_temp2.numRows*matrix_temp2.numCols;
	arm_mat_mult_f32(&B_matrix, &input, &matrix_temp2);

	//初始化matrix_temo3,用于存放预测状态量
	arm_mat_init_f32(&matrix_temp3, matrix_temp2.numRows, matrix_temp2.numCols, matrix_data_temp);
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

void Kalman_Coor_Class::Forecast_Covariance(void)
{
	arm_matrix_instance_f32 matrix_temp1;
	float *matrix_data_temp = this->matrix_data_temp;

	//初始化matrix_temp1,用于存放预测协方差
	arm_mat_init_f32(&matrix_temp1, vcovariance_matrix.numCols, vcovariance_matrix.numRows, matrix_data_temp);
	matrix_data_temp += matrix_temp1.numRows*matrix_temp1.numCols;
	//预测协方差
	arm_mat_add_f32(&vcovariance_matrix, &process_noise_covariance_matrix, &matrix_temp1);

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
}


void Kalman_Coor_Class::Cal_Kalman_Gain(void)
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


void Kalman_Coor_Class::Update_State_Variable(const arm_matrix_instance_f32 & measurement)
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

void Kalman_Coor_Class::Update_Covariance(void)
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
