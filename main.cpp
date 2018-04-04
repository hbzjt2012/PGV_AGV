#include "main.h"
#include <cstring>

//bool encoder_flag = false;
//float encoder_cnt = 0;
//float encoder_time = 0.0f;

int main(void)
{
	Init_System();
	My_Serial.enable();
	//AGV_Current_V_InAGV_Coor_InWorld.Coordinate.angle_coor = 90.0f;
	Gcode_M17();

	while (1)
	{
		//计算当前位姿

		AGV_Current_V_InAGV_Coor_InWorld = Mecanum_AGV.Update_Post(AGV_Current_V_InAGV_Coor_InWorld, AGV_Target_V_InAGV_Coor_InWorld.Velocity); //根据编码器更新速度和坐标
		//融合陀螺仪得到的速度和坐标
		//融合PGV传感器得到的速度和坐标

		//encoder_flag = Mecanum_AGV.demo(AGV_Target_V_InAGV_Coor_InWorld.Velocity, encoder_cnt, encoder_time);

		Get_Available_Command(command_buf_state); //获取处理当前指令(已完成)
		//检查避障
		//计算实际所需速度(需要当前坐标、当前速度，预期坐标，预期速度)
		Mecanum_AGV.Write_Velocity(AGV_Target_V_InAGV_Coor_InWorld.Velocity); //运动控制

		//if (encoder_flag)
		//{
		//	My_Serial.print(encoder_cnt);
		//	My_Serial.print("  time");
		//	My_Serial.print(encoder_time);
		//}

		Update_Print_MSG(); //更新信息(待补全)
	}
}

void Init_System(void)
{
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置系统中断优先级分组 2
	Init_System_RCC();
	Led.Init(GPIO_Mode_OUT);
	Mecanum_AGV.Init();
	My_Serial.Init(115200);
	Gcode_Queue.Init();
}

void Init_System_RCC(void)
{
	//复位外设DMA1-2、GPIOA-GPIOE
	RCC->AHB1RSTR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(21) | _BV(22));
	//复位外设TIM2-7、TIM12-14、USART2-5
	RCC->APB1RSTR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(17) | _BV(18) | _BV(19) | _BV(20));
	//复位外设TIM1、TIM8、USART1、USART6、ADC、SPI1、TIM9-11
	RCC->APB2RSTR |= (_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(8) || _BV(12) | _BV(16) | _BV(17) | _BV(18));

	//外设复位结束
	RCC->AHB1RSTR &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(21) | _BV(22));
	RCC->APB1RSTR &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(17) | _BV(18) | _BV(19) | _BV(20));
	RCC->APB2RSTR &= ~(_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(8) || _BV(12) | _BV(16) | _BV(17) | _BV(18));

	//使能外设DMA1-2，GPIOA-GPIOE时钟
	RCC->AHB1ENR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(21) | _BV(22));
	//使能外设TIM2-7、TIM12-14、USART2-5
	RCC->APB1ENR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(17) | _BV(18) | _BV(19) | _BV(20));
	//使能外设TIM1、TIM8、USART1、USART6、ADC、SPI1、TIM9-11
	RCC->APB2ENR |= (_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(8) | _BV(12) | _BV(16) | _BV(17) | _BV(18));
}

//************************************
// Method:    Get_Available_Command
// FullName:  Get_Available_Command
// Access:    public
// Returns:   void
// Parameter: AGV_State::Command_State::Get_Command_State & state 指令状态
// Description: 接收并处理指令
//				前提：插入指令不会造成堵塞
//************************************
void Get_Available_Command(AGV_State::Command_State::Get_Command_State &state)
{
	state = AGV_State::Command_State::Get_Command_State::No_Action; //无动作
	static bool Is_Parsing_Command = false;							//表示是否在处理指令
	if (My_Serial.Return_rx_flag())									//获取到了新指令
	{
		My_Serial.Clear_rx_flag();
		My_Serial.Clear_rx_cnt();

		int parse_result = Gcode_Inject.parse(My_Serial.Return_RX_buf(), agv_add, command_line + 1); //解析指令，获取解析结果
		if (parse_result == 0)																		 //解析成功
		{
			if (Gcode_Inject.command_letter == 'I') //判断是否为插入指令
			{
				//++command_line;	//指令行数+1	//插入指令不增加指令行数
				Process_Command(&Gcode_Inject, Is_Parsing_Command); //处理指令
			}
			else //不为插入指令
			{
				if (Gcode_Queue.queue_state == Queue_Class::BUFFER_FULL) //指令缓存区满
				{
					state = AGV_State::Command_State::Get_Command_State::BUSY;
				}
				else
				{
					int size = strlen(Gcode_Inject.Return_Command());
					Gcode_Index_w = Gcode_Buf + Gcode_Queue.ENqueue(); //入队
					memcpy(Gcode_Index_w->Return_Command(), Gcode_Inject.Return_Command(), size + 1);
					Gcode_Index_w->command_letter = Gcode_Inject.command_letter;
					Gcode_Index_w->codenum = Gcode_Inject.codenum;
					Gcode_Index_w->Parse_State = Gcode_Class::NO_PARSE; //当前指令未执行
					state = AGV_State::Command_State::Get_Command_State::OK;
					++command_line; //指令行数+1
				}
			}
		}
		else if (parse_result > 0)
		{
			state = AGV_State::Command_State::Get_Command_State::ERROR; //解析指令出错
		}
	}
	if (!Is_Parsing_Command) //当前没有在处理指令
	{
		if (Gcode_Queue.queue_state != Queue_Class::BUFFER_EMPTY) //缓存区不为空
		{
			Gcode_Index_r = Gcode_Buf + Gcode_Queue.DEqueue();  //获取队头
			Process_Command(Gcode_Index_r, Is_Parsing_Command); //处理指令
		}
	}
	else
	{
		Process_Command(Gcode_Index_r, Is_Parsing_Command); //处理指令
	}
}

//************************************
// Method:    Process_Command
// FullName:  Process_Command
// Access:    public
// Returns:   void
// Parameter: Gcode_Class * command
// Parameter: bool & IS_Parsing 指示当前是否在处理指令
// Description: 执行相应指令函数
//************************************
void Process_Command(Gcode_Class *command, bool &IS_Parsing)
{
	int codenum = command->codenum;
	switch (command->command_letter)
	{
	case 'G':
		switch (codenum)
		{
		case 0:
			//Gcode_G0(command, AGV_Current_V_InAGV_Coor_InWorld.Coordinate, AGV_Target_V_InAGV_Coor_InWorld);
			//break;
		case 1:
			Gcode_G1(command, AGV_Current_V_InAGV_Coor_InWorld.Coordinate, AGV_Target_V_InAGV_Coor_InWorld);
			break;
		default:
			break;
		}
		IS_Parsing = (command->Parse_State == Gcode_Class::IS_PARSED) ? false : true; //更新指令处理状态
		break;
	case 'M':
		switch (codenum)
		{
		case 17:
			Gcode_M17();
			command->Parse_State = Gcode_Class::IS_PARSED;
			break;
		case 18:
			Gcode_M18();
			command->Parse_State = Gcode_Class::IS_PARSED;
			break;
		default:
			break;
		}
		IS_Parsing = (command->Parse_State == Gcode_Class::IS_PARSED) ? false : true;												//更新指令处理状态
		AGV_Current_V_InAGV_Coor_InWorld.Coordinate = Position_Class::Truncation_Coor(AGV_Current_V_InAGV_Coor_InWorld.Coordinate); //对当前坐标保留1位小数
		break;
	case 'I':
		switch (codenum)
		{
		case 0:
			Gcode_I0();
			command->Parse_State = Gcode_Class::IS_PARSED;
			break;
		case 114:
			Gcode_I114();
			command->Parse_State = Gcode_Class::IS_PARSED;
			break;
		default:
			break;
		}
		break;
	default:
		command->Parse_State = Gcode_Class::IS_PARSED; //无法识别的指令
		break;
	}
}

//************************************
// Method:    Update_Print_MSG
// FullName:  Update_Print_MSG
// Access:    public
// Returns:   void
// Parameter: void
// Description:	更新小车状态，打印信息
//************************************
void Update_Print_MSG(void)
{
	switch (command_buf_state)
	{
	case AGV_State::Command_State::Get_Command_State::BUSY:
		My_Serial.print("\r\nBusy"); //状态繁忙
		break;
	case AGV_State::Command_State::Get_Command_State::OK:
		My_Serial.print("\r\nOK"); //状态正常
		My_Serial.print("  Next Line:");
		My_Serial.print(command_line + 1);
		break;
	case AGV_State::Command_State::Get_Command_State::ERROR:
		My_Serial.print("\r\nCommand Error:");
		My_Serial.print(My_Serial.Return_RX_buf());
		My_Serial.print("  Next Line:N");
		My_Serial.print(command_line + 1); //指令错误
		break;
	default:
		break;
	}
	//if (My_Serial.Return_tx_cnt() > 0)
	//{
	//	My_Serial.print("\r\n");
	//	My_Serial.flush();
	//}
	My_Serial.flush();
}

//************************************
// Method:    Get_Command_Coor
// FullName:  Get_Command_Coor
// Access:    public
// Returns:   void
// Parameter: Gcode_Class * command
// Parameter: const Position_Class::Coordinate_StructTypedef & Current_Coor_InWorld	当前坐标
// Parameter: Position_Class::Coordinate_StructTypedef & Target_Coor_InWorld 期望坐标
// Description: 获取指令中给定的坐标，未指定参数则设置为当前值
//************************************
void Get_Command_Coor(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class::Coordinate_Class &Target_Coor_InWorld)
{
	char *add = 0;
	const char *command_add = command->Return_Command();

	add = strchr(command_add, 'X');
	Target_Coor_InWorld.x_coor = add ? (atof(add + 1)) : Current_Coor_InWorld.x_coor; //获取x轴坐标
	add = strchr(command_add, 'Y');
	Target_Coor_InWorld.y_coor = add ? (atof(add + 1)) : Current_Coor_InWorld.y_coor; //获取y轴坐标
	add = strchr(command_add, 'C');
	Target_Coor_InWorld.angle_coor = add ? (atof(add + 1)) : Current_Coor_InWorld.angle_coor; //获取angle轴坐标
	if (!Is_Absolute_Coor)
	{
		Target_Coor_InWorld += Current_Coor_InWorld;
	}
	Target_Coor_InWorld = Position_Class::Truncation_Coor(Target_Coor_InWorld);
}

//同G1
inline void Gcode_G0(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class &Target_V_InAGV_Coor_InWorld)
{
	Gcode_G1(command, Current_Coor_InWorld, Target_V_InAGV_Coor_InWorld);
}

//************************************
// Method:    Gcode_G1
// FullName:  Gcode_G1
// Access:    public
// Returns:   void
// Parameter: Gcode_Class * command
// Parameter: const Position_Class::Coordinate_Class & Current_Coor_InWorld 当前坐标
// Parameter: Position_Class& Target_V_InAGV_Coor_InWorld 预期坐标(世界坐标系),预期速度(AGV坐标系)
// Description: 直线插补目标点，获取下一时刻的预期坐标(世界坐标系),预期速度(AGV坐标系)
//************************************
void Gcode_G1(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class &Target_V_InAGV_Coor_InWorld)
{
	static Position_Class::Coordinate_Class Origin_Coor_InWorld, Destination_Coor_InWorld;	//世界坐标系下的起点坐标，终点坐标
	static Position_Class::Coordinate_Class Current_Coor_InOrigin, Destination_Coor_InOrigin; //起点坐标系下的当前坐标，终点坐标
	static bool Is_Interpolation_Angle = false;												  //指示当前是否在插补角度，处理顺序为，先插补x，y，再插补角度，角度也插补完成，表示G1指令处理完成
	static bool Is_X_Coor = true;															  //指示当前插补的是x轴
	float current_coor = 0.0f;																  //指示当前坐标对应的理想AGV在轨迹上运动的距离（位移或速度）

	static int singal_Destination_Coor_InOrigin = 1; //终点坐标在起点坐标中的符号位，若待插补距离<0，则为-1，否则为1

	switch (command->Parse_State)
	{
	case Gcode_Class::NO_PARSE: //接收到指令，对插补做准备工作

		Interpolation::Actual_INPUT_TypedefStructure Para_Input; //用于插补的输入参数

		command->Parse_State = Gcode_Class::IS_PARSING; //切换执行状态至正在执行

		if (!Is_Interpolation_Angle) //表示插补x,y轴
		{
			Origin_Coor_InWorld = Current_Coor_InWorld;								  //保存起点坐标
			Get_Command_Coor(command, Origin_Coor_InWorld, Destination_Coor_InWorld); //获取终点坐标
			//获取终点坐标系在起点坐标系中的坐标
			Destination_Coor_InOrigin = Position_Class::Absolute_To_Relative(Destination_Coor_InWorld, Destination_Coor_InOrigin, Origin_Coor_InWorld);
			Para_Input.acceleration_abs = AGV_MAX_LINE_ACCELERATION_ACCELERATION / (1000.0f * 1000.0f); //单位转换
			float abs_x = ABS(Destination_Coor_InOrigin.x_coor);
			float abs_y = ABS(Destination_Coor_InOrigin.y_coor);
			if (abs_x > abs_y) //对x轴插补
			{
				Is_X_Coor = true;
				if (Destination_Coor_InOrigin.x_coor > 0.0f)
				{
					singal_Destination_Coor_InOrigin = 1;
					Para_Input.displacement = Destination_Coor_InOrigin.x_coor;
				}
				else
				{
					singal_Destination_Coor_InOrigin = -1;
					Para_Input.displacement = -Destination_Coor_InOrigin.x_coor;
				}

				//singal_Destination_Coor_InOrigin = Destination_Coor_InOrigin.x_coor > 0.0f ? 1 : -1;
				//Para_Input.displacement = Destination_Coor_InOrigin.x_coor*singal_Destination_Coor_InOrigin;
				Para_Input.max_velocity_abs = AGV_MAX_LINE_VELOCITY * abs_x / (abs_x + abs_y) / 1000.0f;
				Para_Input.min_velocity_abs = AGV_MIN_LINE_VELOCITY * abs_x / (abs_x + abs_y) / 1000.0f;
			}
			else //对y轴插补
			{
				//singal_Destination_Coor_InOrigin = Destination_Coor_InOrigin.y_coor > 0.0f ? 1 : -1;
				Is_X_Coor = false;

				if (Destination_Coor_InOrigin.y_coor > 0.0f)
				{
					singal_Destination_Coor_InOrigin = 1;
					Para_Input.displacement = Destination_Coor_InOrigin.y_coor;
				}
				else
				{
					singal_Destination_Coor_InOrigin = -1;
					Para_Input.displacement = -Destination_Coor_InOrigin.y_coor;
				}

				//Para_Input.displacement = Destination_Coor_InOrigin.y_coor*singal_Destination_Coor_InOrigin;
				Para_Input.max_velocity_abs = AGV_MAX_LINE_VELOCITY * abs_y / (abs_x + abs_y) / 1000.0f;
				Para_Input.min_velocity_abs = AGV_MIN_LINE_VELOCITY * abs_y / (abs_x + abs_y) / 1000.0f;
			}
			Para_Input.slow_distance_abs = 20.0f;
		}
		else //插补角度
		{
			float angle_delta = Destination_Coor_InOrigin.angle_coor - Current_Coor_InOrigin.angle_coor;

			if (angle_delta > 0.0f)
			{
				singal_Destination_Coor_InOrigin = 1;
				Para_Input.displacement = angle_delta;
			}
			else
			{
				singal_Destination_Coor_InOrigin = -1;
				Para_Input.displacement = -angle_delta;
			}

			//singal_Destination_Coor_InOrigin = Destination_Coor_InOrigin.angle_coor > 0.0f ? 1 : -1;
			Para_Input.acceleration_abs = AGV_MAX_LINE_ACCELERATION_ACCELERATION / (1000.0f * 1000.0f); //单位转换
			Para_Input.max_velocity_abs = AGV_MAX_ANGULAR_VELOCITY / 1000.0f;
			Para_Input.min_velocity_abs = AGV_MIN_ANGULAR_VELOCITY / 1000.0f;

			Para_Input.slow_distance_abs = 10.0f;	//低速最小角度10°

			////下列参数和车型有关，以下为麦克纳姆轮四轮车的最大最小角速度、角加速度
			//Para_Input.acceleration_abs = 2 * WHEEL_MAX_ANGULAR_ACCELERATION / (1000.0f * 1000.0f)*WHEEL_DIAMETER/(DISTANCE_OF_WHEEL_X_AXES+ DISTANCE_OF_WHEEL_Y_AXES);
			////Para_Input.displacement = ABS(Destination_Coor_InOrigin.angle_coor);
			//Para_Input.max_velocity_abs = WHEEL_MAX_ANGULAR_VELOCITY / 1000.0f;
			//Para_Input.min_velocity_abs = WHEEL_MIN_LINE_VELOCITY*abs_x / (abs_x + abs_y) / 1000.0f;
			//Para_Input.min_velocity_abs = 0;
		}
		//根据起点、终点坐标插补速度
		Interpolation::Init(Para_Input);

		//插补工作完成，直接进入插补
		//break;
	case Gcode_Class::IS_PARSING: //对插补的准备工作已完成，正在插补
		//获取当前坐标在起点坐标系上的坐标
		Current_Coor_InOrigin = Position_Class::Absolute_To_Relative(Current_Coor_InWorld, Current_Coor_InOrigin, Origin_Coor_InWorld);

		//获取在轨迹上的位移
		if (!Is_Interpolation_Angle) //在对x,y轴进行插补
		{
			MyMath::Coor coor_temp1, coor_temp2; //过点1，和路径的垂直点2
			coor_temp1.x = Current_Coor_InOrigin.x_coor;
			coor_temp1.y = Current_Coor_InOrigin.y_coor;
			//获取垂直线的交点
			if (ABS(Destination_Coor_InOrigin.x_coor) < FLOAT_DELTA)
			{
				coor_temp2.y = coor_temp1.y;
				coor_temp2.x = Destination_Coor_InOrigin.x_coor;
			}
			else
			{
				MyMath::Get_Vertical_Line_Crossover_Point(Destination_Coor_InOrigin.y_coor / Destination_Coor_InOrigin.x_coor, coor_temp1, coor_temp2);
			}
			//获取在插补路径上移动的距离
			current_coor = singal_Destination_Coor_InOrigin * (Is_X_Coor ? coor_temp2.x : coor_temp2.y);
		}
		else //对角度进行插补
		{
			current_coor = singal_Destination_Coor_InOrigin * (Current_Coor_InOrigin.angle_coor); //获取在角度上移动的距离
		}

		//获取插补速度
		if (!Is_Interpolation_Angle) //在对x,y轴进行插补
		{
			float velocity_temp = 0.0f;
			if (Interpolation::Get_Expectation(velocity_temp, ABS(current_coor)))
			{

				//////////////////////////////////////////////////////////////////////////
				//对于除数0处理错误，记得修正
				//已修正，判断除数的绝对值，若小于某个值，则认为是0
				//////////////////////////////////////////////////////////////////////////
				if (Is_X_Coor)
				{
					//if (Destination_Coor_InOrigin.x_coor < 0.0f)
					//{
					//	velocity_temp = -velocity_temp;
					//}
					Target_V_InAGV_Coor_InWorld.Velocity.x_velocity = singal_Destination_Coor_InOrigin * velocity_temp * 1000.0f;
					if (ABS(Destination_Coor_InOrigin.x_coor) < FLOAT_DELTA)
					{
						Target_V_InAGV_Coor_InWorld.Velocity.y_velocity = 0.0f;
					}
					else
					{
						Target_V_InAGV_Coor_InWorld.Velocity.y_velocity = Destination_Coor_InOrigin.y_coor / Destination_Coor_InOrigin.x_coor * singal_Destination_Coor_InOrigin * velocity_temp * 1000.0f;
					}
					Target_V_InAGV_Coor_InWorld.Velocity.yaw_velocity = 0.0f;
				}
				else
				{
					//if (Destination_Coor_InOrigin.y_coor < 0.0f)
					//{
					//	velocity_temp = -velocity_temp;
					//}

					if (ABS(Destination_Coor_InOrigin.y_coor) < FLOAT_DELTA)
					{
						Target_V_InAGV_Coor_InWorld.Velocity.x_velocity = 0.0f;
					}
					else
					{
						Target_V_InAGV_Coor_InWorld.Velocity.x_velocity = Destination_Coor_InOrigin.x_coor / Destination_Coor_InOrigin.y_coor * singal_Destination_Coor_InOrigin * velocity_temp * 1000.0f;
					}
					Target_V_InAGV_Coor_InWorld.Velocity.y_velocity = singal_Destination_Coor_InOrigin * velocity_temp * 1000.0f;
					Target_V_InAGV_Coor_InWorld.Velocity.yaw_velocity = 0.0f;
				}
			}
			else //表示当前插补完成
			{
				Target_V_InAGV_Coor_InWorld.Velocity.x_velocity = 0.0f;
				Target_V_InAGV_Coor_InWorld.Velocity.y_velocity = 0.0f;
				Target_V_InAGV_Coor_InWorld.Velocity.yaw_velocity = 0.0f;
				Is_Interpolation_Angle = true; //对角度进行插补
				command->Parse_State = Gcode_Class::NO_PARSE;
			}
		}
		else //对角度插补
		{
			float velocity_temp = 0.0f;
			if (Interpolation::Get_Expectation(velocity_temp, ABS(current_coor)))
			{
				//if (Destination_Coor_InOrigin.angle_coor < 0.0f)
				//{
				//	velocity_temp = -velocity_temp;
				//}
				Target_V_InAGV_Coor_InWorld.Velocity.yaw_velocity = singal_Destination_Coor_InOrigin * velocity_temp * 1000.0f;
			}
			else
			{
				Target_V_InAGV_Coor_InWorld.Velocity.yaw_velocity = 0.0f;
				Is_Interpolation_Angle = false;				   //对x,y轴进行插补
				command->Parse_State = Gcode_Class::IS_PARSED; //执行完毕
			}
			Target_V_InAGV_Coor_InWorld.Velocity.x_velocity = 0.0f;
			Target_V_InAGV_Coor_InWorld.Velocity.y_velocity = 0.0f;
		}
		break;
	case Gcode_Class::IS_PARSED: //插补完成

		break;
	default:
		break;
	}
}

//启动所有电机
void Gcode_M17(void)
{
	Mecanum_AGV.Brake(false);
}

//禁用所有电机
void Gcode_M18(void)
{
	Mecanum_AGV.Brake(true);
}

//紧急停止
void Gcode_I0(void)
{
	Mecanum_AGV.Brake(true);
	Gcode_Queue.Init();
}

//返回AGV在世界坐标系中的坐标
void Gcode_I114(void)
{
	My_Serial.print("\r\nx:");
	My_Serial.print(Mecanum_AGV.V_InAGV_Coor_InWorld.Coordinate.x_coor);
	My_Serial.print("  y:");
	My_Serial.print(Mecanum_AGV.V_InAGV_Coor_InWorld.Coordinate.y_coor);
	My_Serial.print("  angle:");
	My_Serial.print(Mecanum_AGV.V_InAGV_Coor_InWorld.Coordinate.angle_coor);
	My_Serial.print("\r\n");
}