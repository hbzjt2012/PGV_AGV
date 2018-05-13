int main(void)
{
	while (1)
	{
	}
}

//#include "main.h"
//#include <cstring>
//
//Mecanum_Wheel_Class Mecanum_AGV;							   //麦克纳姆轮
//IO_Class Led = IO_Class(GPIOA, GPIO_Pin_15);				   //LED指示灯
//C50XB_Class My_Serial;										   //无线串口
//PGV_Class PGV100;												//PGV传感器
//Queue_Class Gcode_Queue = Queue_Class(4);					   //存放Gcode指令用的队列
//AGV_State::Command_State::Get_Command_State command_buf_state; //指令缓存区的状态
//Gcode_Class Gcode_Buf[4], Gcode_Inject;						   //指令暂存区，插入指令暂存区
//Gcode_Class *Gcode_Index_w = 0, *Gcode_Index_r = 0;			   //指令暂存区读写下标
//TL740D_Class TL740;
//
//Position_Class AGV_Target_Position_InWorld, AGV_Current_Position_InWorld;	//AGV在世界坐标系下的目标位姿和速度，在AGV坐标系下的当前位姿和速度
//Position_Class AGV_Target_Position_InAGV, AGV_Current_Position_InAGV;	//AGV在AGV坐标系下的目标位姿和速度,在AGV坐标系下的当前位姿和速度
//Position_Class AGV_Current_Position_InWorld_By_Encoder;	//世界坐标系下由编码器获取的AGV的坐标和位置
//Position_Class AGV_Current_Position_InWorld_By_PGV;	//世界坐标系下由PGV获取的AGV的坐标和位置
//
//int command_line = 0;		  //表示当前已经接收到的指令行数
//int agv_add = 1;			  //AGV地址号
//bool Is_Absolute_Coor = true; //指示当前坐标是否为绝对坐标
//bool update_coor_by_G92 = false;
//bool update_coor_by_PGV = false;
//bool current_existing_task = false;
//
//bool demo_flag = false;
//
//float theta_rate_by_gyro = 0.0f;
//float theta_by_gyro = 0.0f;
//float theta_rate_by_encoder = 0.0f;
//float theta_by_pgv = 0.0f;
//static long long cnt_temp = 0;
//
//extern "C" {
//	void TIM1_TRG_COM_TIM11_IRQHandler()
//	{
//		if (TIM11->SR & TIM_IT_Update) //更新中断
//		{
//			TIM11->SR = ~TIM_IT_Update;
//			demo_flag = true;
//		}
//	}
//}
//
//int main(void)
//{
//	Init_System();
//
//	My_Serial.enable();
//	Gcode_M17();
//
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		  //响应优先级
//	NVIC_Init(&NVIC_InitStructure);
//
//	TIM_Base_Class::Init(TIM11, 2000, 840, true);	//设置定时器11的中断频率，时基--10ms	
//	TIM_Base_Class::Begin(TIM11);
//
//	AGV_Target_Position_InAGV.Velocity.x_velocity = 0.0f;
//	AGV_Target_Position_InAGV.Velocity.y_velocity = 0.0f;
//
//	while (!TL740.Return_rx_flag())
//	{
//		TL740.Clear_rx_flag();
//	}
//
//	while (1)
//	{
//
//		if (demo_flag)
//		{
//			demo_flag = false;
//			if (cnt_temp % 5 == 0)	//50ms读取一次PGV数据
//			{
//				PGV100.Send(PGV_Class::Read_PGV_Data);	//获取PGV数据
//			}
//
//			if (cnt_temp % 2 == 0)	//20ms更新一次数据
//			{
//
//				AGV_Target_Position_InAGV.Velocity.angle_velocity = ABS(AGV_MAX_ANGULAR_VELOCITY* Sin_Lookup(cnt_temp / 100.0f / (M_PI*3) * 180));	//角速度是关于时间的函数
//
//				theta_rate_by_encoder = Mecanum_AGV.Get_theta_rate(20.0f);	//获取编码器数据
//				My_Serial.print(theta_rate_by_encoder);	//编码器角速度
//				My_Serial.print("  ");
//				My_Serial.print(theta_rate_by_gyro);	//陀螺仪角速度
//				My_Serial.print("  ");
//				My_Serial.print(theta_by_gyro);	//陀螺仪角度
//				My_Serial.print("  ");
//				My_Serial.print(theta_by_pgv);	//PGV角度
//				My_Serial.flush_demo(cnt_temp*0.01f);	//刷新数据
//			}
//
//			cnt_temp++;
//		}
//
//		if (PGV100.Return_rx_flag())	//处理PGV数据
//		{
//			PGV100.Clear_rx_flag();
//			if (PGV100.Analyze_Data() && (PGV100.target == PGV_Class::Data_Matrix_Tag))	//解析PGV数据
//			{
//				PGV100.Cal_Coor();
//				theta_by_pgv = PGV100.angle_deviation;
//			}
//		}
//
//		if (TL740.Return_rx_flag())	//处理陀螺仪数据
//		{
//			TL740.Clear_rx_flag();
//			if (TL740.Analyze_Data())	//解析陀螺仪数据
//			{
//				theta_rate_by_gyro = TL740.z_rate;
//				theta_by_gyro = TL740.z_heading;
//			}
//		}
//
//		Mecanum_AGV.Write_Velocity(AGV_Target_Position_InAGV.Velocity);
//
//
//		//计算当前位姿
//		//AGV_Current_Position_InWorld_By_Encoder = Mecanum_AGV.Update_Post_By_Encoder(AGV_Current_Position_InWorld_By_Encoder, update_coor_by_PGV); //根据编码器更新速度和坐标
//		////融合陀螺仪得到的速度和坐标
//		////融合PGV传感器得到的速度和坐标
//		//if (PGV100.Return_rx_flag())
//		//{
//		//	PGV100.Clear_rx_flag();
//		//	if (PGV100.Analyze_Data() && (PGV100.target == PGV_Class::Data_Matrix_Tag))
//		//	{
//		//		AGV_Current_Position_InWorld_By_PGV.Coordinate = PGV100.Cal_Coor();
//		//		update_coor_by_PGV = true;
//		//		//Gcode_I116();
//		//	}
//		//}
//		//if (demo_flag)
//		//{
//		//	demo_flag = false;
//
//		//	PGV100.Send(PGV_Class::Read_PGV_Data);		
//		//	//AGV_Current_Position_InWorld_By_Encoder = Mecanum_AGV.Update_Post_By_Encoder(AGV_Current_Position_InWorld_By_Encoder); //根据编码器更新速度和坐标
//		//}
//
//		//if (TL740.Return_rx_flag())
//		//{
//		//	TL740.Clear_rx_flag();
//		//	if (TL740.Analyze_Data())
//		//	{
//		//		//if (TL740.z_rate>500.0f)
//		//		//{
//		//		//	My_Serial.print("\r\n");
//		//		//	My_Serial.print(TL740.Return_RX_buf(), 14);
//		//		//}
//		//		My_Serial.print("\r\nw=");
//		//		My_Serial.print(TL740.z_rate);  
//		//		My_Serial.print("   theta=");
//		//		My_Serial.print(TL740.z_heading);
//		//	}
//		//	//TL740.Read_Data();
//		//}
//		////Update_Position_InWorld(AGV_Current_Position_InWorld_By_Encoder); //更新世界坐标系下的坐标和速度(此处需要处理与G92指令的关系)
//		//Update_Coor_InWorld(AGV_Current_Position_InWorld_By_Encoder.Coordinate, AGV_Current_Position_InWorld_By_PGV.Coordinate);
//
//		////AGV_Current_Position_InWorld_By_Encoder.Coordinate.angle_coor=0.0f;
//
//		//Get_Available_Command(command_buf_state); //获取处理当前指令(已完成)
//		////检查避障
//		////计算实际所需速度(需要当前坐标、当前速度，预期坐标，预期速度)
//
//		//Mecanum_AGV.AGV_Control_Class::Write_Velocity(AGV_Current_Position_InWorld.Coordinate, AGV_Target_Position_InWorld.Coordinate, AGV_Target_Position_InAGV.Velocity);
//		////Mecanum_AGV.Write_Velocity(AGV_Target_Position_InAGV.Velocity); //运动控制
//
//
//		//if (encoder_flag)
//		//{
//		//	My_Serial.print(encoder_cnt);
//		//	My_Serial.print("  time");
//		//	My_Serial.print(encoder_time);
//		//}
//
//		//Update_Print_MSG(); //更新信息(待补全)
//	}
//}
//
//void Init_System(void)
//{
//	delay_init();
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置系统中断优先级分组 2
//	Init_System_RCC();
//
//	Led.Init(GPIO_Mode_OUT);
//	for (int i = 0; i < 20; i++)
//	{
//		delay_ms(500);
//	}
//	Mecanum_AGV.Init();
//	My_Serial.Init(115200);
//	//My_Serial.Init(512000);
//	Gcode_Queue.Init();
//	PGV100.Init(115200);
//	TL740.Init(115200);
//}
//
//void Init_System_RCC(void)
//{
//	//复位外设DMA1-2、GPIOA-GPIOE
//	RCC->AHB1RSTR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(21) | _BV(22));
//	//复位外设TIM2-7、TIM12-14、USART2-5
//	RCC->APB1RSTR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(17) | _BV(18) | _BV(19) | _BV(20));
//	//复位外设TIM1、TIM8、USART1、USART6、ADC、SPI1、TIM9-11
//	RCC->APB2RSTR |= (_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(8) || _BV(12) | _BV(16) | _BV(17) | _BV(18));
//
//	//外设复位结束
//	RCC->AHB1RSTR &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(21) | _BV(22));
//	RCC->APB1RSTR &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(17) | _BV(18) | _BV(19) | _BV(20));
//	RCC->APB2RSTR &= ~(_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(8) || _BV(12) | _BV(16) | _BV(17) | _BV(18));
//
//	//使能外设DMA1-2，GPIOA-GPIOE时钟
//	RCC->AHB1ENR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(21) | _BV(22));
//	//使能外设TIM2-7、TIM12-14、USART2-5
//	RCC->APB1ENR |= (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(17) | _BV(18) | _BV(19) | _BV(20));
//	//使能外设TIM1、TIM8、USART1、USART6、ADC、SPI1、TIM9-11
//	RCC->APB2ENR |= (_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(8) | _BV(12) | _BV(16) | _BV(17) | _BV(18));
//}
//
////************************************
//// Method:    Get_Available_Command
//// FullName:  Get_Available_Command
//// Access:    public
//// Returns:   void
//// Parameter: AGV_State::Command_State::Get_Command_State & state 指令状态
//// Description: 接收并处理指令
////				前提：插入指令不会造成堵塞
////************************************
//void Get_Available_Command(AGV_State::Command_State::Get_Command_State &state)
//{
//	state = AGV_State::Command_State::Get_Command_State::No_Action; //无动作
//	static bool Is_Parsing_Command = false;							//表示是否在处理指令
//	if (My_Serial.Return_rx_flag())									//获取到了新指令
//	{
//		My_Serial.Clear_rx_flag();
//		My_Serial.Clear_rx_cnt();
//
//		int parse_result = Gcode_Inject.parse(My_Serial.Return_RX_buf(), agv_add, command_line + 1); //解析指令，获取解析结果
//		if (parse_result == 0)																		 //解析成功
//		{
//			if (Gcode_Inject.command_letter == 'I') //判断是否为插入指令
//			{
//				//++command_line;	//指令行数+1	//插入指令不增加指令行数
//				Process_Command(&Gcode_Inject, Is_Parsing_Command); //处理指令
//				My_Serial.print("\r\nInject OK");
//			}
//			else //不为插入指令
//			{
//				if (Gcode_Queue.queue_state == Queue_Class::BUFFER_FULL) //指令缓存区满
//				{
//					state = AGV_State::Command_State::Get_Command_State::BUSY;
//				}
//				else
//				{
//					int size = strlen(Gcode_Inject.Return_Command());
//					Gcode_Index_w = Gcode_Buf + Gcode_Queue.ENqueue(); //入队
//					memcpy(Gcode_Index_w->Return_Command(), Gcode_Inject.Return_Command(), size + 1);
//					Gcode_Index_w->command_letter = Gcode_Inject.command_letter;
//					Gcode_Index_w->codenum = Gcode_Inject.codenum;
//					Gcode_Index_w->Parse_State = Gcode_Class::NO_PARSE; //当前指令未执行
//					state = AGV_State::Command_State::Get_Command_State::OK;
//					++command_line; //指令行数+1
//				}
//			}
//		}
//		else if (parse_result > 0)
//		{
//			state = AGV_State::Command_State::Get_Command_State::ERROR; //解析指令出错
//		}
//	}
//	if (!Is_Parsing_Command) //当前没有在处理指令
//	{
//		if (Gcode_Queue.queue_state != Queue_Class::BUFFER_EMPTY) //缓存区不为空
//		{
//			Gcode_Index_r = Gcode_Buf + Gcode_Queue.DEqueue();  //获取队头
//			Process_Command(Gcode_Index_r, Is_Parsing_Command); //处理指令
//			current_existing_task = true;
//		}
//		else//缓存区空
//		{
//			Gcode_Queue.Init();	//缓存区空，没有在处理指令，初始化
//			current_existing_task = false;
//		}
//	}
//	else
//	{
//		Process_Command(Gcode_Index_r, Is_Parsing_Command); //处理指令
//		current_existing_task = true;
//	}
//}
//
////************************************
//// Method:    Process_Command
//// FullName:  Process_Command
//// Access:    public
//// Returns:   void
//// Parameter: Gcode_Class * command
//// Parameter: bool & IS_Parsing 指示当前是否在处理指令
//// Description: 执行相应指令函数
////************************************
//void Process_Command(Gcode_Class *command, bool &IS_Parsing)
//{
//	int codenum = command->codenum;
//	switch (command->command_letter)
//	{
//	case 'G':
//		switch (codenum)
//		{
//		case 0:
//			//Gcode_G0(command, AGV_Current_Position_InWorld.Coordinate, AGV_Target_Position_InAGV);
//			//break;
//		case 1:
//			Gcode_G1(command, AGV_Current_Position_InWorld.Coordinate, AGV_Target_Position_InAGV.Velocity, AGV_Target_Position_InWorld.Coordinate);
//			if (command->Parse_State == Gcode_Class::IS_PARSED)
//			{
//				AGV_Current_Position_InWorld_By_Encoder.Coordinate = Position_Class::Truncation_Coor(AGV_Current_Position_InWorld_By_Encoder.Coordinate);
//				AGV_Current_Position_InWorld.Coordinate = Position_Class::Truncation_Coor(AGV_Current_Position_InWorld.Coordinate); //圆整坐标
//			}
//			break;
//		case 90:
//			Gcode_G90();
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		case 91:
//			Gcode_G91();
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		case 92:
//			AGV_Current_Position_InWorld.Coordinate = Gcode_G92(command, AGV_Current_Position_InWorld.Coordinate);
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		default:
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		}
//		IS_Parsing = (command->Parse_State == Gcode_Class::IS_PARSED) ? false : true; //更新指令处理状态
//		break;
//	case 'M':
//		switch (codenum)
//		{
//		case 17:
//			Gcode_M17();
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		case 18:
//			Gcode_M18();
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		default:
//			command->Parse_State = Gcode_Class::IS_PARSED;
//			break;
//		}
//		IS_Parsing = (command->Parse_State == Gcode_Class::IS_PARSED) ? false : true;										//更新指令处理状态
//		//AGV_Current_Position_InWorld.Coordinate = Position_Class::Truncation_Coor(AGV_Current_Position_InWorld.Coordinate); //对当前坐标保留1位小数
//		break;
//	case 'I':
//		switch (codenum)
//		{
//		case 0:
//			Gcode_I0();
//			break;
//		case 30:
//			Gcode_I30();
//			break;
//		case 114:
//			Gcode_I114();
//			break;
//		case 115:
//			Gcode_I115();
//			break;
//		case 116:
//			Gcode_I116();
//			break;
//		default:
//			break;
//		}
//		command->Parse_State = Gcode_Class::IS_PARSED;
//		break;
//	default:
//		command->Parse_State = Gcode_Class::IS_PARSED; //无法识别的指令
//		break;
//	}
//}
//
////************************************
//// Method:    Update_Print_MSG
//// FullName:  Update_Print_MSG
//// Access:    public
//// Returns:   void
//// Parameter: void
//// Description:	更新小车状态，打印信息
////************************************
//void Update_Print_MSG(void)
//{
//	//delay_ms(20);
//	//My_Serial.print("\r\nv:");
//	//My_Serial.print(AGV_Current_Position_InWorld.Velocity.y_velocity);
//	//My_Serial.print("  y:");
//	//My_Serial.print(AGV_Current_Position_InWorld.Coordinate.y_coor);
//	switch (command_buf_state)
//	{
//	case AGV_State::Command_State::Get_Command_State::BUSY:
//		My_Serial.print("\r\nBusy"); //状态繁忙
//		break;
//	case AGV_State::Command_State::Get_Command_State::OK:
//		My_Serial.print("\r\nOK"); //状态正常
//		My_Serial.print("  Next Line:");
//		My_Serial.print(command_line + 1);
//		break;
//	case AGV_State::Command_State::Get_Command_State::ERROR:
//		My_Serial.print("\r\nCommand Error:");
//		My_Serial.print(My_Serial.Return_RX_buf());
//		My_Serial.print("  Next Line:N");
//		My_Serial.print(command_line + 1); //指令错误
//		break;
//	default:
//		break;
//	}
//	My_Serial.flush();
//}
//
//void Update_Position_InWorld(Position_Class &Position_By_Encoder)
//{
//	if (update_coor_by_G92)
//	{
//		update_coor_by_G92 = false;
//		Position_By_Encoder = AGV_Current_Position_InWorld;
//	}
//	else
//	{
//		AGV_Current_Position_InWorld = Position_By_Encoder;
//	}
//
//}
//
//void Update_Coor_InWorld(Position_Class::Coordinate_Class & Coor_By_Encoder, Position_Class::Coordinate_Class & Coor_By_PGV)
//{
//	if (update_coor_by_PGV)
//	{
//		AGV_Current_Position_InWorld.Coordinate = Coor_By_PGV;
//
//		Coor_By_Encoder = Coor_By_PGV;
//	}
//	else if (update_coor_by_G92)	//由G92指令更新
//	{
//		update_coor_by_G92 = false;
//		Coor_By_Encoder = AGV_Current_Position_InWorld.Coordinate;
//	}
//	else   //根据编码器更新全局坐标
//	{
//		AGV_Current_Position_InWorld.Coordinate = Coor_By_Encoder;
//	}
//
//	if (!current_existing_task)
//	{
//		AGV_Target_Position_InWorld.Coordinate = AGV_Current_Position_InWorld.Coordinate;
//	}
//}
//
////************************************
//// Method:    Get_Command_Coor
//// FullName:  Get_Command_Coor
//// Access:    public
//// Returns:   Position_Class::Coordinate_Class &
//// Parameter: Gcode_Class * command
//// Parameter: const Position_Class::Coordinate_StructTypedef & Current_Coor_InWorld	当前坐标
//// Parameter: Position_Class::Coordinate_StructTypedef & Target_Coor_InWorld 期望坐标
//// Description: 获取指令中给定的坐标，未指定参数则设置为当前值
////************************************
//Position_Class::Coordinate_Class &Get_Command_Coor(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class::Coordinate_Class &Target_Coor_InWorld, bool Is_Absolute_Coor)
//{
//	char *add = 0;
//	const char *command_add = command->Return_Command();
//
//	int k = Is_Absolute_Coor ? 1 : 0;
//
//	Position_Class::Coordinate_Class Coor_temp;
//
//	add = strchr(command_add, 'X');
//	Coor_temp.x_coor = add ? (atof(add + 1)) : k * Current_Coor_InWorld.x_coor; //获取x轴坐标
//	add = strchr(command_add, 'Y');
//	Coor_temp.y_coor = add ? (atof(add + 1)) : k * Current_Coor_InWorld.y_coor; //获取y轴坐标
//	add = strchr(command_add, 'C');
//	Coor_temp.angle_coor = add ? (atof(add + 1)) : k * Current_Coor_InWorld.angle_coor; //获取angle轴坐标
//
//	//if (Coor_temp.angle_coor-Current_Coor_InWorld.angle_coor>180.0f)
//	//{
//	//	Coor_temp.angle_coor -= 360.0f;
//	//}
//	if (!Is_Absolute_Coor) //当前是相对坐标
//	{
//		Target_Coor_InWorld = Position_Class::Relative_To_Absolute(Target_Coor_InWorld, Coor_temp, Current_Coor_InWorld); //坐标变换
//	}
//	else
//	{
//		if (Coor_temp.angle_coor - Current_Coor_InWorld.angle_coor > 180.0f)
//		{
//			Coor_temp.angle_coor -= 360.0f;
//		}
//		else if (Coor_temp.angle_coor - Current_Coor_InWorld.angle_coor < -180.0f)
//		{
//			Coor_temp.angle_coor += 360.0f;
//		}
//		Target_Coor_InWorld = Coor_temp;
//	}
//
//	//Target_Coor_InWorld = Position_Class::Truncation_Coor(Target_Coor_InWorld); //圆整
//	return Target_Coor_InWorld;
//}
//
////同G1
//inline void Gcode_G0(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class::Velocity_Class &Target_Velocity_InAGV, Position_Class::Coordinate_Class &Target_Coor_InWorld)
//{
//	Gcode_G1(command, Current_Coor_InWorld, Target_Velocity_InAGV, Target_Coor_InWorld);
//}
//
////************************************
//// Method:    Gcode_G1
//// FullName:  Gcode_G1
//// Access:    public 
//// Returns:   void
//// Parameter: Gcode_Class * command 指令
//// Parameter: const Position_Class::Coordinate_Class & Current_Coor_InWorld 当前坐标(世界坐标系)
//// Parameter: Position_Class::Velocity_Class & Target_Velocity_InAGV 期望速度(AGV坐标系)
//// Parameter: Position_Class::Coordinate_Class & Target_Coor_InWorld 期望坐标(世界坐标系)
//// Description: 直线插补目标点，获取下一时刻的期望坐标(世界坐标系),期望速度(AGV坐标系)
////************************************
//void Gcode_G1(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class::Velocity_Class &Target_Velocity_InAGV, Position_Class::Coordinate_Class &Target_Coor_InWorld)
//{
//	static Position_Class::Coordinate_Class Origin_Coor_InWorld, Destination_Coor_InWorld;	//世界坐标系下的起点坐标，终点坐标
//	static Position_Class::Coordinate_Class Current_Coor_InOrigin, Destination_Coor_InOrigin; //起点坐标系下的当前坐标，终点坐标
//	static bool Is_Interpolation_Angle = false;												  //指示当前是否在插补角度，处理顺序为，先插补x，y，再插补角度，角度也插补完成，表示G1指令处理完成
//	static bool Is_X_Coor = true;															  //指示当前插补的是x轴
//	float current_coor = 0.0f;																  //指示当前坐标对应的理想AGV在轨迹上运动的距离（位移或角度）
//	float target_coor = 0.0f;
//	MyMath::Coor coor_temp1, coor_temp2; //过点1，和路径的垂直点2
//
//	float velocity_temp = 0.0f;
//	bool interpolation_result;	//插补结果
//	Position_Class::Coordinate_Class Target_Coor_InOrigin;	//起点坐标系中的目标坐标
//
//	//static int singal_Destination_Coor_InOrigin = 1; //终点坐标在起点坐标中的符号位，若待插补距离<0，则为-1，否则为1
//
//	switch (command->Parse_State)
//	{
//	case Gcode_Class::NO_PARSE: //接收到指令，对插补做准备工作
//
//		Interpolation::Actual_INPUT_TypedefStructure Para_Input; //用于插补的输入参数
//
//		command->Parse_State = Gcode_Class::IS_PARSING; //切换执行状态至正在执行
//
//		if (!Is_Interpolation_Angle) //表示插补x,y轴
//		{
//			Origin_Coor_InWorld = Current_Coor_InWorld; //保存起点坐标
//
//			Destination_Coor_InWorld = Get_Command_Coor(command, Current_Coor_InWorld, Destination_Coor_InWorld, Is_Absolute_Coor);						//获取终点坐标
//			Destination_Coor_InOrigin = Position_Class::Absolute_To_Relative(Destination_Coor_InWorld, Destination_Coor_InOrigin, Origin_Coor_InWorld); //获取终点坐标系在起点坐标系中的坐标
//
//			Para_Input.acceleration_abs = AGV_MAX_LINE_ACCELERATION_ACCELERATION / (1000.0f * 1000.0f); //单位转换
//			float abs_x = ABS(Destination_Coor_InOrigin.x_coor);
//			float abs_y = ABS(Destination_Coor_InOrigin.y_coor);
//
//			float abs_temp = 0;
//			if (abs_x > abs_y) //对x轴插补
//			{
//				Is_X_Coor = true;
//				abs_temp = abs_x;
//				Para_Input.displacement = Destination_Coor_InOrigin.x_coor;
//			}
//			else //对y轴插补
//			{
//				Is_X_Coor = false;
//
//				abs_temp = abs_y;
//				Para_Input.displacement = Destination_Coor_InOrigin.y_coor;
//			}
//
//			if (ABS(Para_Input.displacement) < 5.0f)
//			{
//				Target_Velocity_InAGV.x_velocity = 0.0f;
//				Target_Velocity_InAGV.y_velocity = 0.0f;
//				Target_Velocity_InAGV.angle_velocity = 0.0f;
//				Target_Coor_InWorld = Current_Coor_InWorld;
//				Is_Interpolation_Angle = true; //对角度进行插补
//				command->Parse_State = Gcode_Class::NO_PARSE;
//				break;
//			}
//
//			Para_Input.max_velocity_abs = AGV_MAX_LINE_VELOCITY * abs_temp / (abs_x + abs_y) / 1000.0f;
//			Para_Input.min_velocity_abs = AGV_MIN_LINE_VELOCITY * abs_temp / (abs_x + abs_y) / 1000.0f;
//			Para_Input.slow_distance_abs = LINE_SLOWEST_DISTANCE;
//		}
//		else //插补角度
//		{
//			Origin_Coor_InWorld = Current_Coor_InWorld; //保存起点坐标
//			Destination_Coor_InOrigin = Position_Class::Absolute_To_Relative(Destination_Coor_InWorld, Destination_Coor_InOrigin, Origin_Coor_InWorld); //获取终点坐标系在起点坐标系中的坐标
//			float angle_delta = Destination_Coor_InOrigin.angle_coor;
//			Origin_Coor_InWorld = Current_Coor_InWorld; //保存起点坐标
//			Para_Input.displacement = angle_delta;
//
//			if (ABS(Para_Input.displacement) < 2.0f)
//			{
//				Target_Velocity_InAGV.x_velocity = 0.0f;
//				Target_Velocity_InAGV.y_velocity = 0.0f;
//				Target_Velocity_InAGV.angle_velocity = 0.0f;
//				Target_Coor_InWorld = Current_Coor_InWorld;
//				Is_Interpolation_Angle = false;				   //对x,y轴进行插补
//				command->Parse_State = Gcode_Class::IS_PARSED; //执行完毕
//				break;
//			}
//
//			Para_Input.acceleration_abs = AGV_MAX_LINE_ACCELERATION_ACCELERATION / (1000.0f * 1000.0f); //单位转换
//			Para_Input.max_velocity_abs = AGV_MAX_ANGULAR_VELOCITY / 1000.0f;
//			Para_Input.min_velocity_abs = AGV_MIN_ANGULAR_VELOCITY / 1000.0f;
//
//			Para_Input.slow_distance_abs = ANGULAR_SLOWEST_DISTANCE;
//
//			////下列参数和车型有关，以下为麦克纳姆轮四轮车的最大最小角速度、角加速度
//			//Para_Input.acceleration_abs = 2 * WHEEL_MAX_ANGULAR_ACCELERATION / (1000.0f * 1000.0f)*WHEEL_DIAMETER/(DISTANCE_OF_WHEEL_X_AXES+ DISTANCE_OF_WHEEL_Y_AXES);
//			////Para_Input.displacement = ABS(Destination_Coor_InOrigin.angle_coor);
//			//Para_Input.max_velocity_abs = WHEEL_MAX_ANGULAR_VELOCITY / 1000.0f;
//			//Para_Input.min_velocity_abs = WHEEL_MIN_LINE_VELOCITY*abs_x / (abs_x + abs_y) / 1000.0f;
//			//Para_Input.min_velocity_abs = 0;
//		}
//		//根据起点、终点坐标插补速度
//		Interpolation::Init(Para_Input);
//
//		//插补工作完成，直接进入插补
//		//break;
//	case Gcode_Class::IS_PARSING: //对插补的准备工作已完成，正在插补
//		//获取当前坐标在起点坐标系上的坐标
//		Current_Coor_InOrigin = Position_Class::Absolute_To_Relative(Current_Coor_InWorld, Current_Coor_InOrigin, Origin_Coor_InWorld);
//
//		//获取在轨迹上的位移
//		if (!Is_Interpolation_Angle) //在对x,y轴进行插补
//		{
//			coor_temp1.x = Current_Coor_InOrigin.x_coor;
//			coor_temp1.y = Current_Coor_InOrigin.y_coor;
//			//获取斜率不存在的交点
//			if (ABS(Destination_Coor_InOrigin.x_coor) < FLOAT_DELTA)
//			{
//				coor_temp2.y = coor_temp1.y;
//				coor_temp2.x = Destination_Coor_InOrigin.x_coor;
//			}
//			else
//			{
//				MyMath::Get_Vertical_Line_Crossover_Point(Destination_Coor_InOrigin.y_coor / Destination_Coor_InOrigin.x_coor, coor_temp1, coor_temp2);
//			}
//			//获取在插补路径上移动的距离
//			current_coor = Is_X_Coor ? coor_temp2.x : coor_temp2.y;
//		}
//		else //对角度进行插补
//		{
//			current_coor = Current_Coor_InOrigin.angle_coor; //获取在角度上移动的距离
//		}
//
//		target_coor = current_coor;
//
//		interpolation_result = Interpolation::Get_Expectation(velocity_temp, current_coor, target_coor);	//插补结果
//		//Target_Coor_InOrigin.x_coor = target_coor;
//		//Target_Coor_InOrigin.y_coor = target_coor;
//		//Target_Coor_InOrigin.angle_coor = target_coor;
//
//		//Target_Coor_InWorld = Position_Class::Relative_To_Absolute(Target_Coor_InWorld, Target_Coor_InOrigin, Origin_Coor_InWorld);
//
//
//		if (!Is_Interpolation_Angle) //在对x,y轴进行插补
//		{
//			if (Is_X_Coor)	//对x轴插补
//			{
//				Target_Coor_InOrigin.x_coor = target_coor;
//				Target_Coor_InOrigin.y_coor = coor_temp2.y;
//			}
//			else
//			{
//				Target_Coor_InOrigin.x_coor = coor_temp2.x;
//				Target_Coor_InOrigin.y_coor = target_coor;
//			}
//		}
//		else //对角度进行插补
//		{
//			Target_Coor_InOrigin.angle_coor = target_coor;
//		}
//		Target_Coor_InWorld = Position_Class::Relative_To_Absolute(Target_Coor_InWorld, Target_Coor_InOrigin, Origin_Coor_InWorld);
//
//
//		//获取插补速度
//		if (!Is_Interpolation_Angle) //在对x,y轴进行插补
//		{
//			//float velocity_temp = 0.0f;
//			//if (Interpolation::Get_Expectation(velocity_temp, current_coor, target_coor))
//			if (interpolation_result)
//			{
//
//				if (Is_X_Coor)
//				{
//					//if (Destination_Coor_InOrigin.x_coor < 0.0f)
//					//{
//					//	velocity_temp = -velocity_temp;
//					//}
//					Target_Velocity_InAGV.x_velocity = velocity_temp * 1000.0f; //计算目标x轴速度
//					//Target_Coor_InWorld.x_coor = target_coor;				 //更新目标点
//					//if (ABS(Destination_Coor_InOrigin.x_coor) < FLOAT_DELTA)
//					//{
//					//	Target_Position_InAGV.Velocity.y_velocity = 0.0f;
//					//	Target_Position_InAGV.Coordinate.y_coor = 0.0f;
//					//}
//					//else
//					{
//						Target_Velocity_InAGV.y_velocity = Destination_Coor_InOrigin.y_coor / Destination_Coor_InOrigin.x_coor * Target_Velocity_InAGV.x_velocity;
//						//类似该语句存在问题
//						//Target_Coor_InWorld.y_coor = coor_temp2.y;
//					}
//				}
//				else
//				{
//					//if (Destination_Coor_InOrigin.y_coor < 0.0f)
//					//{
//					//	velocity_temp = -velocity_temp;
//					//}
//					Target_Velocity_InAGV.y_velocity = velocity_temp * 1000.0f;
//					//类似该处地方错误，target_coor为相对坐标，而左值为绝对坐标
//					//target_coor有错误，需纠正
//					//Target_Coor_InWorld.y_coor = target_coor; //更新目标点
//					//if (ABS(Destination_Coor_InOrigin.y_coor) < FLOAT_DELTA)
//					//{
//					//	Target_Position_InAGV.Velocity.x_velocity = 0.0f;
//					//	Target_Position_InAGV.Coordinate.x_coor = 0.0f;
//					//}
//					//else
//					{
//						Target_Velocity_InAGV.x_velocity = Destination_Coor_InOrigin.x_coor / Destination_Coor_InOrigin.y_coor * Target_Velocity_InAGV.y_velocity;
//						//Target_Coor_InWorld.x_coor = coor_temp2.x;
//					}
//				}
//				Target_Velocity_InAGV.angle_velocity = 0.0f;
//				Target_Coor_InWorld.angle_coor = Origin_Coor_InWorld.angle_coor;
//			}
//			else //表示当前插补完成
//			{
//				Target_Velocity_InAGV.x_velocity = 0.0f;
//				Target_Velocity_InAGV.y_velocity = 0.0f;
//				Target_Velocity_InAGV.angle_velocity = 0.0f;
//				Target_Coor_InWorld = Current_Coor_InWorld;
//				//Target_Coor_InWorld.x_coor = Current_Coor_InWorld.x_coor;
//				//Target_Coor_InWorld.y_coor = Current_Coor_InWorld.y_coor;
//				//Target_Coor_InWorld.angle_coor = Current_Coor_InWorld.angle_coor;
//				Is_Interpolation_Angle = true; //对角度进行插补
//				command->Parse_State = Gcode_Class::NO_PARSE;
//			}
//		}
//		else //对角度插补
//		{
//			//float velocity_temp = 0.0f;
//			//if (Interpolation::Get_Expectation(velocity_temp, current_coor, target_coor))
//			if (interpolation_result)
//			{
//				//if (Destination_Coor_InOrigin.angle_coor < 0.0f)
//				//{
//				//	velocity_temp = -velocity_temp;
//				//}
//				Target_Velocity_InAGV.x_velocity = 0.0f;
//				Target_Velocity_InAGV.y_velocity = 0.0f;
//				Target_Velocity_InAGV.angle_velocity = velocity_temp * 1000.0f;
//				Target_Coor_InWorld.x_coor = Destination_Coor_InWorld.x_coor;
//				Target_Coor_InWorld.y_coor = Destination_Coor_InWorld.y_coor;
//				//Target_Coor_InWorld.angle_coor = target_coor;
//			}
//			else
//			{
//				Target_Velocity_InAGV.x_velocity = 0.0f;
//				Target_Velocity_InAGV.y_velocity = 0.0f;
//				Target_Velocity_InAGV.angle_velocity = 0.0f;
//				Target_Coor_InWorld = Current_Coor_InWorld;
//				//Target_Coor_InWorld.x_coor = Current_Coor_InWorld.x_coor;
//				//Target_Coor_InWorld.y_coor = Current_Coor_InWorld.y_coor;
//				//Target_Coor_InWorld.angle_coor = Current_Coor_InWorld.angle_coor;
//				Is_Interpolation_Angle = false;				   //对x,y轴进行插补
//				command->Parse_State = Gcode_Class::IS_PARSED; //执行完毕
//			}
//
//		}
//		break;
//	case Gcode_Class::IS_PARSED: //插补完成
//
//		break;
//	default:
//		break;
//	}
//}
//
//void Gcode_G90(void)
//{
//	Is_Absolute_Coor = true;
//}
//
//void Gcode_G91(void)
//{
//	Is_Absolute_Coor = false;
//}
//
//Position_Class::Coordinate_Class & Gcode_G92(Gcode_Class * command, Position_Class::Coordinate_Class & Current_Coor_InWorld)
//{
//	Position_Class::Coordinate_Class Coor_Temp = Current_Coor_InWorld;
//	Current_Coor_InWorld = Get_Command_Coor(command, Current_Coor_InWorld, Coor_Temp);
//	Current_Coor_InWorld = Position_Class::Truncation_Coor(Current_Coor_InWorld); //圆整
//	update_coor_by_G92 = true;
//	current_existing_task = false;
//	return Current_Coor_InWorld;
//}
//
////启动所有电机
//void Gcode_M17(void)
//{
//	Mecanum_AGV.Brake(false);
//}
//
////禁用所有电机
//void Gcode_M18(void)
//{
//	Mecanum_AGV.Brake(true);
//}
//
////紧急停止
//void Gcode_I0(void)
//{
//	Mecanum_AGV.Brake(true);
//	Gcode_Queue.Init();
//}
//
//void Gcode_I30(void)
//{
//	Gcode_Queue.Init();	//缓存区空，没有在处理指令，初始化
//	current_existing_task = false;
//}
//
////返回AGV在世界坐标系中的坐标
//void Gcode_I114(void)
//{
//	My_Serial.print("\r\nx:");
//	My_Serial.print(AGV_Current_Position_InWorld.Coordinate.x_coor);
//	My_Serial.print("  y:");
//	My_Serial.print(AGV_Current_Position_InWorld.Coordinate.y_coor);
//	My_Serial.print("  angle:");
//	My_Serial.print(AGV_Current_Position_InWorld.Coordinate.angle_coor);
//
//	//My_Serial.print("  v_x:");
//	//My_Serial.print(AGV_Target_Position_InAGV.Velocity.x_velocity);
//	//My_Serial.print("  v_y:");
//	//My_Serial.print(AGV_Target_Position_InAGV.Velocity.y_velocity);
//	//My_Serial.print("  w:");
//	//My_Serial.print(AGV_Target_Position_InAGV.Velocity.angle_velocity);
//	//My_Serial.print("\r\n");
//}
//
//void Gcode_I115(void)
//{
//	My_Serial.print("\r\nEncoder---x:");
//	My_Serial.print(AGV_Current_Position_InWorld_By_Encoder.Coordinate.x_coor);
//	My_Serial.print("  y:");
//	My_Serial.print(AGV_Current_Position_InWorld_By_Encoder.Coordinate.y_coor);
//	My_Serial.print("  angle:");
//	My_Serial.print(AGV_Current_Position_InWorld_By_Encoder.Coordinate.angle_coor);
//}
//
//void Gcode_I116(void)
//{
//	My_Serial.print("\r\nPGV---x:");
//	My_Serial.print(AGV_Current_Position_InWorld_By_PGV.Coordinate.x_coor);
//	My_Serial.print("  y:");
//	My_Serial.print(AGV_Current_Position_InWorld_By_PGV.Coordinate.y_coor);
//	My_Serial.print("  angle:");
//	My_Serial.print(AGV_Current_Position_InWorld_By_PGV.Coordinate.angle_coor);
//}
