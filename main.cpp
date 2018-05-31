#include "main.h"
#include <stdlib.h>
#include <cstring>

Mecanum_Wheel_Class Mecanum_AGV;			//麦克纳姆轮
IO_Class Led = IO_Class(GPIOA, GPIO_Pin_15);//LED指示灯
C50XB_Class My_Serial;						//无线串口
PGV_Class PGV100;							//PGV传感器
TL740D_Class TL740;							//陀螺转角仪

Queue_Class Gcode_Queue = Queue_Class(Gcode_Command_Buf_SIZE);		//存放Gcode指令用的队列
Gcode_Class Gcode_Buf[Gcode_Command_Buf_SIZE], Gcode_Inject;		//指令暂存区，插入指令暂存区
Gcode_Class *Gcode_Index_w = Gcode_Buf, *Gcode_Index_r = Gcode_Buf;	//指令暂存区读写下标
AGV_State::Gcode_Command_State command_buf_state;					//Gcode指令缓存区的状态

Queue_Class Movement_Queue = Queue_Class(Movement_Command_Buf_SIZE);//存放运动指令用的队列
Movement_Mecanum_Class Movement_Buf[Movement_Command_Buf_SIZE];		//运动指令暂存区
Movement_Class *Movement_Index_w = Movement_Buf, *Movement_Index_r = Movement_Buf;	//运动指令暂存区读写下标
AGV_State::Movement_Command_State movement_buf_state;				//指令缓存区的状态

Coordinate_Class Virtual_AGV_Coor_InWorld;	//虚拟的AGV坐标，获取指令坐标时，避免累计误差
Coordinate_Class AGV_Current_Coor_InWorld, AGV_Target_Coor_InWorld;	//AGV在世界坐标系下的当前坐标和目标坐标
Velocity_Class AGV_Current_Velocity_InAGV, AGV_Target_Velocity_InAGV;	//AGV在小车坐标系下的当前速度和目标速度

int &agv_add_code = Parameter_Class::AGV_Address_Code;	//AGV的地址码
bool &Is_Absolute_Coor = Parameter_Class::Is_Absolute_Coor;	//指示当前输入是否为绝对坐标

int gcode_command_line_received = 0;		  //表示当前已经接收到的指令行数
bool time11_flag = false;	//用于时基定时器的定时标志位
unsigned long time11_cnt = 0;

inline void Gcode_Commond_Over(Gcode_Class *gcode_command);

int main(void)
{
	Init_System();//配置系统所需的硬件、外设
	//TL740.Bias_Init();

	Encoder_Class::Clear_Time_US();
	//My_Serial.print(TL740.forward_accel_bias, 3);
	while (1)
	{
		if (time11_flag)	//表示控制周期到
		{
			time11_flag = false;
			Location_AGV();	//AGV定位函数
			Process_Movement_Command();	//获取并处理运动指令
			Movement_Control();	//运动控制
		}
		Check_Avoidance_Buton();	//检查避障和按键动作
		Parse_Sensor_Data();	//解析传感器数据

		Process_Gcode_Command(command_buf_state); //获取处理当前指令(已完成)                                                                                          

		Update_Print_MSG();	//打印信息
	}
}


void Init_System(void)
{
	delay_init();	//设置delay函数
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置系统中断优先级分组 2
	Init_System_RCC();	//初始化系统所需时钟

	Gcode_Queue.Init();
	Movement_Queue.Init();
	Led.Init(GPIO_Mode_OUT);
	My_Serial.Init(115200);
	Mecanum_AGV.Init();

	Parameter_Class::Init_Parameter();	//初始化参数
	Movement_Class::Init_Parameter();	//初始化参数

	My_Serial.enable();	//使能串口
	Gcode_M17();	//启动电机

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	TIM_Base_Class::Init(TIM11, 2000, 840, true);	//设置定时器11的中断频率为100Hz，时基--10ms	
	TIM_Base_Class::Begin(TIM11);

	while (!Gcode_G4(1000));	//延时10s
	Led.Set();
	PGV100.Init(115200);
	TL740.Init(115200);
	while (!Gcode_G4(1000));	//延时10s

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

//使用编码器、陀螺仪、PGV传感器的数据对AGV定位
void Location_AGV(void)
{
	float time_s = Mecanum_AGV.Cal_Velocity_By_Encoder(AGV_Current_Velocity_InAGV) / 1000.0f;	//获取由编码器计算得到的速度，两次运行间隔时间

	if (time_s > FLOAT_DELTA)
	{
		AGV_Current_Coor_InWorld = Mecanum_AGV.Update_Coor_demo(AGV_Current_Coor_InWorld, AGV_Current_Velocity_InAGV, time_s);
		if (TL740.data_OK)	//表示陀螺仪数据已更新
		{
			//	float omega_TL740 = TL740.z_rate;	//陀螺仪测量得到的角速度
			//	float theta_TL740 = TL740.z_heading;	//陀螺仪测量得到的角度
			//	float omega = 0.0;
			//	//获得更新后的角度，角速度
			//	Kalman_Filter::Cal_Theta_Omega(AGV_Current_Velocity_InAGV.angular_velocity * 180 / M_PI, omega_TL740, theta_TL740, omega, time_s, AGV_Current_Coor_InWorld.angle_coor);

			//	AGV_Current_Velocity_InAGV.angular_velocity = omega / 180 * M_PI;	//一次更新后的角速度

			//	Kalman_Filter::Cal_XY_By_Gyro_Encoder(AGV_Current_Coor_InWorld, AGV_Current_Velocity_InAGV, theta_TL740, time_s);	//更新坐标
			//static float velocity;
			//velocity += (TL740.Return_Forward_Accel()*time_s);
			//My_Serial << "\r\n ";
			////My_Serial.print(TL740.forward_accel - TL740.forward_accel_bias, 3);
			//My_Serial << " " << TL740.z_heading << " " << AGV_Current_Coor_InWorld.angle_coor;
			//My_Serial << " " << TL740.z_rate << " " << AGV_Current_Velocity_InAGV.angular_velocity_angle;
			TL740.data_OK = false;

		}
		//else
		//{
		//	//Kalman_Filter::Cal_X_Y_Theta_By_Encoder_Gyro(AGV_Current_Coor_InWorld, AGV_Current_Velocity_InAGV, time_s);
		//}

		if (PGV100.data_OK)	//表示PGV数据已更新
		{
			PGV100.data_OK = false;
			AGV_Current_Coor_InWorld = PGV100.coor;
			//AGV_Current_Coor_InWorld = Kalman_Filter::Cal_Coor_By_PGV(AGV_Current_Coor_InWorld, PGV100.coor);
			//融合PGV数据和测量数据
			//使用PGV数据更新当前坐标

			//TL740.Set_Bias(AGV_Current_Coor_InWorld.angle_coor);	//更新陀螺仪偏置
		}
	}
	AGV_Current_Coor_InWorld.Transform_Angle();
}

//获取并处理运动指令
void Process_Movement_Command(void)
{
	static bool Is_Parsing_Movement = false;	//指示当前是否在执行运动指令

	if (!Is_Parsing_Movement)	//没有在执行运动指令
	{
		if (Movement_Queue.queue_state != Queue_Class::BUFFER_EMPTY) //缓存区不为空
		{
			Movement_Index_r = Movement_Buf + Movement_Queue.DEqueue();
			Is_Parsing_Movement = !Run_Movement_Command(Movement_Index_r, AGV_Current_Coor_InWorld);	//执行运动指令并返回结果
			movement_buf_state = AGV_State::Movement_Command_State::Movement_Command_OK;	//缓存区正常
		}
		else
		{
			Movement_Queue.Init();	//初始化缓存区
			movement_buf_state = AGV_State::Movement_Command_State::Movement_Command_IDLE;
		}
	}
	else
	{
		Is_Parsing_Movement = !Run_Movement_Command(Movement_Index_r, AGV_Current_Coor_InWorld);	//执行运动指令并返回结果
	}
}

//运动控制
void Movement_Control(void)
{
	if (movement_buf_state != AGV_State::Movement_Command_State::Movement_Command_IDLE)	//指令缓存区存在数据
	{
		AGV_Target_Coor_InWorld = Movement_Index_r->Target_Coor_InWorld;
		AGV_Target_Velocity_InAGV = Movement_Index_r->Target_Velocity_InAGV;
	}
	else
	{
		AGV_Target_Coor_InWorld = AGV_Current_Coor_InWorld;
		AGV_Target_Velocity_InAGV *= 0.0f;
	}
	//控制小车
	//Mecanum_AGV.AGV_Control_Class::Write_Velocity(AGV_Current_Coor_InWorld, AGV_Target_Coor_InWorld, AGV_Target_Velocity_InAGV);
	Mecanum_AGV.Write_Velocity(AGV_Target_Velocity_InAGV);
}

//检查避障和按键动作，生成避障信息
void Check_Avoidance_Buton(void)
{
}

//获取处理传感器数据(如陀螺仪，PGV)
void Parse_Sensor_Data(void)
{
	if (PGV100.Return_rx_flag())	//接收到了PGV的数据
	{
		PGV100.Clear_rx_flag();
		if (PGV100.Analyze_Data() && (PGV100.target == PGV_Class::Data_Matrix_Tag))
		{
			PGV100.Cal_Coor();	//处理数据
		}
	}
	if (!(time11_cnt % 5))	//50ms时间到
	{
		PGV100.Send(PGV_Class::Read_PGV_Data);	//读取PGV传感器数据
	}

	if (TL740.Return_rx_flag())
	{
		TL740.Clear_rx_flag();
		if (TL740.Analyze_Data())
		{
			TL740.data_OK = true;	//表示接收到了新的陀螺仪数据
		}
	}

}

//获取并处理Gcode命令指令
void Process_Gcode_Command(AGV_State::Gcode_Command_State & state)
{
	state = AGV_State::Gcode_Command_State::Gcode_Command_IDLE; //无动作
	static bool Is_Parsing_Command = false;							//表示是否在处理指令
	if (My_Serial.Return_rx_flag())									//获取到了新指令
	{
		My_Serial.Clear_rx_flag();
		My_Serial.Clear_rx_cnt();

		//解析指令，获取解析结果
		int parse_result = Gcode_Inject.parse(My_Serial.Return_RX_buf(), agv_add_code, gcode_command_line_received + 1);
		if (parse_result == 0)																		 //解析成功
		{
			if (Gcode_Inject.command_letter == 'I') //判断是否为插入指令
			{
				Is_Parsing_Command = !Run_Gcode_Command(&Gcode_Inject); //处理指令
				My_Serial.print("\r\nInject OK");
			}
			else //不为插入指令
			{
				if (Gcode_Queue.queue_state == Queue_Class::BUFFER_FULL) //指令缓存区满
				{
					state = AGV_State::Gcode_Command_State::Gcode_Command_BUSY;
				}
				else
				{
					int size = strlen(Gcode_Inject.Return_Command());
					Gcode_Index_w = Gcode_Buf + Gcode_Queue.ENqueue(); //入队
					memcpy(Gcode_Index_w->Return_Command(), Gcode_Inject.Return_Command(), size + 1);
					Gcode_Index_w->command_letter = Gcode_Inject.command_letter;
					Gcode_Index_w->codenum = Gcode_Inject.codenum;
					Gcode_Index_w->Parse_State = Gcode_Class::NO_PARSE; //当前指令未执行
					state = AGV_State::Gcode_Command_State::Gcode_Command_OK;
					++gcode_command_line_received; //指令行数+1
				}
			}
		}
		else if (parse_result > 0)
		{
			state = AGV_State::Gcode_Command_State::Gcode_Command_ERROR; //解析指令出错
		}
	}
	if (!Is_Parsing_Command) //当前没有在处理指令
	{
		if (Gcode_Queue.queue_state != Queue_Class::BUFFER_EMPTY) //缓存区不为空
		{
			Gcode_Index_r = Gcode_Buf + Gcode_Queue.DEqueue();  //获取队头
			Is_Parsing_Command = !Run_Gcode_Command(Gcode_Index_r); //处理指令
		}
		else//缓存区空
		{
			Gcode_Queue.Init();	//缓存区空，没有在处理指令，初始化
		}
	}
	else
	{
		Is_Parsing_Command = !Run_Gcode_Command(Gcode_Index_r); //处理指令
	}
}

//打印信息
void Update_Print_MSG(void)
{
	switch (command_buf_state)
	{
	case AGV_State::Gcode_Command_State::Gcode_Command_BUSY:
		My_Serial.print("\r\nBusy"); //状态繁忙
		break;
	case AGV_State::Gcode_Command_State::Gcode_Command_OK:
		My_Serial.print("\r\nOK"); //状态正常
		My_Serial.print("  Next Line:");
		My_Serial.print(gcode_command_line_received + 1);
		break;
	case AGV_State::Gcode_Command_State::Gcode_Command_ERROR:
		My_Serial.print("\r\nCommand Error:");
		My_Serial.print(My_Serial.Return_RX_buf());
		My_Serial.print("  Next Line:N");
		My_Serial.print(gcode_command_line_received + 1); //指令错误
		break;
	default:
		break;
	}
	My_Serial.flush();
}

//************************************
// Method:    Add_Movement_Command
// FullName:  Add_Movement_Command
// Access:    public 
// Returns:   AGV_State::Movement_Command_State
// Parameter: const Coordinate_Class & Destination 终点
// Parameter: Movement_Class * & command 写入的指令缓存区地址
// Parameter: const float threshold 阈值(mm)
// Parameter: const bool Is_X_Y	在xoy平面上插补插补
// Description: 添加运动指令，返回添加结果
//************************************
AGV_State::Movement_Command_State Add_Movement_Command(const Coordinate_Class & Destination, Movement_Class *&command, const float threshold)
{
	AGV_State::Movement_Command_State state = AGV_State::Movement_Command_State::Movement_Command_IDLE;
	if (Movement_Queue.queue_state == Queue_Class::BUFFER_FULL) //指令缓存区满
	{
		state = AGV_State::Movement_Command_State::Movement_Command_BUSY;	//繁忙
	}
	else
	{
		Movement_Index_w = Movement_Buf + Movement_Queue.ENqueue(); //入队
		Movement_Index_w->Set_Destination(Destination, threshold);
		Movement_Index_w->Interpolation_State = Movement_Class::NO_Interpolation;
		command = Movement_Index_w;
		state = AGV_State::Movement_Command_State::Movement_Command_OK;	//正常
	}
	return state;
}

//************************************
// Method:    Run_Movement_Command
// FullName:  Run_Movement_Command
// Access:    public 
// Returns:   bool true表示该运动指令执行完毕
// Parameter: Movement_Class * movement_command
// Parameter: const Coordinate_Class & Current_Coor
// Description: 根据当前坐标获取目标速度和坐标
//************************************
bool Run_Movement_Command(Movement_Class * movement_command, const Coordinate_Class & Current_Coor)
{
	switch (movement_command->Interpolation_State)
	{
	case Movement_Class::NO_Interpolation: //未插补
										   //插补
		if (movement_command->Init(Current_Coor))
		{
			movement_command->Interpolation_State = Movement_Class::IS_Interpolating;
		}
		else//插补失败，即要移动的距离小于阈值
		{
			movement_command->Interpolation_State = Movement_Class::IS_Interpolated;
			return true;
		}
		//break;
	case Movement_Class::IS_Interpolating://正在插补
		if (!(movement_command->Cal_Velocity(Current_Coor)))	//插补完成
		{
			movement_command->Interpolation_State = Movement_Class::IS_Interpolated;	//插补完成
		}
		break;
	case Movement_Class::IS_Interpolated://插补完毕
		AGV_Target_Coor_InWorld = Current_Coor;	//获取目标坐标
		AGV_Target_Velocity_InAGV *= 0.0f;	//清空目标速度
		return true;
		break;
	default:
		break;
	}
	return false;

}

//************************************
// Method:    Run_Gcode_Command
// FullName:  Run_Gcode_Command
// Access:    public 
// Returns:   bool
// Parameter: Gcode_Class * gcode_command
// Description: 执行Gcode指令，返回执行结果，true表示执行完毕
//************************************
bool Run_Gcode_Command(Gcode_Class * gcode_command)
{
	int codenum = gcode_command->codenum;
	switch (gcode_command->command_letter)
	{
	case 'G':
		switch (codenum)
		{
		case 0:
			Gcode_G0(gcode_command, Virtual_AGV_Coor_InWorld);	//先旋转后直线运动到目标点,使用虚拟坐标可以降低累计误差
			break;
		case 1:
			Gcode_G1(gcode_command, Virtual_AGV_Coor_InWorld);	//先直线运动后旋转到目标点,使用虚拟坐标可以降低累计误差
			break;
		case 2:
			Gcode_G2(gcode_command, Virtual_AGV_Coor_InWorld);	//直接运动到目标点
			break;
		case 4:
			Gcode_G4(gcode_command);
			break;
		case 90:
			Gcode_G90();	//设定输入为绝对坐标
			Gcode_Commond_Over(gcode_command);
			break;
		case 91:
			Gcode_G91();
			Gcode_Commond_Over(gcode_command);
			break;
		default:
			Gcode_Commond_Over(gcode_command);
			break;
		}
		break;
	case 'M':
		switch (codenum)
		{
		case 17:
			Gcode_M17();
			Gcode_Commond_Over(gcode_command);
			break;
		case 18:
			Gcode_M18();
			Gcode_Commond_Over(gcode_command);
			break;
		default:
			Gcode_Commond_Over(gcode_command);
			break;
		}
		break;
	case 'I':
		switch (codenum)
		{
		case 0:
			Gcode_I0();
			break;
		case 30:
			Gcode_I30();
			break;
		case 114:
			Gcode_I114();
			break;
		default:
			break;
		}
		Gcode_Commond_Over(gcode_command);
		break;
	default:
		Gcode_Commond_Over(gcode_command);
		break;
	}
	return gcode_command->Parse_State == Gcode_Class::IS_PARSED; //返回执行结果，true表示执行完毕
}

Coordinate_Class Get_Command_Coor(Gcode_Class * command, const Coordinate_Class & Base_Coor_InWorld, bool Is_Absolute_Coor)
{
	char *add = 0;
	const char *command_add = command->Return_Command();

	int k = Is_Absolute_Coor ? 1 : 0;

	Coordinate_Class Coor_temp;

	add = strchr(command_add, 'X');
	Coor_temp.x_coor = add ? (atof(add + 1)) : k * Base_Coor_InWorld.x_coor; //获取x轴坐标
	add = strchr(command_add, 'Y');
	Coor_temp.y_coor = add ? (atof(add + 1)) : k * Base_Coor_InWorld.y_coor; //获取y轴坐标
	add = strchr(command_add, 'C');
	Coor_temp.angle_coor = add ? (atof(add + 1)) : k * Base_Coor_InWorld.angle_coor; //获取angle轴坐标

	if (!Is_Absolute_Coor) //当前是相对坐标
	{
		Coor_temp = Base_Coor_InWorld + Coor_temp;
	}
	//else   //当前为绝对坐标
	//{
	//	if (Coor_temp.angle_coor - Base_Coor_InWorld.angle_coor > 180.0f)
	//	{
	//		Coor_temp.angle_coor -= 360.0f;
	//	}
	//	else if (Coor_temp.angle_coor - Base_Coor_InWorld.angle_coor < -180.0f)
	//	{
	//		Coor_temp.angle_coor += 360.0f;
	//	}
	//}
	Coor_temp.Transform_Angle();

	//Coor_temp.Truncation_Coor();


	return Coor_temp;
}

//指令执行结束
void Gcode_Commond_Over(Gcode_Class *gcode_command)
{
	gcode_command->Parse_State = Gcode_Class::IS_PARSED;
}

extern "C" {
	void TIM1_TRG_COM_TIM11_IRQHandler()
	{
		if (TIM11->SR & TIM_IT_Update) //更新中断
		{
			TIM11->SR = ~TIM_IT_Update;
			time11_flag = true;
			time11_cnt++;
		}
	}
}

//************************************
// Method:    Gcode_G0
// FullName:  Gcode_G0
// Access:    public 
// Returns:   void
// Parameter: Gcode_Class * command
// Parameter: Coordinate_Class & Virtual_Current_Coor_InWorld 虚拟的当前坐标(为了消除累计误差)
// Description: 将指令中的坐标分解成先旋转后直线运动的两个运动指令，加入到运动缓存区
//************************************
void Gcode_G0(Gcode_Class * command, Coordinate_Class & Virtual_Current_Coor_InWorld)
{
	static Movement_Class *movement_command = 0;
	Coordinate_Class Virtual_Mid_Coor;
	switch (command->Parse_State)
	{
	case Gcode_Class::NO_PARSE: //接收到指令，对插补做准备工作
		Virtual_Mid_Coor = Virtual_Current_Coor_InWorld;	//获取中间点的x,y坐标
		Virtual_Current_Coor_InWorld = Get_Command_Coor(command, Virtual_Current_Coor_InWorld, Is_Absolute_Coor);	//获取终点坐标
		Virtual_Mid_Coor.angle_coor = Virtual_Current_Coor_InWorld.angle_coor;	//获取中间点的angle坐标
		Virtual_Mid_Coor.angle_rad = Virtual_Mid_Coor.angle_coor / 180 * M_PI;
		Add_Movement_Command(Virtual_Mid_Coor, movement_command, Parameter_Class::movement_threshold);	//旋转运动
		Add_Movement_Command(Virtual_Current_Coor_InWorld, movement_command, Parameter_Class::movement_threshold);	//直线运动
		command->Parse_State = Gcode_Class::IS_PARSING;
		break;
	case Gcode_Class::IS_PARSING: //对插补的准备工作已完成，正在插补
		if ((movement_command == Movement_Index_r) && (movement_command->Interpolation_State == Movement_Class::IS_Interpolated))	//当前指令插补完成
		{
			command->Parse_State = Gcode_Class::IS_PARSED;
		}
		break;
	case Gcode_Class::IS_PARSED: //插补完成
		break;
	default:
		break;
	}
}

void Gcode_G1(Gcode_Class * command, Coordinate_Class & Virtual_Current_Coor_InWorld)
{
	static Movement_Class *movement_command = 0;
	Coordinate_Class Virtual_Mid_Coor;
	switch (command->Parse_State)
	{
	case Gcode_Class::NO_PARSE: //接收到指令，对插补做准备工作
		Virtual_Mid_Coor = Virtual_Current_Coor_InWorld;	//获取中间点的angle坐标
		Virtual_Current_Coor_InWorld = Get_Command_Coor(command, Virtual_Current_Coor_InWorld, Is_Absolute_Coor);	//获取终点坐标
		Virtual_Mid_Coor.x_coor = Virtual_Current_Coor_InWorld.x_coor;	//获取中间点的x,y坐标
		Virtual_Mid_Coor.y_coor = Virtual_Current_Coor_InWorld.y_coor;
		Add_Movement_Command(Virtual_Mid_Coor, movement_command, Parameter_Class::movement_threshold);	//直线运动
		Add_Movement_Command(Virtual_Current_Coor_InWorld, movement_command, Parameter_Class::movement_threshold);	//旋转运动
		command->Parse_State = Gcode_Class::IS_PARSING;
		break;
	case Gcode_Class::IS_PARSING: //对插补的准备工作已完成，正在插补
		if ((movement_command == Movement_Index_r) && (movement_command->Interpolation_State == Movement_Class::IS_Interpolated))	//当前指令插补完成
		{
			command->Parse_State = Gcode_Class::IS_PARSED;
		}
		break;
	case Gcode_Class::IS_PARSED: //插补完成
		break;
	default:
		break;
	}
}

void Gcode_G2(Gcode_Class * command, Coordinate_Class & Virtual_Current_Coor_InWorld)
{
	static Movement_Class *movement_command = 0;
	Coordinate_Class Virtual_Mid_Coor;
	switch (command->Parse_State)
	{
	case Gcode_Class::NO_PARSE: //接收到指令，对插补做准备工作
		Virtual_Current_Coor_InWorld = Get_Command_Coor(command, Virtual_Current_Coor_InWorld, Is_Absolute_Coor);	//获取终点坐标
		Add_Movement_Command(Virtual_Current_Coor_InWorld, movement_command, Parameter_Class::movement_threshold);	//直线运动
		command->Parse_State = Gcode_Class::IS_PARSING;

		break;
	case Gcode_Class::IS_PARSING: //对插补的准备工作已完成，正在插补
		if ((movement_command == Movement_Index_r) && (movement_command->Interpolation_State == Movement_Class::IS_Interpolated))	//当前指令插补完成
		{
			command->Parse_State = Gcode_Class::IS_PARSED;
		}
		break;
	case Gcode_Class::IS_PARSED: //插补完成
		break;
	default:
		break;
	}
}

//************************************
// Method:    Gcode_G4
// FullName:  Gcode_G4
// Access:    public 
// Returns:   bool true表示延时时间到
// Parameter: unsigned long time_10ms 延时时间，单位10ms
// Description: 延时一段时间
//************************************
bool Gcode_G4(unsigned long time_10ms)
{
	static bool first_run = true;
	static unsigned long time_init = 0;

	if (first_run)
	{
		time_init = time11_cnt;	//获取第一次运行时的时间
		first_run = false;
	}

	if (!first_run)	//不是第一次运行
	{
		if ((time11_cnt - time_init) >= time_10ms)
		{
			first_run = true;
			return true;
		}
		return false;
	}
}

void Gcode_G4(Gcode_Class * command)
{
	char *add = 0;
	const char *command_add = command->Return_Command();

	static unsigned long time_10ms = 0;
	static bool no_time = true;	//true表示还未获取暂停时间
	if (no_time)
	{
		add = strchr(command_add, 'T');
		if (add)	//表示查找到了T
		{
			time_10ms = atol(add + 1);	//获取时间
			no_time = false;
		}
		else
		{
			time_10ms = 0;
		}
	}
	else
	{
		no_time = Gcode_G4(time_10ms);
		if (no_time)
		{
			command->Parse_State == Gcode_Class::IS_PARSED;	//指令完成
		}
	}


}

void Gcode_G90(void)
{
	Is_Absolute_Coor = true;
}

void Gcode_G91(void)
{
	Is_Absolute_Coor = false;
}

void Gcode_M17(void)
{
	Mecanum_AGV.Brake(false);
}

void Gcode_M18(void)
{
	Mecanum_AGV.Brake(true);
}

void Gcode_I0(void)
{
}

void Gcode_I30(void)
{
}

void Gcode_I114(void)
{
	My_Serial.print("\r\nx:");
	My_Serial.print(AGV_Current_Coor_InWorld.x_coor);
	My_Serial.print("  y:");
	My_Serial.print(AGV_Current_Coor_InWorld.y_coor);
	My_Serial.print("  angle:");
	My_Serial.print(AGV_Current_Coor_InWorld.angle_coor);
}


