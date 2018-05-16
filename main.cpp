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

int main(void)
{
	Init_System();//配置系统所需的硬件、外设


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
	PGV100.Init(115200);
	TL740.Init(115200);

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

void Location_AGV(void)
{
}

void Process_Movement_Command(void)
{
}

void Movement_Control(void)
{
}

void Check_Avoidance_Buton(void)
{
}

void Parse_Sensor_Data(void)
{
}

void Process_Gcode_Command(AGV_State::Gcode_Command_State & state)
{
}

void Update_Print_MSG(void)
{
}

bool Get_Next_Movement_Command(Movement_Class *& movement_command)
{
	return false;
}

AGV_State::Movement_Command_State Add_Movement_Command(const Coordinate_Class & Destination, const float threshold, const bool Is_Linear)
{
	return AGV_State::Movement_Command_State();
}

bool Run_Movement_Command(Movement_Class * movement_command, const Coordinate_Class & Current_Coor)
{
	return false;
}

bool Run_Gcode_Command(Gcode_Class * gcode_command)
{
	return false;
}

Coordinate_Class Get_Command_Coor(Gcode_Class * command, const Coordinate_Class & Base_Coor_InWorld, bool Is_Absolute_Coor)
{
	return Coordinate_Class();
}

extern "C" {
	void TIM1_TRG_COM_TIM11_IRQHandler()
	{
		if (TIM11->SR & TIM_IT_Update) //更新中断
		{
			TIM11->SR = ~TIM_IT_Update;
			time11_flag = true;
		}
	}
}

void Gcode_G0(Gcode_Class * command, const Coordinate_Class & Current_Coor_InWorld)
{
}

void Gcode_G1(Gcode_Class * command, const Coordinate_Class & Current_Coor_InWorld)
{
}

void Gcode_G04(Gcode_Class * command)
{
}

void Gcode_G90(void)
{
}

void Gcode_G91(void)
{
}

void Gcode_M17(void)
{
}

void Gcode_M18(void)
{
}

void Gcode_I0(void)
{
}

void Gcode_I30(void)
{
}

void Gcode_I114(void)
{
}


