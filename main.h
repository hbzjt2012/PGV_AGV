#pragma once
#include "delay.h"
#include "./Driver/Mecanum.h"
#include "./HALayer/IO.h"
#include "./Driver/C50XB.h"
#include "./App/Queue.h"
#include "./App/Gcode.h"
#include <stdlib.h>
#include "macros.h"
#include "./App/interpolation.h"
#include "./Math/MyMath.h"
#include "./App/Position.h"
#include "./Driver/PGV100.h"
#include "./Driver/TL740D.h"
#include "./App/Movement_Mecanum.h"


/*
* TIM1 编码器FL
* TIM2 FR、FL轮子转速
* TIM3 编码器BL
* TIM4 编码器BR
* TIM5 用于避障、按键等的扫描（程序还未实现）
* TIM6 串口4超时检测
* TIM7 串口2超时检测
* TIM8 编码器FR
* TIM9 用于编码器的频率计算
* TIM10 用于执行机构的PWM波
* TIM11 时基，测试用，定时时间10ms
* TIM12 BR、BL轮子转速
* TIM13
* TIM14
*/

/*
*			中断向量						抢占优先级				响应优先级
* TIM1_UP_TIM10_IRQHandler(用于编码器)			2						1
* TIM6_DAC_IRQHandler(用于通信用串口)			2						2
* UART4_IRQHandler(用于通信用串口)				2						3
* UART5_IRQHandler(用于PGV)						2						4
* DMA1_Stream0_IRQHandler(用于PGV)				1						1
* USART2_IRQHandler	(用于陀螺仪)				1						3
* TIM7_IRQHandler(用于陀螺仪)					1						2
*/

namespace AGV_State
{
	namespace Command_State
	{
		typedef enum {
			No_Action, //无动作
			BUSY,	  //缓存区繁忙
			OK,		  //指令接收正常
			ERROR	  //指令错误
		} Get_Command_State;
	}
}

void Init_System(void);
void Init_System_RCC(void);		//初始化系统所需时钟

Coordinate_Class Location(Coordinate_Class Current_Coor);		//对AGV当前坐标定位
void Process_Movement_Command(void);	//处理运动指令
void Movement_Control(void);	//运动控制
void Check_Avoidance_Buton(void);	//检查避障和按键
void Parse_Sensor_Data(void);	//处理传感器数据
void Process_Gcode_Command(AGV_State::Command_State::Get_Command_State &state); //获取并处理命令指令
void Update_Print_MSG(void);		//打印信息

bool Get_Next_Movement_Command(Movement_Class*&command);	//获取下一条可执行的指令
bool Run_Movement_Command(Movement_Class*movement, Coordinate_Class Target_Coor, Velocity_Class Target_Velocity);
void Run_Gcode_Command(Gcode_Class *command, bool &IS_Parsing);

bool Add_Movement_Command(Coordinate_Class Origin, Coordinate_Class Destination);

Coordinate_Class & Get_Command_Coor(Gcode_Class *command, const Coordinate_Class &Current_Coor_InWorld, Coordinate_Class &Target_Coor_InWorld_ByGcode, bool Is_Absolute_Coor = true);	//获取指令中的坐标

void Gcode_G0(Gcode_Class *command, const Coordinate_Class &Current_Coor_InWorld);	//直接插补
void Gcode_G1(Gcode_Class *command, const Coordinate_Class &Current_Coor_InWorld);	//直线插补

void Gcode_G90(void);	//设定为绝对坐标
void Gcode_G91(void);	//设定为相对坐标

void Gcode_M17(void);	//启动所有电机
void Gcode_M18(void);	//禁用所有电机

void Gcode_I0(void);	//急停
void Gcode_I30(void);	//清除指令队列
void Gcode_I114(void);	//获取坐标
//void Gcode_I115(void);	//获取最近一次编码器的坐标
//void Gcode_I116(void);	//获取最近一次PGV的坐标
