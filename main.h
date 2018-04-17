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
* TIM11 用于看门狗喂狗（50ms）
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
			OK,		   //指令接收正常
			ERROR	  //指令错误
		} Get_Command_State;
	}
}

Mecanum_Wheel_Class Mecanum_AGV;							   //麦克纳姆轮
IO_Class Led = IO_Class(GPIOA, GPIO_Pin_15);				   //LED指示灯
C50XB_Class My_Serial;										   //无线串口
Queue_Class Gcode_Queue = Queue_Class(4);					   //存放Gcode指令用的队列
AGV_State::Command_State::Get_Command_State command_buf_state; //指令缓存区的状态
Gcode_Class Gcode_Buf[4], Gcode_Inject;						   //指令暂存区，插入指令暂存区
Gcode_Class *Gcode_Index_w = 0, *Gcode_Index_r = 0;			   //指令暂存区读写下标

Position_Class AGV_Target_Position_InWorld, AGV_Current_Position_InWorld;	//AGV在世界坐标系下的目标位姿和速度，在AGV坐标系下的当前位姿和速度
Position_Class AGV_Target_Position_InAGV, AGV_Current_Position_InAGV;	//AGV在AGV坐标系下的目标位姿和速度,在AGV坐标系下的当前位姿和速度
Position_Class AGV_Current_Position_InWorld_By_Encoder;	//世界坐标系下AGV的坐标和位置
//Position_Class &AGV_Current_Encoder_Position_InWorld = Mecanum_AGV.Position_InWorld_By_Encoder; //世界坐标系下由编码器获取的AGV的当前速度和坐标

void Init_System(void);
void Init_System_RCC(void);														//初始化系统所需时钟
void Get_Available_Command(AGV_State::Command_State::Get_Command_State &state); //获取指令
void Process_Command(Gcode_Class *command, bool &IS_Parsing);					//处理指令
void Update_Print_MSG(void);													//更新状态，打印信息
void Update_Position_InWorld(Position_Class&Position_By_Encoder);	//由编码器获取的坐标和速度更新全局坐标和速度，待更新

Position_Class::Coordinate_Class & Get_Command_Coor(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class::Coordinate_Class &Target_Coor_InWorld, bool Is_Absolute_Coor = true);

void Gcode_G0(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class &Target_Position_InWorld);
void Gcode_G1(Gcode_Class *command, const Position_Class::Coordinate_Class &Current_Coor_InWorld, Position_Class &Target_Position_InWorld);

void Gcode_G90(void);	//设定为绝对坐标
void Gcode_G91(void);	//设定为相对坐标

void Gcode_M17(void);
void Gcode_M18(void);

void Gcode_I0(void);
void Gcode_I114(void);