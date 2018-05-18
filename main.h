#pragma once
#include "delay.h"
#include "./Driver/Mecanum.h"
#include "./HALayer/IO.h"
#include "./Driver/C50XB.h"
#include "./App/Queue.h"
#include "./App/Gcode.h"
#include "macros.h"
#include "./App/Position.h"
#include "./Driver/PGV100.h"
#include "./Driver/TL740D.h"
#include "./App/Movement_Mecanum.h"
#include "parameter_define.h"
#include "./App/Kalman_filter.h"


/*
* TIM1 编码器FL
* TIM2 FR、FL轮子转速
* TIM3 编码器BL
* TIM4 编码器BR
* TIM5 用于避障、按键等的扫描（未实现）
* TIM6 串口4超时检测
* TIM7 串口2超时检测
* TIM8 编码器FR
* TIM9 用于编码器的频率计算
* TIM10 用于输出执行机构的PWM波
* TIM11 时基，定时时间10ms
* TIM12 BR、BL轮子转速
* TIM13
* TIM14
*/

/*
*			中断向量						抢占优先级				响应优先级
* TIM1_TRG_COM_TIM11_IRQn(用于时基)				3						4
* TIM6_DAC_IRQHandler(用于通信用串口)			2						2
* UART4_IRQHandler(用于通信用串口)				2						3
* UART5_IRQHandler(用于PGV)						2						4
* DMA1_Stream0_IRQHandler(用于PGV)				1						1
* USART2_IRQHandler	(用于陀螺仪)				1						3
* TIM7_IRQHandler(用于陀螺仪)					1						2
*/


void Init_System(void);	//配置系统所需的硬件、外设
void Init_System_RCC(void);		//初始化系统所需时钟

void Location_AGV(void);		//AGV定位函数
void Process_Movement_Command(void);	//获取并处理运动指令
void Movement_Control(void);	//运动控制
void Check_Avoidance_Buton(void);	//检查避障和按键动作
void Parse_Sensor_Data(void);	//处理传感器数据
void Process_Gcode_Command(AGV_State::Gcode_Command_State &state); //获取并处理Gcode命令指令
void Update_Print_MSG(void);		//打印信息

//添加运动指令，返回添加结果
//缓存区满，未做相应处理
AGV_State::Movement_Command_State Add_Movement_Command(const Coordinate_Class &Destination, Movement_Class *&command, const float threshold, const bool Is_X_Y);

//根据当前坐标获取目标速度和坐标，返回执行结果（true表示执行完毕）
bool Run_Movement_Command(Movement_Class*movement_command, const Coordinate_Class &Current_Coor);
bool Run_Gcode_Command(Gcode_Class *gcode_command);	//执行Gcode指令，返回执行结果，true表示执行完毕


//获取指令中的坐标
Coordinate_Class Get_Command_Coor(Gcode_Class *command, const Coordinate_Class &Base_Coor_InWorld, bool Is_Absolute_Coor = true);

void Gcode_G0(Gcode_Class *command, Coordinate_Class &Virtual_Current_Coor_InWorld);	//先旋转后直线运动到目标点
void Gcode_G1(Gcode_Class *command, Coordinate_Class &Virtual_Current_Coor_InWorld);	//先直线运动后旋转到目标点

bool Gcode_G4(unsigned long time_10ms);	//暂停一段时间(单位10ms)
void Gcode_G4(Gcode_Class *command);	//从指令中获取暂停时间，暂停

void Gcode_G90(void);	//设定输入为绝对坐标
void Gcode_G91(void);	//设定输入为相对坐标

void Gcode_M17(void);	//启动所有电机
void Gcode_M18(void);	//禁用所有电机

void Gcode_I0(void);	//急停
void Gcode_I30(void);	//清除指令队列
void Gcode_I114(void);	//获取坐标