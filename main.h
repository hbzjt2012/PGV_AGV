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
#include "./Math/Kalman_Filter_Angle.h"
#include "./Math/Kalman_Filter_Line.h"
#include "./Math/Kalman_Filter_Coor.h"
#include "./DSP_Lib/arm_math.h"
#include "./HardwareDefine/Version_Boards.h"

#define Gcode_Command_Buf_SIZE	32
#define Movement_Command_Buf_SIZE	128

/*
* TIM1 用于三色指示灯(暂定)
* TIM2 通道1输出IO口预留
* TIM3 编码器FR
* TIM4 编码器BL
* TIM5 编码器FL
* TIM6
* TIM7
* TIM8 编码器BR
* TIM9 电机FL、FR转速
* TIM10
* TIM11 用于获取编码器计算周期
* TIM12 BR、BL轮子转速
* TIM13 用于控制周期测量
* TIM14 用于避障、按键等的扫描(暂定)
*/

/*
*			中断向量						抢占优先级				响应优先级
* Serial_Uart_IRQIRQHandler(通信用串口)			3						1
* PGV_Uart_IRQHandler(用于PGV串口)				1						1
* Gyro_Uart_IRQHandler	(用于陀螺仪串口)		2						1
*/

/*
*	  DMA_Channel				优先级
* Gyro_TX_DMA_Channel			  低
* Gyro_RX_DMA_Channel			  高
* Serial_TX_DMA_Channel			  低
* Serial_RX_DMA_Channel			 中等
* PGV_TX_DMA_Channel			  高
* PGV_RX_DMA_Channel			非常高
*/


//若之后仍出现到达目标点之后，速度突变(目标坐标突变)
//则修改插补路径中的坐标投影方式，将角度剔除，使用xy计算因子，再计算投影坐标

void Init_System(void);	//配置系统所需的硬件、外设
void Init_System_RCC(void);		//初始化系统所需时钟

void Location_AGV(void);		//AGV定位函数
void Process_Movement_Command(void);	//获取并处理运动指令
void Movement_Control(void);	//运动控制
void Check_Avoidance_Buton(void);	//检查避障和按键动作
void Parse_Sensor_Data(void);	//处理传感器数据
void Process_Gcode_Command(AGV_State::Gcode_Command_State &state); //获取并处理Gcode命令指令
void Update_Print_MSG(void);		//打印信息

void Location_AGV_demo(void);	//去除了TL740的定位测试程序

//添加运动指令，返回添加结果
//缓存区满，未做相应处理
AGV_State::Movement_Command_State Add_Movement_Command(const Coordinate_Class &Destination, Movement_Class *&command, const float threshold);

//根据当前坐标获取目标速度和坐标，返回执行结果（true表示执行完毕）
bool Run_Movement_Command(Movement_Class*movement_command, const Coordinate_Class &Current_Coor);
bool Run_Gcode_Command(Gcode_Class *gcode_command);	//执行Gcode指令，返回执行结果，true表示执行完毕


//获取指令中的坐标
Coordinate_Class Get_Command_Coor(Gcode_Class *command, const Coordinate_Class &Base_Coor_InWorld, bool Is_Absolute_Coor = true);

void Gcode_G0(Gcode_Class *command);	//先旋转后直线运动到目标点
void Gcode_G1(Gcode_Class *command);	//先直线运动后旋转到目标点
void Gcode_G2(Gcode_Class *command);	//直接运动到目标点

void Gcode_G3(Gcode_Class *command);	//以AGV的x轴逆时针角度C，距离为R的点为圆心，顺时针运动角度Z
void Gcode_G4(Gcode_Class *command);	//以AGV的x轴逆时针角度C，距离为R的点为圆心，逆时针运动角度Z

bool Gcode_G5(unsigned long time_10ms);	//暂停一段时间(单位10ms)
void Gcode_G5(Gcode_Class *command);	//从指令中获取暂停时间，暂停

void Gcode_G6(Gcode_Class *command);	//快速移动到坐标
void Gcode_G10(Gcode_Class *command);	//设定原点坐标在世界坐标系中的坐标(设定坐标偏置)
void Gcode_G10(const Coordinate_Class &Base_Coor);	//设定原点坐标偏置

void Gcode_G28(Gcode_Class *command);	//移动到原点
void Gcode_G28(void);	//移动到原点

void Gcode_G90(void);	//设定输入为绝对坐标
void Gcode_G91(void);	//设定输入为相对坐标

void Gcode_G92(Gcode_Class *command);	//设定当前坐标
void Gcode_G92(const Coordinate_Class &Coor);	//设置当前坐标为Coor


void Gcode_M15(void);	//失能所有电机
void Gcode_M16(void);	//使能所有电机
void Gcode_M17(void);	//所有电机刹车
void Gcode_M18(void);	//所有电机刹车解除

void Gcode_M72(Gcode_Class *command);	//播放指定曲目
void Gcode_M132(Gcode_Class *command);	//设置参数值
void Gcode_M133(Gcode_Class *command);	//返回参数值
void Gcode_M134(Gcode_Class *command);	//设置参数值，并写入EEPROM

void Gcode_I0(void);	//急停
void Gcode_I30(void);	//清除指令队列
void Gcode_I17(void);	//所有电机刹车，同M17
void Gcode_I18(void);	//所有电机刹车解除，同M18
void Gcode_I114(void);	//获取当前坐标
void Gcode_I114(const Coordinate_Class &Coor);
void Gcode_I115(void);	//获取当前速度
void Gcode_I115(const Velocity_Class &Velocity);