#pragma once

/*
* 由串口派生得到的无线串口
* 提供了设置无线透传模块、使无线模块进入休眠的接口
*/
#include "Serial.h"

class Wireless_Serial_Class : public Serial_Class
{
public:
	typedef enum
	{
		SET,	//设置无线模块参数
		READ	//读取无线模块参数
	}PAR_SET_OR_READ;	//设置或读取参数

	Wireless_Serial_Class() : Serial_Class() {};
	virtual ~Wireless_Serial_Class() = default;

	enum SET_WF_Property
	{
		SET_BAUDRATE = 0x01,		//设置波特率
		SET_WF_RATE = 0x02,		//设置无线速率
		SET_CHANNEL = 0x03		//设置信道
	};

	bool Parameter(PAR_SET_OR_READ value);	//设置或读取无线模块参数
	virtual uint32_t Return_Baudrate(void) = 0;	//返回串口波特率

	typedef enum
	{
		Normal,	//正常通信模式
		Set_Mode,	//设置模式
		Sleep_Mode	//睡眠模式
	}WF_Mode;
	virtual void Mode(WF_Mode mode, bool value) = 0;	//true-进入该模式，false-退出该模式

	/*
	* 两个引脚为无线串口的设置用引脚
	* WF_Set0与C50XB_CS、AS62_M0连接，WF_Set1与C50XB_SET、AS62_M1连接
	*/
	bool WF_Set0;
	bool WF_Set1;

	/*
	* 无线模块可设置的属性
	* 波特率：单片机和无线串口通信用的波特率
	* 无线速率：无线串口之间通信所用的速率
	* 信道：无线串口之间通信的信号频率
	*/
	uint8_t baudrate_level; //波特率速率等级
	uint8_t wfrate_level;   //无线速率等级
	uint8_t channel;		//无线通信的信道

private:
	virtual bool Analyze_Para_Data(const char*rx) = 0;	//分析参数数据
	virtual void Code_Para_Data(void) = 0;	//对参数进行编码,发送
	virtual void Send_Read_CMD(void) = 0;	//发送读参数指令
};