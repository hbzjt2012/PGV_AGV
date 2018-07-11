#pragma once

/*
* 按照无线串口抽象类提供的接口，完成的C50XB驱动函数
*/

#include "Wireless_Serial.h"
#include "../HALayer/IO.h"

class C50XB_Class final : public Wireless_Serial_Class
{
public:
	C50XB_Class() : Wireless_Serial_Class(), CS(Wireless_M0_Port, Wireless_M0_Pin), SET(Wireless_M1_Port, Wireless_M1_Pin) {}
	~C50XB_Class() = default;

	void Init(uint32_t baudrate); //根据波特率初始化串口,设置CS脚和SET脚

	uint32_t Return_Baudrate(void) override; //返回无线串口波特率
	void Mode(WF_Mode mode, bool value) override;	//true-进入模式，false-退出模式

private:

	bool Analyze_Para_Data(const char*rx) override;	//分析参数数据
	void Code_Para_Data(void) override;	//对参数进行编码,发送
	void Send_Read_CMD(void) override;	//发送读参数指令

	IO_Class CS;  //C50XB的CS脚
	IO_Class SET; //C50XB的SET脚
};