#pragma once
#include "Wireless_Serial.h"
#include "../HALayer/IO.h"

class AS62_Class final :public Wireless_Serial_Class
{
public:
	AS62_Class() :Wireless_Serial_Class(), M0(Wireless_M0_Port, Wireless_M0_Pin), M1(Wireless_M1_Port, Wireless_M1_Pin) {}
	~AS62_Class() = default;

	void Init(uint32_t baudrate); //根据波特率初始化串口,设置M0脚和M1脚

	uint32_t Return_Baudrate(void) override;	//返回串口波特率
	void Mode(WF_Mode mode, bool value) override;

private:

	bool Analyze_Para_Data(const char*rx) override;	//分析参数数据
	void Code_Para_Data(void) override;	//对参数进行编码,发送
	void Send_Read_CMD(void) override;	//发送读参数指令

	IO_Class M0;  //AS62的M0脚
	IO_Class M1; //AS62的M1脚
};
