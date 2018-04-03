#include "Wireless_Serial.h"
#include "../delay.h"

//************************************
// Method:    Parameter
// FullName:  WirelessSerial_Class::Parameter
// Access:    public 
// Returns:   bool true-设置或读取参数成功，false-设置或读取参数失败
// Parameter: PAR_SET_OR_READ value
// Description: 设置或者读取无线模块参数
//************************************
bool Wireless_Serial_Class::Parameter(PAR_SET_OR_READ value)
{
	switch (value)
	{
	case SET:
		Code_Para_Data();	//对无线参数进行编码并发送
		return true;
		break;
	case READ:
		Clear_rx_flag();	//清除接收标志
		Clear_rx_cnt();	//清除计数
		Send_Read_CMD();	//发送读参数命令
		delay_ms(200);	//等待模块返回数据
		if (Return_rx_flag())	//返回数据
		{
			Clear_rx_flag();	//清除接收标志
			Clear_rx_cnt();	//清除计数
			return Analyze_Para_Data(Return_RX_buf());	//解析参数
		}
		break;
	default:
		break;
	}
	return false;
}