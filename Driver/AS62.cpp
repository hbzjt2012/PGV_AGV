#include "AS62.h"
#include <cstring>

#define AS62_M0	WF_Set0
#define AS62_M1	WF_Set1

enum AS62_Baud
{
	AS62_BAUDRATE_1200 = 0,
	AS62_BAUDRATE_2400,
	AS62_BAUDRATE_4800,
	AS62_BAUDRATE_9600,
	AS62_BAUDRATE_19200,
	AS62_BAUDRATE_38400,
	AS62_BAUDRATE_57600,
	AS62_BAUDRATE_115200
};

void AS62_Class::Init(uint32_t baudrate)
{
	M0.Init(GPIO_Mode_OUT);
	M1.Init(GPIO_Mode_OUT);

	Mode(Wireless_Serial_Class::Normal, true);

	Serial_Class::Init(baudrate);
}

//************************************
// Method:    Return_Baudrate
// FullName:  AS62_Class::Return_Baudrate
// Access:    public 
// Returns:   uint32_t
// Parameter: void
// Description: 返回串口波特率
//************************************
uint32_t AS62_Class::Return_Baudrate(void)
{
	uint32_t baud = 9600;

	switch (baudrate_level)
	{
	case AS62_BAUDRATE_1200:
		baud = 1200;
		break;
	case AS62_BAUDRATE_2400:
		baud = 2400;
		break;
	case AS62_BAUDRATE_4800:
		baud = 4800;
		break;
	case AS62_BAUDRATE_9600:
		baud = 9600;
		break;
	case AS62_BAUDRATE_19200:
		baud = 19200;
		break;
	case AS62_BAUDRATE_38400:
		baud = 38400;
		break;
	case AS62_BAUDRATE_57600:
		baud = 57600;
		break;
	case AS62_BAUDRATE_115200:
		baud = 115200;
		break;
	default:
		baud = 9600;
		break;
	}
	return baud;
}

//************************************
// Method:    Mode
// FullName:  AS62_Class::Mode
// Access:    public 
// Returns:   void
// Parameter: WF_Mode mode
// Parameter: bool value true-进入;false-退出
// Description: 设置无线模块进入或者退出某种模式
//				正常通信模式进入或退出均为进入
//************************************
void AS62_Class::Mode(WF_Mode mode, bool value)
{
	switch (mode)
	{
	case Wireless_Serial_Class::WF_Mode::Normal:
		AS62_M0 = false;
		AS62_M1 = false;
		break;
	case Wireless_Serial_Class::WF_Mode::Set_Mode:
	case Wireless_Serial_Class::WF_Mode::Sleep_Mode:
		AS62_M0 = value;
		AS62_M1 = value;	//M0=1,M1=1进入设置、休眠模式
		break;
	default:
		break;
	}
	M0.Write(!AS62_M0);
	M1.Write(!AS62_M1);
}

//************************************
// Method:    Analyze_Para_Data
// FullName:  AS62_Class::Analyze_Para_Data
// Access:    virtual private 
// Returns:   bool
// Parameter: const char * rx
// Description: 分析参数数据
//************************************
bool AS62_Class::Analyze_Para_Data(const char * rx)
{
	baudrate_level = (rx[3] >> 3) & 0x07;	//获取bit[3:5]
	wfrate_level = (rx[3] & 0x07);	//获取bit[2:0]
	channel = rx[4] & 0x1F;	//获取bit[4:0]
	return true;
}

//************************************
// Method:    Code_Para_Data
// FullName:  AS62_Class::Code_Para_Data
// Access:    virtual private 
// Returns:   void
// Parameter: void
// Description: 对参数编码，发送
//************************************
void AS62_Class::Code_Para_Data(void)
{
	char Para[6] = { 0xC0,0x00,0x00,0x1A,0x17,0x44 };

	Para[3] |= (((baudrate_level & 0x07) << 3) | (wfrate_level & 0x07));	//设置bit[0:5]
	Para[4] |= (channel & 0x1F);	//设置bit[0:4]
	print(Para, 6);
}

void AS62_Class::Send_Read_CMD(void)
{
	print("\xC1\xC1\xC1", 3);	//查询指令
}
