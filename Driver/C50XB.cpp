#include "C50XB.h"
#include "cstring"

#define C50XB_CS	WF_Set0
#define C50XB_SET	WF_Set1

enum C50XB_Baud
{
	C50XB_BAUDRATE_1200 = 0,
	C50XB_BAUDRATE_2400,
	C50XB_BAUDRATE_4800,
	C50XB_BAUDRATE_9600,
	C50XB_BAUDRATE_14400,
	C50XB_BAUDRATE_19200,
	C50XB_BAUDRATE_38400,
	C50XB_BAUDRATE_57600,
	C50XB_BAUDRATE_76800,
	C50XB_BAUDRATE_115200
};

void C50XB_Class::Init(uint32_t baudrate)
{
	CS.Init(GPIO_Mode_OUT);
	SET.Init(GPIO_Mode_OUT);

	Mode(Wireless_Serial_Class::Normal, true);

	Serial_Class::Init(baudrate);
}

uint32_t C50XB_Class::Return_Baudrate(void)
{
	uint32_t baud = 9600;
	switch (baudrate_level)
	{
	case C50XB_BAUDRATE_1200:
		baud = 1200;
		break;
	case C50XB_BAUDRATE_2400:
		baud = 2400;
		break;
	case C50XB_BAUDRATE_4800:
		baud = 4800;
		break;
	case C50XB_BAUDRATE_9600:
		baud = 9600;
		break;
	case C50XB_BAUDRATE_14400:
		baud = 14400;
		break;
	case C50XB_BAUDRATE_19200:
		baud = 19200;
		break;
	case C50XB_BAUDRATE_38400:
		baud = 38400;
		break;
	case C50XB_BAUDRATE_57600:
		baud = 57600;
		break;
	case C50XB_BAUDRATE_76800:
		baud = 76800;
		break;
	case C50XB_BAUDRATE_115200:
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
// FullName:  C50XB_Class::Mode
// Access:    public 
// Returns:   void
// Parameter: WF_Mode mode
// Parameter: bool value true-进入;false-退出
// Description: 设置无线模块进入或者退出某种模式
//				正常通信模式进入或退出均为进入
//************************************
void C50XB_Class::Mode(WF_Mode mode, bool value)
{
	switch (mode)
	{
	case Wireless_Serial_Class::WF_Mode::Normal:
		C50XB_SET = true;
		C50XB_CS = true;
		break;
	case Wireless_Serial_Class::WF_Mode::Set_Mode:
		C50XB_SET = !value;	//SET脚拉低，进入设置模式，置高，退出设置模式
		break;
	case Wireless_Serial_Class::WF_Mode::Sleep_Mode:
		C50XB_CS = !value;	//CS脚拉低，进入休眠模式，置高，休眠设置模式
		break;
	default:
		break;
	}
	SET.Write(!C50XB_SET);	//电路反向
	CS.Write(!C50XB_CS);	//电路反向
}


//************************************
// Method:    Analyze_Para_Data
// FullName:  C50XB_Class::Analyze_Para_Data
// Access:    virtual private 
// Returns:   bool
// Parameter: const char * rx
// Description: 分析参数数据
//************************************
bool C50XB_Class::Analyze_Para_Data(const char* rx)
{
	channel = rx[0];
	wfrate_level = rx[2];
	baudrate_level = rx[4];
	return true;
}

//************************************
// Method:    Code_Para_Data
// FullName:  C50XB_Class::Code_Para_Data
// Access:    virtual private 
// Returns:   void
// Parameter: void
// Description: 对参数进行编码,发送
//************************************
void C50XB_Class::Code_Para_Data(void)
{
	char Para[17] = { 0 };

	memcpy(Para, "\xAA\xFA\x03\x14\x01\x03\x07\x03\x02\x01\x01", 11);

	Para[3] = channel;
	Para[5] = wfrate_level;
	Para[7] = baudrate_level;
	print(Para, 17);
}

void C50XB_Class::Send_Read_CMD(void)
{
	print("\xAA\xFA\x01", 3);	//查询指令
}