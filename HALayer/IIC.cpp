#include "IIC.h"

//************************************
// Method:    Init
// FullName:  IIC_Class::Init
// Access:    public
// Returns:   void
// Parameter: void
// Description:	初始化IIC设备，设置SDA、SCL为开漏输出，空闲状态（高电平）
//************************************
void IIC_Class::Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	SCL.Init(&GPIO_InitStructure);
	SDA.Init(&GPIO_InitStructure);

	SDA.Set();
	SCL.Set();

	//每次I2C主设备复位后，如果检测到SDA数据线被拉低，则控制I2C中的SCL时钟线产生9个时钟脉冲(针对8位数据的情况)
	if (SDA.Read() == false)
	{
		for (int i = 0; i < 9; i++)
		{
			SCL.Clear();
			delay_us(3);
			SCL.Set();
			delay_us(3);
		}
	}
}

//************************************
// Method:    ACK
// FullName:  IIC_Class::ACK
// Access:    private
// Returns:   void
// Parameter: void
// Description:	产生ACK应答，设备在接受或发送一个字节后拉低SDA
//************************************
void IIC_Class::ACK(void)
{
	SCL.Clear();
	SDA.Clear();
	delay_us(2);
	SCL.Set();
	delay_us(2);
	SCL.Clear();
	SDA.Set();
	delay_us(2);
}

//************************************
// Method:    NACK
// FullName:  IIC_Class::NACK
// Access:    private
// Returns:   void
// Parameter: void
// Description:	产生一个NACK应答，设备在接受或发送一个字节后释放SDA
//************************************
void IIC_Class::NACK(void)
{
	SCL.Clear();
	SDA.Set();
	delay_us(2);
	SCL.Set();
	delay_us(2);
	SCL.Clear();
	SDA.Set();
	delay_us(2);
}

//************************************
// Method:    Send
// FullName:  IIC_Class::Send
// Access:    protected
// Returns:   void
// Parameter: uint8_t txd 待发送的字节
// Description:	使用IIC发送一个字节
//************************************
void IIC_Class::Send_Byte(uint8_t txd)
{
	SCL.Clear();
	for (int i = 0; i < 8; i++)
	{
		SDA.Write(txd & 0x80);
		txd <<= 1;
		delay_us(2);
		SCL.Set();
		delay_us(2);
		SCL.Clear();
		delay_us(2);
	}
}

//************************************
// Method:    Read
// FullName:  IIC_Class::Read
// Access:    protected
// Returns:   uint8_t
// Parameter: bool ack	ack=1,发送ack,ack=0,发送nack
// Description:	读一个字节
//************************************
uint8_t IIC_Class::Read_Byte(bool ack)
{
	uint8_t receive = 0;
	SDA.Set(); //置1释放总线
	for (int i = 0; i < 8; i++)
	{
		SCL.Clear();
		delay_us(2);
		SCL.Set();
		receive <<= 1;
		if (SDA.Read())
		{
			++receive;
		}
		delay_us(1);
	}
	ack ? ACK() : NACK();
	return receive;
}

//************************************
// Method:    Start
// FullName:  IIC_Class::Start
// Access:    protected
// Returns:   void
// Parameter: void
// Description:	产生起始信号,SCL为高电平时，SDA下降沿
//************************************
void IIC_Class::Start(void)
{
	SDA.Set();
	SCL.Set();
	delay_us(4);
	SDA.Clear();
	delay_us(4);
	SCL.Clear(); //拉低SCL信号，准备发送或接受数据
}

//************************************
// Method:    Stop
// FullName:  IIC_Class::Stop
// Access:    protected
// Returns:   void
// Parameter: void
// Description:	产生停止信号，SCL为高电平时，SDA上升沿
//************************************
void IIC_Class::Stop(void)
{
	SCL.Clear();
	SDA.Clear();
	delay_us(4);
	SCL.Set();
	SDA.Set();
	delay_us(4);
}

//************************************
// Method:    Wait_ACK
// FullName:  IIC_Class::Wait_ACK
// Access:    protected
// Returns:   bool	false-接受应答信号失败,true-接受应答成功
// Parameter: void
// Description:	等到应答信号到来
//************************************
bool IIC_Class::Wait_ACK(void)
{
	uint16_t ucErrTime = 0;
	SDA.Set(); //释放总线
	delay_us(1);
	SCL.Set();
	delay_us(1);
	while (SDA.Read())
	{
		++ucErrTime;
		if (ucErrTime > 250) //stm32F1中，该值是250
		{
			Stop();
			return false;
		}
	}
	SCL.Clear();
	return true;
}
