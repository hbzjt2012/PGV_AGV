#pragma once
#include "IO.h"
#include "../delay.h"

/*
* IIC驱动类，提供了IIC的读写函数
*/
class IIC_Class
{
public:
	IIC_Class(IO_Class SDA, IO_Class SCL) : SDA(SDA), SCL(SCL) {}
	virtual ~IIC_Class() = default;

	void Init(void);

private:
	IO_Class SDA;
	IO_Class SCL;

	void ACK(void);
	void NACK(void);

protected:
	void Send_Byte(uint8_t txd);
	uint8_t Read_Byte(bool ack);
	void Start(void);
	void Stop(void);
	bool Wait_ACK(void);
};