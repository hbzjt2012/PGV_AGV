#include "EEPROM.h"

//************************************
// Method:    Read
// FullName:  EEPROM_Class::Read
// Access:    public
// Returns:   uint32_t
// Parameter: uint16_t add
// Parameter: uint16_t length	最大为4
// Description:	从指定的地址开始读取占length字节长度的数据
//************************************
uint32_t EEPROM_Class::Read(uint16_t add, uint16_t length)
{
	uint32_t temp = 0;
	if (length > 4)
	{
		length = 4;
	}
	Read(add, (uint8_t *)&temp, length);
	return temp;
}

//************************************
// Method:    Read
// FullName:  EEPROM_Class::Read
// Access:    public
// Returns:   void
// Parameter: uint16_t add
// Parameter: uint8_t * source
// Parameter: uint16_t num
// Description:
//************************************
void EEPROM_Class::Read(uint16_t add, uint8_t *source, uint16_t num)
{
	if (add + num <= 8192) //待读取的数据在第一个EEPROM内
	{
		Set_ADD(add, IIC_ADD);
		IIC_Class::Start();				 //重复起始命令
		IIC_Class::Send_Byte(IIC_ADD | 0x01); //读字节
		IIC_Class::Wait_ACK();

		Read_Byte(source, num);
		IIC_Class::Stop();
	}
	else if (add < 8192 && add + num > 8192) //待读的数据跨越了两片芯片
	{
		uint8_t i = add + num - 8192;
		uint8_t j = 8192 - add;
		Set_ADD(add, IIC_ADD);
		IIC_Class::Start();
		IIC_Class::Send_Byte(IIC_ADD | 0x01); //读字节
		IIC_Class::Wait_ACK();
		Read_Byte(source, j);
		IIC_Class::Stop();

		Set_ADD(0, IIC_ADD | _BV(1));
		IIC_Class::Start();
		IIC_Class::Send_Byte((IIC_ADD | _BV(1)) | 0x01); //读字节
		IIC_Class::Wait_ACK();
		Read_Byte(source + j, i);
		IIC_Class::Stop();
	}
	else //待读取的数据在第二片芯片
	{
		uint8_t i = add - 8192;
		Set_ADD(i, IIC_ADD | _BV(1));
		IIC_Class::Start();
		IIC_Class::Send_Byte((IIC_ADD | _BV(1)) | 0x01); //读字节
		IIC_Class::Wait_ACK();
		Read_Byte(source, num);
		IIC_Class::Stop();
	}
}

float EEPROM_Class::Read_float(uint16_t add)
{
	uint32_t temp = 0;
	float temp_float;
	temp = Read(add, 4);
	temp_float = *((float *)&temp);
	return temp_float;
}

void EEPROM_Class::Write(uint16_t add, uint32_t data, uint16_t length)
{
	uint8_t *temp = (uint8_t *)&data;
	Write(add, temp, length);
}

void EEPROM_Class::Write(uint16_t add, uint8_t *source, uint16_t num)
{
	if (add + num <= 8192) //待读取的数据在第一个EEPROM内
	{
		Set_ADD(add, IIC_ADD);
		Write_Byte(source, num);
		IIC_Class::Stop();
	}
	else if (add < 8192 && add + num > 8192) //待读的数据跨越了两片芯片
	{
		uint8_t i = add + num - 8192;
		uint8_t j = 8192 - add;
		Set_ADD(add, IIC_ADD);
		Write_Byte(source, j);
		IIC_Class::Stop();

		Set_ADD(0, IIC_ADD | _BV(1));
		Write_Byte(source + j, i);
		IIC_Class::Stop();
	}
	else //待读取的数据在第二片芯片
	{
		uint8_t i = add - 8192;
		Set_ADD(i, IIC_ADD | _BV(1));
		Write_Byte(source, num);
		IIC_Class::Stop();
	}
}

void EEPROM_Class::Write_float(uint16_t add, float data)
{
	uint32_t temp;
	temp = *((uint32_t *)&data);
	Write(add, temp, 4);
}
