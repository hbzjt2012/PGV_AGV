#pragma once
#include "../macros.h"
#include "../HALayer/IIC.h"

//针对单片容量为8k的EEPROM芯片，最多两片串联

class EEPROM_Class : public IIC_Class
{
public:
	EEPROM_Class(IO_Class SDA, IO_Class SCL, uint8_t add) : IIC_Class(SDA, SCL), IIC_ADD(add) {}
	virtual ~EEPROM_Class() = default;

	uint32_t Read(uint16_t add, uint16_t length = 1);			//从指定地址读取length个字节数据
	void Read(uint16_t add, uint8_t *source, uint16_t num = 1); //从指定地址读取num个字节
	float Read_float(uint16_t add);								//从指定地址读取一个浮点数

	void Write(uint16_t add, uint32_t data, uint16_t length = 1); //在指定地址写入length个数据
	void Write(uint16_t add, uint8_t *source, uint16_t num = 1);  //在指定地址写入num个字节
	void Write_float(uint16_t add, float data);					  //在指定地址处写入一个浮点数

private:
	const uint8_t IIC_ADD;										   //挂载在IIC总线上的器件地址
	virtual void Set_ADD(uint16_t data_add, uint16_t iic_add) = 0; //设置EEPROM的数据地址
	virtual void Read_Byte(uint8_t *source, uint16_t num) = 0;	 //读指定数量的字节
	virtual void Write_Byte(uint8_t *source, uint16_t num) = 0;	//写指定数量的字节
};