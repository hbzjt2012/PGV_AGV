#pragma once
#include "EEPROM.h"
#include "../HardwareDefine/Version_Boards.h"

class FM24CL64B_Class : public EEPROM_Class
{
  public:
	FM24CL64B_Class() : EEPROM_Class(IO_Class(IIC_SDA_GPIO_Port, IIC_SDA_GPIO_Pin), IO_Class(IIC_SCL_GPIO_Port, IIC_SCL_GPIO_Pin), 0xA0) {}
	~FM24CL64B_Class() = default;

  private:
	void Set_ADD(uint16_t data_add, uint16_t iic_add) override; //设置EEPROM的数据地址
	void Read_Byte(uint8_t *source, uint16_t num) override;		//读指定数量的字节
	void Write_Byte(uint8_t *source, uint16_t num) override;	//写指定数量的字节
};