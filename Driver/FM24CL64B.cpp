#include "FM24CL64B.h"

//************************************
// Method:    Set_ADD
// FullName:  FM24CL64B_Class::Set_ADD
// Access:    private
// Returns:   void
// Parameter: uint16_t data_add 待跳转的数据地址
// Parameter: uint16_t iic_add 待设置的器件地址
// Description:	设置IIC上挂载的EEPROM待操作的数据地址
//************************************
void FM24CL64B_Class::Set_ADD(uint16_t data_add, uint16_t iic_add)
{
	Start();
	Send_Byte(iic_add & (~0x01)); //发送写命令
	Wait_ACK();
	Send_Byte(highByte(data_add));
	Wait_ACK();
	Send_Byte(lowByte(data_add));
	Wait_ACK();
}

void FM24CL64B_Class::Read_Byte(uint8_t *source, uint16_t num)
{
	while (num > 1)
	{
		*source = IIC_Class::Read_Byte(true);
		++source;
		--num;
	}
	*source = IIC_Class::Read_Byte(false);
}

void FM24CL64B_Class::Write_Byte(uint8_t *source, uint16_t num)
{
	while (num)
	{
		IIC_Class::Send_Byte(*source);
		IIC_Class::Wait_ACK();
		++source;
		--num;
	}
}
