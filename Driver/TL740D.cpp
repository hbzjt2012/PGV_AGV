#include "TL740D.h"
#include "./macros.h"

bool TL740D_Class::Analyze_Data(void)
{
	if (rx_cnt >= 14)
	{
		uint8_t bc = 0;
		for (int i = 1; i < 13; i++)
		{
			bc += data_Buf[i];
		}
		Clear_rx_cnt();
		//rx_cnt = 0;
		if (bc == data_Buf[13])
		{
			int32_t z_head_temp = 0;
			z_rate = BCD2DEC((uint8_t *)(&data_Buf[4])) / 100.0f;
			forward_accel = BCD2DEC((uint8_t *)(&data_Buf[7])) / 1000.0f;
			z_head_temp = BCD2DEC((uint8_t *)(&data_Buf[10]));
			if (z_head_temp < 0)
			{
				z_head_temp += 36000;
			}
			z_heading = z_head_temp / 100.0f;
			z_rate = z_rate / 180 * M_PI;
			return true;
		}
		return false;
	}
	else
	{
		rx_cnt = 0;
		return false;
	}
}

int32_t TL740D_Class::BCD2DEC(const uint8_t *source)
{
	int32_t temp = 0, temp1 = 0;
	for (int i = 0; i < 3; i++)
	{
		temp *= 100;
		//temp = ((*source & 0xF0)>>4)*10;
		//temp1 = (*source & 0x0F);
		temp += ((*source & 0xF0) >> 4) * 10 + (*source & 0x0F);
		++source;
	}
	if (temp >= 100000)
	{
		temp = 100000 - temp;
	}
	return temp;
}

void TL740D_Class::Read_Data(void)
{
	print((uint8_t *)"\x68\x04\x00\x04\x08", 5);
	TX_DMA.Set_Data_Num(tx_cnt); //设置要发送的数据数量
	tx_cnt = 0;
}
