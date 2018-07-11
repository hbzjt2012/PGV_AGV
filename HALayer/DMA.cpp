#include "DMA.h"

uint32_t DMA_Base_Class::Set_Data_Num(uint32_t number)
{
	/*
	* 在stm32f4的手册中，DMA-SR寄存器说明下面有一句话
	* 将 EN 位置“1”以启动新传输之前， DMA_LISR 或 DMA_HISR 寄存器中与数据流相对应的事件标志必须清零
	*
	* 在传输结束前禁止数据流（EN位清零），会产生传输完成事件
	*/

	uint8_t x = 0;		   //指示当前是哪个流，DMAy_Streamx
	uint32_t add_temp = 0; //保存需修改寄存器的地址
	uint32_t temp = 0;

	Close();
	if (dma_stream < DMA2_Stream0)
	{
		x = ((uint32_t)dma_stream - (uint32_t)DMA1_Stream0) / 0x18; //获取当前流控制器
		add_temp = (uint32_t) & (DMA1->LIFCR) + (x & (0x04));		 //保存寄存器地址
	}
	else
	{
		x = ((uint32_t)dma_stream - (uint32_t)DMA2_Stream0) / 0x18;
		add_temp = (uint32_t) & (DMA2->LIFCR) + (x & (0x04)); //保存寄存器地址
	}
	//temp = x & 0x02;
	//temp = temp<< 3;
	//temp = (0x3D << (6 * (x & 0x01)))<<((x&0x02)<<3);
	//*(uint32_t*)add_temp = temp;
	temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);
	*(uint32_t *)add_temp = temp;//清除所有中断标志
	//*(uint32_t*)add_temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);	

	uint32_t remaining = dma_stream->NDTR;
	dma_stream->NDTR = number;
	Open();
	return remaining;
}

void DMA_Base_Class::ITConfig(uint32_t DMA_IT, FunctionalState NewState)
{
	DMA_ITConfig(dma_stream, DMA_IT, NewState);
}
