#pragma once
#include <stm32f4xx_dma.h>

class DMA_Base_Class
{
public:
	DMA_Base_Class(DMA_Stream_TypeDef *DMA_Channel) : dma_channel(DMA_Channel) {}
	~DMA_Base_Class() = default;
	void Init(DMA_InitTypeDef *DMA_InitStruct) { DMA_Init(dma_channel, DMA_InitStruct); } //初始化DMA
	void Set_Data_Num(uint32_t number);													  //关闭DMA，设置DMA传输数据的数量，打开DMA通道
	void Close(void) { dma_channel->CR &= ~DMA_SxCR_EN; }								  //关闭DMA
	void Open(void) { dma_channel->CR |= DMA_SxCR_EN; }									  //打开DMA
	void ITConfig(uint32_t DMA_IT, FunctionalState NewState);							  //开启中断

private:
	DMA_Stream_TypeDef *dma_channel;
};
