#include "TL740D.h"
#include "./macros.h"
#include <cstring>
#include <stm32f4xx_gpio.h>
#include <misc.h>

bool TL740D_Class::rx_flag = false;
uint16_t TL740D_Class::tx_cnt = 0;			   //发送字节的计数
uint16_t TL740D_Class::rx_cnt = 0;			   //接收字节的计数
uint8_t TL740D_Class::TX_buf[16] = { 0 };		   //发送数据的缓冲区，若缓冲区满，则不会发送
volatile uint8_t TL740D_Class::RX_buf[32] = { 0 }; //接收数据的缓冲区

uint8_t TL740D_Class::data_Buf[24] = { 0 };

void TL740D_Class::Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t TIM7_Arr = 0; //TIM7重装值（用于检测串口接收超时）

	//配置DMA
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;			//通道4
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; //内存到外设
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  //直接传输
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&TX_buf;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; //突发单次传输
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (Uart->DR);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	TX_DMA.Init(&DMA_InitStructure);

	//配置TIM7中断时间
	//定时器7的时钟频率为系统时钟的一半（APB1分配系数2，定时器频率*2）
	TIM7_Arr = (uint16_t)(SystemCoreClock / 2 / (baudrate / 22) / 10 + 1); //设置静默时间为22个位(定时器分频系数10)
	TIM_Base_Class::Init(TIM7, TIM7_Arr, 10, true);							//设置定时器7的中断频率，用于设置串口接收超时检测


	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	//配置串口用的GPIO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD5 与 GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		   //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  //速度 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		   //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		   //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);				   //初始化 PA0， PA1

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //PD5复用为 UART_TX
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //PD6 复用为 UART_RX

															  //配置串口
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验位;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//一个停止位;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长为8位数据格式;
	Uart_Base_Class::Init(&USART_InitStructure);

	Uart->CR3 |= USART_DMAReq_Tx; //打开DMA_TX请求
	Uart->CR1 |= (1 << 5);		  //开启RXNE中断

								  //配置中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	enable(); //开启串口
}

bool TL740D_Class::Analyze_Data(void)
{
	int length = rx_cnt;
	Clear_rx_cnt();
	for (int j = 0; j + 14 <= length; j++)
	{
		if (data_Buf[j] != (uint8_t)0x68)
		{
			continue;//寻找帧头
		}

		uint8_t bc = 0;

		for (int i = 1; i < 13; i++)
		{
			bc += data_Buf[i + j];
		}

		if (bc == data_Buf[j + 13])
		{
			int32_t z_head_temp = 0;
			z_rate = BCD2DEC((uint8_t *)(&data_Buf[j + 4])) / 100.0f;
			forward_accel = BCD2DEC((uint8_t *)(&data_Buf[j + 7])) / 1000.0f;
			z_head_temp = BCD2DEC((uint8_t *)(&data_Buf[j + 10]));
			//if (z_head_temp < 0)
			//{
			//	z_head_temp += 36000;
			//}
			z_heading = z_head_temp / 100.0f;
			//z_rate = z_rate / 180 * M_PI;

			z_heading = z_heading - z_heading_bias;
			z_heading = Coordinate_Class::Transform_Angle(z_heading);

			//z_heading_bias = z_heading;	//设置基准

			return true;
		}

	}
	return false;
}

void TL740D_Class::Read_Data(void)
{
	print((uint8_t *)"\x68\x04\x00\x04\x08", 5);
	TX_DMA.Set_Data_Num(tx_cnt); //设置要发送的数据数量
	tx_cnt = 0;
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

void TL740D_Class::write(const char c)
{
	if (tx_cnt < 16)
	{
		TX_buf[tx_cnt] = c;
		++tx_cnt;
	}
}


void TIM7_IRQHandler(void)
{
	if (TIM7->SR & TIM_IT_Update) //更新中断
	{
		TIM7->SR = ~TIM_IT_Update;
		TIM7->CR1 &= ~TIM_CR1_CEN;  //关闭定时器6
		TL740D_Class::rx_flag = true; //接受到了一帧数据
		memcpy(TL740D_Class::data_Buf, (uint8_t *)TL740D_Class::RX_buf, 24);
	}
}

void USART2_IRQHandler(void)
{
	static uint8_t temp = 0;
	if (USART2->SR & USART_FLAG_RXNE) //接收中断
	{
		TIM7->CNT = 0;			  //计数器清0
		TIM7->CR1 |= TIM_CR1_CEN; //使能定时器1
		temp = USART2->DR;
		if (TL740D_Class::rx_cnt < 32)
		{
			TL740D_Class::RX_buf[TL740D_Class::rx_cnt] = temp;
			++TL740D_Class::rx_cnt;
		}
	}
}