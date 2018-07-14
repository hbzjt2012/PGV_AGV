#include "Serial.h"
#include "../HALayer/IO.h"

bool Serial_Class::rx_flag = false;
char Serial_Class::TX_buf[1024] = { 0 };			//发送数据的缓冲区，若缓冲区满，则不会发送
volatile char Serial_Class::RX_buf[1024] = { 0 }; //接收数据的缓冲区
uint16_t Serial_Class::tx_cnt = 0;				//发送字节的计数
uint16_t Serial_Class::rx_cnt = 0;				//接收字节的计数

DMA_Base_Class Serial_Class::TX_DMA = DMA_Base_Class(Serial_TX_DMA_Stream);
DMA_Base_Class Serial_Class::RX_DMA = DMA_Base_Class(Serial_RX_DMA_Stream);

void Serial_Class::Init(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	IO_Class TX = IO_Class(Serial_Uart_TX_Port, Serial_Uart_TX_Pin);
	IO_Class RX = IO_Class(Serial_Uart_RX_Port, Serial_Uart_RX_Pin);

	//配置DMA
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_Channel = Serial_TX_DMA_Channel;
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
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	TX_DMA.Init(&DMA_InitStructure);

	//配置接收中断
	DMA_InitStructure.DMA_BufferSize = 1024;
	DMA_InitStructure.DMA_Channel = Serial_RX_DMA_Channel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&RX_buf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	RX_DMA.Init(&DMA_InitStructure);

	RX_DMA.Open();	//开启DMA接收

	TX.Init(GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP);
	RX.Init(GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP);

	GPIO_PinAFConfig(Serial_Uart_TX_Port, Serial_Uart_TX_PinSource, Serial_Uart_TX_AF);
	GPIO_PinAFConfig(Serial_Uart_RX_Port, Serial_Uart_RX_PinSource, Serial_Uart_RX_AF);

	//配置串口
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验位;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//一个停止位;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长为8位数据格式;
	Uart_Base_Class::Init(&USART_InitStructure);

	enable(); //开启串口

	//配置中断
	NVIC_InitStructure.NVIC_IRQChannel = Serial_Uart_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	asm("nop");

	Clear_IDLE_Flag();

	Uart->CR3 |= USART_DMAReq_Tx | USART_DMAReq_Rx; //打开DMA_TX、DMA_RX请求
	//USART_ITConfig(Uart, USART_IT_IDLE, ENABLE);//开启空闲线路中断
	Uart->CR1 |= _BV(4);	//开启空闲中断
}

void Serial_Class::flush(void)
{
	if (tx_cnt > 0)
	{
		TX_DMA.Set_Data_Num(tx_cnt); //设置要发送的数据数量
		tx_cnt = 0;
	}
}

void Serial_Class::write(const char c)
{
	if (tx_cnt < 1024)
	{
		TX_buf[tx_cnt] = c;
		++tx_cnt;
	}
}

void Serial_Uart_IRQHandler(void)
{
	if (Serial_Uart_Port->SR&USART_FLAG_IDLE)	//接收空闲中断
	{
		//软件序列清空中断标志
		uint16_t temp = Serial_Uart_Port->SR;
		temp = Serial_Uart_Port->DR;
		Serial_Class::rx_flag = true;
		Serial_Class::rx_cnt = 1024 - Serial_Class::RX_DMA.Set_Data_Num(1024);
		Serial_Class::RX_buf[Serial_Class::rx_cnt] = '\0';
		//Serial_RX_DMA_Stream->CR &= ~DMA_SxCR_EN;	//关闭DMA
		//Serial_Class::rx_cnt = 1024 - Serial_RX_DMA_Stream->NDTR;
		//Serial_RX_DMA_Stream->NDTR = 1024;
		//Serial_RX_DMA_Stream->CR |= DMA_SxCR_EN;	//开启DMA
	}
}
