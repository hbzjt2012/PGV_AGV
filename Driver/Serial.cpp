#include "Serial.h"

bool Serial_Class::rx_flag = false;
char Serial_Class::TX_buf[1024] = { 0 };			//发送数据的缓冲区，若缓冲区满，则不会发送
volatile char Serial_Class::RX_buf[1024] = { 0 }; //接收数据的缓冲区
uint16_t Serial_Class::tx_cnt = 0;				//发送字节的计数
uint16_t Serial_Class::rx_cnt = 0;				//接收字节的计数

void Serial_Class::Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t TIM6_Arr = 0; //TIM6重装值（用于检测串口接收超时）

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
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	TX_DMA.Init(&DMA_InitStructure);

	//配置TIM6中断时间
	//定时器6的时钟频率为系统时钟的一半（APB1分配系数2，定时器频率*2）
	TIM6_Arr = (uint16_t)(SystemCoreClock / 2 / (baudrate / 22) / 10 + 1); //设置静默时间为22个位(定时器分频系数10)
	TIM_Base_Class::Init(TIM6, TIM6_Arr, 10,true);	//设置定时器6的中断频率，用于设置串口接收超时检测		   

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	//配置串口用的GPIO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA0 与 GPIOA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		   //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  //速度 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		   //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		   //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   //初始化 PA0， PA1

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); //PA0 复用为 UART4_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); //PA1 复用为 UART_RX

	//配置串口
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验位;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//一个停止位;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长为8位数据格式;
	Uart_Base_Class::Init(&USART_InitStructure);

	Uart->CR3 |= USART_DMAReq_Tx; //打开DMA_TX请求
	Uart->CR1 |= _BV(5);		  //开启RXNE中断

	//配置中断
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	enable(); //开启串口
}

void Serial_Class::flush(void)
{
	if (tx_cnt > 0)
	{
		TX_DMA.Set_Data_Num(tx_cnt); //设置要发送的数据数量
		tx_cnt = 0;
	}
}

void Serial_Class::flush_demo(float time_s)
{
	if (tx_cnt > 0)
	{
		print(" S:");
		print(time_s);
		print("\r\n");
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

void TIM6_DAC_IRQHandler(void)
{
	if (TIM6->SR & TIM_IT_Update) //更新中断
	{
		TIM6->SR = ~TIM_IT_Update;
		TIM6->CR1 &= ~TIM_CR1_CEN;						//关闭定时器6
		Serial_Class::rx_flag = true;					//接受到了一帧数据
		Serial_Class::RX_buf[Serial_Class::rx_cnt] = 0; //结尾0
	}
}

void UART4_IRQHandler(void)
{
	static uint8_t temp = 0;
	if (UART4->SR & USART_FLAG_RXNE) //接收中断
	{
		TIM6->CNT = 0;			  //计数器清0
		TIM6->CR1 |= TIM_CR1_CEN; //使能定时器1
		temp = UART4->DR;
		if (Serial_Class::rx_cnt < 1023)
		{
			Serial_Class::RX_buf[Serial_Class::rx_cnt] = temp;
			++Serial_Class::rx_cnt;
		}
	}
}