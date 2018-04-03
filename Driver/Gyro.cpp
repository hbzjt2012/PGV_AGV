#include "Gyro.h"
#include <cstring>
#include <stm32f4xx_gpio.h>
#include <misc.h>

bool Gyro_Class::rx_flag = false;
uint16_t Gyro_Class::tx_cnt = 0;			   //发送字节的计数
uint16_t Gyro_Class::rx_cnt = 0;			   //接收字节的计数
uint8_t Gyro_Class::TX_buf[16] = {0};		   //发送数据的缓冲区，若缓冲区满，则不会发送
volatile uint8_t Gyro_Class::RX_buf[32] = {0}; //接收数据的缓冲区

uint8_t Gyro_Class::data_Buf[16] = {0};

//************************************
// Method:    Init
// FullName:  Gyro_Class::Init
// Access:    public
// Returns:   void
// Parameter: uint32_t baudrate
// Description:	配置串口2
//************************************
void Gyro_Class::Init(uint32_t baudrate)
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
	TIM_Base_Class::Init(TIM7, TIM7_Arr, 10);								   //设置定时器7的中断频率，用于设置串口接收超时检测

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

void Gyro_Class::write(const char c)
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
		Gyro_Class::rx_flag = true; //接受到了一帧数据
		memcpy(Gyro_Class::data_Buf, (uint8_t *)Gyro_Class::RX_buf, 16);
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
		if (Gyro_Class::rx_cnt < 32)
		{
			Gyro_Class::RX_buf[Gyro_Class::rx_cnt] = temp;
			++Gyro_Class::rx_cnt;
		}
	}
}