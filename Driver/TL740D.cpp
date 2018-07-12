#include "TL740D.h"
#include "./macros.h"
#include <cstring>
#include "../HALayer/IO.h"
#include <misc.h>
#include "../Math/My_Math.h"

bool TL740D_Class::rx_flag = false;
uint16_t TL740D_Class::tx_cnt = 0;			   //发送字节的计数
uint16_t TL740D_Class::rx_cnt = 0;			   //接收字节的计数
uint8_t TL740D_Class::TX_buf[32] = { 0 };		   //发送数据的缓冲区，若缓冲区满，则不会发送
volatile uint8_t TL740D_Class::RX_buf[64] = { 0 }; //接收数据的缓冲区

DMA_Base_Class TL740D_Class::TX_DMA = DMA_Base_Class(Gyro_TX_DMA_Stream);
DMA_Base_Class TL740D_Class::RX_DMA = DMA_Base_Class(Gyro_RX_DMA_Stream);

uint8_t TL740D_Class::data_Buf[64] = { 0 };

void TL740D_Class::Init(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	IO_Class TX = IO_Class(Gyro_Uart_TX_Port, Gyro_Uart_TX_Pin);
	IO_Class RX = IO_Class(Gyro_Uart_RX_Port, Gyro_Uart_RX_Pin);

	//配置DMA
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_Channel = Gyro_TX_DMA_Channel;
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
	DMA_InitStructure.DMA_BufferSize = 64;
	DMA_InitStructure.DMA_Channel = Gyro_RX_DMA_Channel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&RX_buf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	RX_DMA.Init(&DMA_InitStructure);

	RX_DMA.Open();	//开启DMA接收

	TX.Init(GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP);
	RX.Init(GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP);

	GPIO_PinAFConfig(Gyro_Uart_TX_Port, Gyro_Uart_TX_PinSource, Gyro_Uart_TX_AF);
	GPIO_PinAFConfig(Gyro_Uart_RX_Port, Gyro_Uart_RX_PinSource, Gyro_Uart_RX_AF);

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
	NVIC_InitStructure.NVIC_IRQChannel = Gyro_Uart_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	asm("nop");

	Clear_IDLE_Flag();

	Uart->CR3 |= USART_DMAReq_Tx | USART_DMAReq_Rx; //打开DMA_TX、DMA_RX请求
	//USART_ITConfig(Uart, USART_IT_IDLE, ENABLE);//开启空闲线路中断
	Uart->CR1 |= _BV(4);	//开启空闲中断
}

void TL740D_Class::Forward_Accel_Bias_Init(void)
{
	int cnt = 2000;
	float forward_accel_bias_temp[2000];
	for (int i = 0; i < cnt; i++)
	{
		while (!rx_flag);	//等到数据接收
		Analyze_Data();	//解析数据
		//if (ABS(forward_accel) > 0.02f)
		//{
		//	cnt--;
		//	continue;
		//}
		forward_accel_bias_temp[i] = forward_accel;
	}
	My_Math_Class::HeapSort(forward_accel_bias_temp, cnt);	//排序
	int mid = cnt / 2;

	int min = mid - 499;
	int max = mid + 500;

	float sum = 0.0f;
	for (int i = min; i < max; i++)
	{
		sum += forward_accel_bias_temp[i];
	}
	forward_accel_bias = sum / (max - min + 1);

	//forward_accel_bias = forward_accel_bias_temp[(int)(cnt / 2)];
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
			//z_heading = Coordinate_Class::Transform_Angle(z_heading);

			//forward_accel = forward_accel*9806.65f;	//单位转化为mm/s2
			//z_heading_bias = z_heading;	//设置基准

			return true;
		}

	}
	return false;
}

void TL740D_Class::Read_Data(void)
{
	print((uint8_t *)"\x68\x04\x00\x04\x08", 5);
	if (tx_cnt>0)
	{
		TX_DMA.Set_Data_Num(tx_cnt); //设置要发送的数据数量
		tx_cnt = 0;
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

void TL740D_Class::write(const char c)
{
	if (tx_cnt < 16)
	{
		TX_buf[tx_cnt] = c;
		++tx_cnt;
	}
}

void Gyro_Uart_IRQHandler(void)
{
	if (Gyro_Uart_Port->SR&USART_FLAG_IDLE)	//接收空闲中断
	{
		//软件序列清空中断标志
		uint16_t temp = Gyro_Uart_Port->SR;
		temp = Gyro_Uart_Port->DR;
		TL740D_Class::rx_flag = true;
		TL740D_Class::rx_cnt = 64 - TL740D_Class::RX_DMA.Set_Data_Num(64);
	}
}
