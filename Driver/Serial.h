#pragma once
#include "../HALayer/Uart.h"
#include "../HALayer/DMA.h"
#include "../macros.h"
#include <stm32f4xx_gpio.h>
#include <misc.h>

/*
使用DMA发送，DMA接收
串口空闲中断
*/


class Serial_Class : public Uart_Base_Class
{
	friend void Serial_Uart_IRQHandler(void);	//串口中断函数，友元申明
public:
	Serial_Class() : Uart_Base_Class(Serial_Uart_Port){}
	virtual ~Serial_Class() = default;

	void Init(uint32_t baudrate); //根据波特率初始化串口
	void flush(void);			  //发送缓存区内的数据

	bool Return_rx_flag(void) { return rx_flag; }
	void Clear_rx_flag(void) { rx_flag = false; }
	uint16_t Return_rx_cnt(void) { return rx_cnt; }
	void Clear_rx_cnt(void) { rx_cnt = 0; }
	char *Return_RX_buf(void) { return (char *)RX_buf; }

protected:
	static DMA_Base_Class TX_DMA;
	static DMA_Base_Class RX_DMA;
	static char TX_buf[1024];		   //发送数据的缓冲区，若缓冲区满，则不会发送
	static volatile char RX_buf[1024]; //接收数据的缓冲区

	static uint16_t tx_cnt; //发送字节的计数
	static uint16_t rx_cnt; //接收字节的计数
	static bool rx_flag;	//接收数据是否完成

	void write(const char c) override;
};
