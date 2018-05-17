#pragma once
#include "../HALayer/Uart.h"
#include "../HALayer/DMA.h"
#include "../HALayer/Tim.h"

/*
* 该串口（串口2）用于和陀螺仪通信
* 通过陀螺仪获取角度、角速度、前向加速度
* 使用DMA发送，使用定时器7超时检测接收数据帧
*/

extern "C" void USART2_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);

class Gyro_Class : protected Uart_Base_Class
{
	friend void USART2_IRQHandler(void);
	friend void TIM7_IRQHandler(void);

public:
	Gyro_Class() : Uart_Base_Class(USART2), TX_DMA(DMA1_Stream6), data_OK(false) {}
	virtual ~Gyro_Class() = default;

	void Init(uint32_t baudrate); //根据波特率初始化串口

	virtual void Read_Data(void) = 0;	//读传感器数据
	virtual bool Analyze_Data(void) = 0; //解析数据
	bool Return_rx_flag(void) { return rx_flag; }
	void Clear_rx_flag(void) { rx_flag = false; }
	void Clear_rx_cnt(void) { rx_cnt = 0; }

	float z_rate;		 //Z轴角速率(rad/s)
	float forward_accel; //前向加速度
	float z_heading;	 //Z轴方位角，0°~360°

	float z_rate_bias;	//Z轴角速度偏置(rad/s)

	bool data_OK;

protected:
	DMA_Base_Class TX_DMA;
	static bool rx_flag;				//表明收到了一帧数据
	static uint16_t tx_cnt;				//发送字节的计数
	static uint16_t rx_cnt;				//接收字节的计数
	static uint8_t TX_buf[16];			//发送数据的缓冲区，若缓冲区满，则不会发送
	static volatile uint8_t RX_buf[32]; //接收数据的缓冲区

	void write(const char c) override;
	static uint8_t data_Buf[16]; //数据暂存，避免数据遭到破坏
};