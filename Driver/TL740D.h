#pragma once
#include "Gyro.h"
#include "Accelerometer.h"
#include "../HALayer/Uart.h"
#include "../HALayer/DMA.h"
#include "../HALayer/Tim.h"
#include "../App/Position.h"

/*
* 该陀螺转角仪(瑞芬科技的TL740D)使用串口（串口2）和MCU通信
* 可以获取角度、角速度、前向加速度
* 使用DMA发送，使用定时器7超时检测接收数据帧
*/

extern "C" void USART2_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);

class TL740D_Class : public Gyro_Class, public Accelerometer_Class, protected Uart_Base_Class
{
	friend void USART2_IRQHandler(void);
	friend void TIM7_IRQHandler(void);
public:
	TL740D_Class() : Uart_Base_Class(USART2), TX_DMA(DMA1_Stream6) {}
	~TL740D_Class() = default;

	void Init(uint32_t baudrate); //根据波特率初始化串口
	void Forward_Accel_Bias_Init(void);	//用于设定数据偏置
	bool Analyze_Data(void); //解析数据
	void Read_Data(void);	//读传感器数据

	float Return_Forward_Accel(void) { return (forward_accel - forward_accel_bias)*9806.65f; }	//单位转化为mm/s2

	static char *Return_RX_buf(void) { return (char *)RX_buf; }
	static bool Return_rx_flag(void) { return rx_flag; }
	static void Clear_rx_flag(void) { rx_flag = false; }
	static void Clear_rx_cnt(void) { rx_cnt = 0; }

	bool data_OK;

private:
	void Cal_Gyro_Data(void) override {};	//计算陀螺仪数据
	void Cal_Accelerometer_Data(void) override {};	//计算加速度数据

	int32_t BCD2DEC(const uint8_t *source); //将TL740返回的BCD编码的数转换成十进制

	DMA_Base_Class TX_DMA;
	static bool rx_flag;				//表明收到了一帧数据
	static uint16_t tx_cnt;				//发送字节的计数
	static uint16_t rx_cnt;				//接收字节的计数
	static uint8_t TX_buf[16];			//发送数据的缓冲区，若缓冲区满，则不会发送
	static volatile uint8_t RX_buf[32]; //接收数据的缓冲区

	void write(const char c) override;
	static uint8_t data_Buf[24]; //数据暂存，避免数据遭到破坏

};