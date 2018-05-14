#include "PGV100.h"
#include <stm32f4xx_gpio.h>
#include <misc.h>

bool PGV_Class::rx_flag = false;			  //表明收到了一帧数据
uint16_t PGV_Class::tx_cnt = 0;				  //发送字节的计数
uint16_t PGV_Class::rx_cnt = 0;				  //接收字节的计数
uint8_t PGV_Class::TX_buf[16] = { 0 };		  //发送数据的缓冲区，若缓冲区满，则不会发送
volatile uint8_t PGV_Class::RX_buf[32] = { 0 }; //接收数据的缓冲区

IO_Class PGV_Class::dir = IO_Class(GPIOD, GPIO_Pin_0);

void PGV_Class::Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//配置TX_DMA
	DMA_InitStructure.DMA_BufferSize = 2;
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
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	TX_DMA.Init(&DMA_InitStructure);

	//配置RX_DMA
	DMA_InitStructure.DMA_BufferSize = 21;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&RX_buf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	RX_DMA.Init(&DMA_InitStructure);

	RX_DMA.ITConfig(DMA_IT_TC, ENABLE); //接收用DMA的完成中断

	//配置串口用的GPIO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		  //GPIOC12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //速度 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);			  //初始化 PA0， PA1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOD2
	GPIO_Init(GPIOD, &GPIO_InitStructure);	//初始化 PA0， PA1

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);  //PD2复用为 UART_RX
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); //PC12 复用为 UART_TX

	//配置485用的方向IO
	dir.Init(GPIO_Mode_OUT);

	//配置串口
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_InitStructure.USART_Parity = USART_Parity_Even;			//偶校验位;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//一个停止位;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;		//字长为8位数据格式;
	Uart_Base_Class::Init(&USART_InitStructure);

	Uart->CR3 |= (USART_DMAReq_Tx | USART_DMAReq_Rx); //打开DMA_TX、DMA_RX请求
	//Uart->CR3 |= USART_DMAReq_Tx;
	Uart->CR1 |= _BV(6); //开启发送完成中断

	//配置中断
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  //响应优先级
	NVIC_Init(&NVIC_InitStructure);

	RX_Dir(); //设置接收

	enable(); //开启串口

	Send(Set_Dir_Best);
}

void PGV_Class::Send(PGV_CMD_Mode _cmd)
{
	TX_Dir();		 //设置发送方向
	cmd_mode = _cmd; //保存指令
	//tx_cnt = 0;	//待发送计数清零
	switch (_cmd)
	{
	case Read_PGV_Data: //读取数据
		print((uint8_t *)"\xC8\x37", 2);
		rx_cnt = 21;
		break;
	case Set_Color_B: //设置色带颜色蓝色
		print((uint8_t *)"\xC4\x7B", 2);
		rx_cnt = 2;
		break;
	case Set_Color_G: //设置色带颜色绿色
		print((uint8_t *)"\x88\x77", 2);
		rx_cnt = 2;
		break;
	case Set_Color_R: //设置色带颜色红色
		print((uint8_t *)"\x90\x6F", 2);
		rx_cnt = 2;
		break;
	case Set_Dir_Best: //设置选择最优的轨道
		print((uint8_t *)"\xEC\x13", 2);
		rx_cnt = 3;
		break;
	case Set_Dir_L: //设置选择左边的轨道
		print((uint8_t *)"\xE8\x17", 2);
		rx_cnt = 3;
		break;
	case Set_Dir_R: //设置选择右边的轨道
		print((uint8_t *)"\xE4\x1B", 2);
		rx_cnt = 3;
		break;
	default:
		break;
	}

	RX_DMA.ITConfig(DMA_IT_TC, DISABLE); //重新设置DMA接收数目前先关闭接受完成中断
	RX_DMA.Set_Data_Num(rx_cnt);		 //设置DMA通道需接受数据的数量
	RX_DMA.ITConfig(DMA_IT_TC, ENABLE);  //开启接收用DMA的完成中断

	flush();
}

/*
*	矩阵变换,x1=ax+b,将传感器在不同工作模式下的坐标系转化为统一标准的坐标系
*
*	Tag标签：X、Y方向由Tag标签定义
*	  1 0 0		  0
*	a=0 1 0		b=0
*     0 0 -1	  0
*
*	位置码带：位置码带沿Y方向粘贴，码带上的一排小字为X正方向
*	  0 -1 0	  0
*	a=1 0 0		b=0
*     0 0 -1	  90
*
*	色带：色带沿Y方向粘贴，X方向由程序处理
*	  0 1 0		  0
*	a=0 0 0		b=0
*     0 0 -1	  90
*
*
* 世界坐标系
* →正，↑正，逆时针正
* PGV读头传感器：安装座为y轴负方向、指示灯一侧为x轴正方向
*
*/
//************************************
// Method:    Analyze_Data
// FullName:  PGV_Class::Analyze_Data
// Access:    public
// Returns:   bool
// Parameter: void
// Description:	解析PGV读头返回的命令，并转到到一个统一的坐标系内
//************************************
bool PGV_Class::Analyze_Data(void)
{
	uint8_t xor_temp = 0;
	uint8_t _target = 0;
	uint32_t data_temp = 0;
	switch (cmd_mode)
	{
	case Read_PGV_Data: //读取数据
		for (int i = 0; i < 20; i++)
		{
			xor_temp ^= RX_buf[i]; //前20个字节的异或校验
		}
		rx_xor_flag = (xor_temp == RX_buf[20]); //检验校验码
		if (rx_xor_flag == false)
			return false; //校验失败

		//校验通过
		warn_flag = (bool)(RX_buf[0] & _BV(2)); //提取错误标志
		warn = RX_buf[18] << 7 | RX_buf[19];	//错误代码

		if ((RX_buf[1] & _BV(6)) == _BV(6)) //检测到Tag标签
		{
			target = Data_Matrix_Tag; //检测到Tag标签
			break;					  //跳出switch
		}

		_target = (RX_buf[0] & _BV(1)) | (RX_buf[1] & _BV(2)); //提取NP位和NL位
		switch (_target)
		{
		case 0:		 //同时识别到位置码带和色带，但以位置码带为准，忽略色带
		case _BV(2): //识别到位置码带
			target = Code_Tape;
			break;
		case _BV(1): //识别到有效色带
			target = Colored_Tape;
			break;
		case (_BV(1) | _BV(2)): //读头没有识别到码带，颜色轨道，tag标签
			target = NO_Target;
			break;
		default:
			break;
		}

		break;
	case Set_Color_B: //设置色带颜色蓝色
		if ((RX_buf[0] == RX_buf[1]) && (RX_buf[0] == 0x01))
			return true;
		else
			return false;
		break;
	case Set_Color_G: //设置色带颜色绿色
		if ((RX_buf[0] == RX_buf[1]) && (RX_buf[0] == 0x02))
			return true;
		else
			return false;
		break;
	case Set_Color_R: //设置色带颜色红色
		if ((RX_buf[0] == RX_buf[1]) && (RX_buf[0] == 0x04))
			return true;
		else
			return false;
		break;
	case Set_Dir_Best: //设置选择最优的轨道
		if ((RX_buf[2] == RX_buf[0] ^ RX_buf[1]) && (RX_buf[1] == 0x03))
			return true;
		else
			return false;
		break;
	case Set_Dir_L: //设置选择左边的轨道
		if ((RX_buf[2] == RX_buf[0] ^ RX_buf[1]) && (RX_buf[1] == 0x02))
			return true;
		else
			return false;
		break;
	case Set_Dir_R: //设置选择右边的轨道
		if ((RX_buf[2] == RX_buf[0] ^ RX_buf[1]) && (RX_buf[1] == 0x01))
			return true;
		else
			return false;
		break;
	default:
		break;
	}

	//提取返回指令中的数据，得到x,y,angle,tag_control_num
	//x偏差
	data_temp = ((RX_buf[2] & 0x07) << 21) + (RX_buf[3] << 14) + (RX_buf[4] << 7) + RX_buf[5];
	x_temp = (data_temp & _BV(23)) ? (int32_t)(-(_BV(24) - data_temp)) : (int32_t)data_temp;
	//if (data_temp&_BV(23))	x_temp = (int32_t)(-(_BV(24) - data_temp));//负数
	//else	x_temp = (int32_t)data_temp;

	//y偏差
	data_temp = (RX_buf[6] << 7) + RX_buf[7];
	y_temp = (data_temp & _BV(13)) ? (int32_t)(-(_BV(14) - data_temp)) : (int32_t)data_temp;
	//if (data_temp&_BV(13))	y_temp = (int32_t)(-(_BV(14) - data_temp));//负数
	//else	y_temp = (int32_t)data_temp;

	//angle偏差
	data_temp = (RX_buf[10] << 7) + RX_buf[11];
	angle_temp = (data_temp & _BV(13)) ? (int32_t)(-(_BV(14) - data_temp)) : (int32_t)data_temp;
	//if (data_temp&_BV(13))	angle_temp = (int32_t)(-(_BV(14) - data_temp));//负数
	//else	angle_temp = (int32_t)data_temp;

	//tag_control_num Tag标签号或者控制码
	data_temp = ((RX_buf[14]) << 21) + (RX_buf[15] << 14) + (RX_buf[16] << 7) + RX_buf[17];
	tag_control_num = (RX_buf[0] & _BV(3)) ? ((data_temp >> 14) & 0x3FFF) : data_temp;
	//if (RX_buf[0] & _BV(3))	//提取控制码标志CC1
	//{
	//	//读取到的是控制码
	//	tag_control_num = (data_temp >> 14) & 0x3FFF;	//保留低10位
	//}
	//else
	//{
	//	tag_control_num = data_temp;	//读取到的是Tag标签号
	//}

	switch (target)
	{
	case Colored_Tape:
		x_deviation = y_temp / 10.0;
		y_deviation = 0.0;
		angle_deviation = ((-angle_temp) + 900) / 10.0;
		break;
	case Code_Tape:
		x_deviation = (-y_temp) / 10.0;
		y_deviation = x_temp / 10.0;
		angle_deviation = ((-angle_temp) + 900) / 10.0;
		break;
	case Data_Matrix_Tag:
		x_deviation = x_temp / 10.0;
		y_deviation = y_temp / 10.0;
		angle_deviation = (-angle_temp) / 10.0;
		break;
	default:
		break;
	}
	if (angle_deviation < 0.0)
	{
		angle_deviation += 360.0;
	}
	return rx_xor_flag;
}

Coordinate_Class PGV_Class::Cal_Coor(void)
{
	coor.x_coor = (tag_control_num % 4) * 600.0f + x_deviation;	//测试用间隔为60cm
	coor.y_coor = (tag_control_num / 4) * 600.0f + y_deviation;	//测试用间隔为60cm
	coor.angle_coor = angle_deviation;	//标签的角度和世界坐标系的角度重合，故直接赋值

	coor.Truncation_Coor();	
}

inline void PGV_Class::TX_Dir(void)
{
	dir.Set();
}

inline void PGV_Class::RX_Dir(void)
{
	dir.Clear();
}

void PGV_Class::write(const char c)
{
	if (tx_cnt < 16)
	{
		TX_buf[tx_cnt] = c;
		++tx_cnt;
	}
}

void PGV_Class::flush(void)
{
	TX_DMA.Set_Data_Num(tx_cnt); //设置要发送的数据数量
	tx_cnt = 0;
}

void UART5_IRQHandler(void)
{
	if (UART5->SR & (1 << 6)) //发送完成中断
	{
		UART5->SR &= ~(1 << 6); //清除TC位
		PGV_Class::RX_Dir();	//设置方向为接收
	}
}

void DMA1_Stream0_IRQHandler(void)
{
	if (DMA1->LISR & _BV(5)) //传输完成中断
	{
		DMA1->LIFCR = _BV(5); //清除标志位
		PGV_Class::rx_flag = true;
	}
}