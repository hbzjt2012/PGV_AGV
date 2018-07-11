#pragma once

//陀螺仪用串口
#define Gyro_Uart_Port				USART1
#define Gyro_Uart_TX_Port			GPIOB
#define Gyro_Uart_TX_PinSource		GPIO_PinSource6
#define Gyro_Uart_TX_Pin			((uint16_t)(1<<Gyro_Uart_TX_PinSource))
#define Gyro_Uart_TX_AF				GPIO_AF_USART1
#define Gyro_Uart_RX_Port			GPIOB
#define Gyro_Uart_RX_PinSource		GPIO_PinSource7
#define Gyro_Uart_RX_Pin			((uint16_t)(1<<Gyro_Uart_RX_PinSource))
#define Gyro_Uart_RX_AF				GPIO_AF_USART1
#define Gyro_Uart_IRQn				USART1_IRQn
#define Gyro_Uart_IRQHandler		USART1_IRQHandler

#define Gyro_TX_DMA_Channel			USART1_TX_DMA_Channel
#define Gyro_TX_DMA_Stream			USART1_TX_DMA_Stream
#define Gyro_RX_DMA_Channel			USART1_RX_DMA_Channel
#define Gyro_RX_DMA_Stream			USART1_RX_DMA_Stream

//通信用串口
#define Serial_Uart_Port			USART2
#define Serial_Uart_TX_Port			GPIOD
#define Serial_Uart_TX_PinSource	GPIO_PinSource5
#define Serial_Uart_TX_Pin			((uint16_t)(1<<Serial_Uart_TX_PinSource))
#define Serial_Uart_TX_AF			GPIO_AF_USART2
#define Serial_Uart_RX_Port			GPIOD
#define Serial_Uart_RX_PinSource	GPIO_PinSource6
#define Serial_Uart_RX_Pin			((uint16_t)(1<<Serial_Uart_RX_PinSource))
#define Serial_Uart_RX_AF			GPIO_AF_USART2
#define Wireless_M0_Port			GPIOD
#define Wireless_M0_Pin				GPIO_Pin_4
#define Wireless_M1_Port			GPIOD
#define Wireless_M1_Pin				GPIO_Pin_3
#define Serial_Uart_IRQn			USART2_IRQn
#define Serial_Uart_IRQHandler		USART2_IRQHandler

#define Serial_TX_DMA_Channel			USART2_TX_DMA_Channel
#define Serial_TX_DMA_Stream			USART2_TX_DMA_Stream
#define Serial_RX_DMA_Channel			USART2_RX_DMA_Channel
#define Serial_RX_DMA_Stream			USART2_RX_DMA_Stream


//执行设备用串口
#define Actuator_Uart_Port			USART3
#define Actuator_Uart_TX_Port		GPIOB
#define Actuator_Uart_TX_PinSource	GPIO_PinSource10
#define Actuator_Uart_TX_Pin		((uint16_t)(1<<Actuator_Uart_TX_PinSource))
#define Actuator_Uart_TX_AF			GPIO_AF_USART3
#define Actuator_Uart_RX_Port		GPIOB
#define Actuator_Uart_RX_PinSource	GPIO_PinSource11
#define Actuator_Uart_RX_Pin		((uint16_t)(1<<Actuator_Uart_RX_PinSource))
#define Actuator_Uart_RX_AF			GPIO_AF_USART3
#define Actuator_Uart_IRQn			USART3_IRQn
#define Actuator_Uart_IRQHandler	USART3_IRQHandler

#define Actuator_TX_DMA_Channel			USART3_TX_DMA_Channel
#define Actuator_TX_DMA_Stream			USART3_RX_DMA_Stream
#define Actuator_RX_DMA_Channel			USART3_TX_DMA_Channel
#define Actuator_RX_DMA_Stream			USART3_RX_DMA_Stream

//备用串口
#define Reserve_Uart_Port			UART4
#define Reserve_Uart_TX_Port		GPIOC
#define Reserve_Uart_TX_PinSource	GPIO_PinSource10
#define Reserve_Uart_TX_Pin			((uint16_t)(1<<Reserve_Uart_TX_PinSource))
#define Reserve_Uart_TX_AF			GPIO_AF_UART4
#define Reserve_Uart_RX_Port		GPIOC
#define Reserve_Uart_RX_PinSource	GPIO_PinSource11
#define Reserve_Uart_RX_Pin			((uint16_t)(1<<Reserve_Uart_RX_PinSource))
#define Reserve_Uart_RX_AF			GPIO_AF_UART4
#define Reserve_Uart_IRQn			UART4_IRQn
#define Reserve_Uart_IRQHandler		UART4_IRQHandler

#define Reserve_TX_DMA_Channel			UART4_TX_DMA_Channel
#define Reserve_TX_DMA_Stream			UART4_RX_DMA_Stream
#define Reserve_RX_DMA_Channel			UART4_TX_DMA_Channel
#define Reserve_RX_DMA_Stream			UART4_RX_DMA_Stream

//PGV通信用串口
#define PGV_Uart_Port				UART5
#define PGV_Uart_TX_Port			GPIOC
#define PGV_Uart_TX_PinSource		GPIO_PinSource12
#define PGV_Uart_TX_Pin				((uint16_t)(1<<PGV_Uart_TX_PinSource))
#define PGV_Uart_TX_AF				GPIO_AF_UART5
#define PGV_Uart_RX_Port			GPIOD
#define PGV_Uart_RX_PinSource		GPIO_PinSource2
#define PGV_Uart_RX_Pin				((uint16_t)(1<<PGV_Uart_RX_PinSource))
#define PGV_Uart_RX_AF				GPIO_AF_UART5
#define PGV_Uart_DIR_Port			GPIOD
#define PGV_Uart_DIR_Pin			GPIO_Pin_0
#define PGV_Uart_IRQn				UART5_IRQn
#define PGV_Uart_IRQHandler			UART5_IRQHandler

#define PGV_TX_DMA_Channel				UART5_TX_DMA_Channel
#define PGV_TX_DMA_Stream				UART5_RX_DMA_Stream
#define PGV_RX_DMA_Channel				UART5_TX_DMA_Channel
#define PGV_RX_DMA_Stream				UART5_RX_DMA_Stream

//定义了车轮的默认前进方向
#define FRONT_LEFT_DEFAULT_DIR		false  //前左轮默认方向
#define FRONT_RIGHT_DEFAULT_DIR		true   //前右轮默认方向
#define BEHIND_LEFT_DEFAULT_DIR	false //后左轮默认方向
#define BEHIND_RIGHT_DEFAULT_DIR	true  //后右轮默认方向

//定义了电机和编码器的硬件参数
#define FRONT_LEFT_MOTOR_TIM				TIM9			//定时器9
#define FRONT_LEFT_MOTOR_TIM_CHANNEL		1				//TIM9_CH1
#define FRONT_LEFT_MOTOR_SPEED_Port			GPIOA		
#define FRONT_LEFT_MOTOR_SPEED_PinSource	GPIO_PinSource2
#define FRONT_LEFT_MOTOR_SPEED_Pin			(1<<FRONT_LEFT_MOTOR_SPEED_PinSource)	
#define FRONT_LEFT_MOTOR_SPEED_AF			GPIO_AF_TIM9	//前左轮速度IO
#define FRONT_LEFT_MOTOR_DIR_Port			GPIOC		
#define FRONT_LEFT_MOTOR_DIR_Pin			GPIO_Pin_4		//前左轮方向IO
#define FRONT_LEFT_MOTOR_BRAKE_Port			GPIOC
#define FRONT_LEFT_MOTOR_BRAKE_Pin			GPIO_Pin_5		//前左轮刹车IO
#define FRONT_LEFT_MOTOR_STOP_Port			GPIOB
#define FRONT_LEFT_MOTOR_STOP_Pin			GPIO_Pin_0		//前左轮启动IO
#define FRONT_LEFT_ENCODER_TIM				TIM5			//前左编码器
#define FRONT_LEFT_ENCODER_A_Port			GPIOA
#define FRONT_LEFT_ENCODER_A_PinSource		GPIO_PinSource0
#define FRONT_LEFT_ENCODER_A_Pin			(1<<FRONT_LEFT_ENCODER_A_PinSource)
#define FRONT_LEFT_ENCODER_A_AF				GPIO_AF_TIM5	//前左编码器A相
#define FRONT_LEFT_ENCODER_B_Port			GPIOA
#define FRONT_LEFT_ENCODER_B_PinSource		GPIO_PinSource1
#define FRONT_LEFT_ENCODER_B_Pin			(1<<FRONT_LEFT_ENCODER_B_PinSource)
#define FRONT_LEFT_ENCODER_B_AF				GPIO_AF_TIM5	//前左编码器B相

#define FRONT_RIGHT_MOTOR_TIM				TIM9			//定时器9
#define FRONT_RIGHT_MOTOR_TIM_CHANNEL		2				//TIM9_CH2
#define FRONT_RIGHT_MOTOR_SPEED_Port		GPIOA		
#define FRONT_RIGHT_MOTOR_SPEED_PinSource	GPIO_PinSource3
#define FRONT_RIGHT_MOTOR_SPEED_Pin			(1<<FRONT_RIGHT_MOTOR_SPEED_PinSource)
#define FRONT_RIGHT_MOTOR_SPEED_AF			GPIO_AF_TIM9	//前右轮速度IO
#define FRONT_RIGHT_MOTOR_DIR_Port			GPIOB		
#define FRONT_RIGHT_MOTOR_DIR_Pin			GPIO_Pin_1		//前右轮方向IO
#define FRONT_RIGHT_MOTOR_BRAKE_Port		GPIOB
#define FRONT_RIGHT_MOTOR_BRAKE_Pin			GPIO_Pin_2		//前右轮刹车IO
#define FRONT_RIGHT_MOTOR_STOP_Port			GPIOE
#define FRONT_RIGHT_MOTOR_STOP_Pin			GPIO_Pin_7		//前右轮启动IO
#define FRONT_RIGHT_ENCODER_TIM				TIM3			//前右编码器
#define FRONT_RIGHT_ENCODER_A_Port			GPIOA
#define FRONT_RIGHT_ENCODER_A_PinSource		GPIO_PinSource6
#define FRONT_RIGHT_ENCODER_A_Pin			(1<<FRONT_RIGHT_ENCODER_A_PinSource)
#define FRONT_RIGHT_ENCODER_A_AF			GPIO_AF_TIM3	//前右编码器A相
#define FRONT_RIGHT_ENCODER_B_Port			GPIOA
#define FRONT_RIGHT_ENCODER_B_PinSource		GPIO_PinSource7
#define FRONT_RIGHT_ENCODER_B_Pin			(1<<FRONT_RIGHT_ENCODER_B_PinSource)
#define FRONT_RIGHT_ENCODER_B_AF			GPIO_AF_TIM3	//前右编码器B相

#define BEHIND_LEFT_MOTOR_TIM				TIM12			//定时器12
#define BEHIND_LEFT_MOTOR_TIM_CHANNEL		1				//TIM12_CH1
#define BEHIND_LEFT_MOTOR_SPEED_Port		GPIOB		
#define BEHIND_LEFT_MOTOR_SPEED_PinSource	GPIO_PinSource14
#define BEHIND_LEFT_MOTOR_SPEED_Pin			(1<<BEHIND_LEFT_MOTOR_SPEED_PinSource)
#define BEHIND_LEFT_MOTOR_SPEED_AF			GPIO_AF_TIM12	//前左轮速度IO
#define BEHIND_LEFT_MOTOR_DIR_Port			GPIOE		
#define BEHIND_LEFT_MOTOR_DIR_Pin			GPIO_Pin_15		//后左轮方向IO
#define BEHIND_LEFT_MOTOR_BRAKE_Port		GPIOE
#define BEHIND_LEFT_MOTOR_BRAKE_Pin			GPIO_Pin_14		//后左轮刹车IO
#define BEHIND_LEFT_MOTOR_STOP_Port			GPIOE
#define BEHIND_LEFT_MOTOR_STOP_Pin			GPIO_Pin_13		//后左轮启动IO
#define BEHIND_LEFT_ENCODER_TIM				TIM4			//后左编码器
#define BEHIND_LEFT_ENCODER_A_Port			GPIOD
#define BEHIND_LEFT_ENCODER_A_PinSource		GPIO_PinSource12
#define BEHIND_LEFT_ENCODER_A_Pin			(1<<BEHIND_LEFT_ENCODER_A_PinSource)
#define BEHIND_LEFT_ENCODER_A_AF			GPIO_AF_TIM4	//后左编码器A相
#define BEHIND_LEFT_ENCODER_B_Port			GPIOD
#define BEHIND_LEFT_ENCODER_B_PinSource		GPIO_PinSource13
#define BEHIND_LEFT_ENCODER_B_Pin			(1<<BEHIND_LEFT_ENCODER_B_PinSource)
#define BEHIND_LEFT_ENCODER_B_AF			GPIO_AF_TIM4	//后左编码器B相

#define BEHIND_RIGHT_MOTOR_TIM				TIM12		//定时器12
#define BEHIND_RIGHT_MOTOR_TIM_CHANNEL		2			//TIM12_CH2
#define BEHIND_RIGHT_MOTOR_SPEED_Port		GPIOB		
#define BEHIND_RIGHT_MOTOR_SPEED_PinSource	GPIO_PinSource15
#define BEHIND_RIGHT_MOTOR_SPEED_Pin		(1<<BEHIND_RIGHT_MOTOR_SPEED_PinSource)
#define BEHIND_RIGHT_MOTOR_SPEED_AF			GPIO_AF_TIM12	//前左轮速度IO
#define BEHIND_RIGHT_MOTOR_DIR_Port			GPIOD		
#define BEHIND_RIGHT_MOTOR_DIR_Pin			GPIO_Pin_8//后右轮方向IO
#define BEHIND_RIGHT_MOTOR_BRAKE_Port		GPIOD
#define BEHIND_RIGHT_MOTOR_BRAKE_Pin		GPIO_Pin_9	//后右轮刹车IO
#define BEHIND_RIGHT_MOTOR_STOP_Port		GPIOD
#define BEHIND_RIGHT_MOTOR_STOP_Pin			GPIO_Pin_10	//后右轮启动IO
#define BEHIND_RIGHT_ENCODER_TIM			TIM8		//后右编码器
#define BEHIND_RIGHT_ENCODER_A_Port			GPIOC
#define BEHIND_RIGHT_ENCODER_A_PinSource	GPIO_PinSource6
#define BEHIND_RIGHT_ENCODER_A_Pin			(1<<BEHIND_RIGHT_ENCODER_A_PinSource)
#define BEHIND_RIGHT_ENCODER_A_AF			GPIO_AF_TIM8	//后右编码器A相
#define BEHIND_RIGHT_ENCODER_B_Port			GPIOC
#define BEHIND_RIGHT_ENCODER_B_PinSource	GPIO_PinSource7
#define BEHIND_RIGHT_ENCODER_B_Pin			(1<<BEHIND_RIGHT_ENCODER_B_PinSource)
#define BEHIND_RIGHT_ENCODER_B_AF			GPIO_AF_TIM8	//后右编码器B相

//板载LED指示灯
#define LED_GPIO_Port	GPIOD
#define LED_GPIO_Pin	GPIO_Pin_11

//蜂鸣器控制IO
#define Buzzer_GPIO_Port	GPIOE
#define Buzzer_GPIO_Pin		GPIO_Pin_12

//IIC器件的SDA、SCL GPIO
#define IIC_SDA_GPIO_Port	GPIOC
#define IIC_SDA_GPIO_Pin	GPIO_Pin_9
#define IIC_SCL_GPIO_Port	GPIOC
#define IIC_SCL_GPIO_Pin	GPIO_Pin_8

//看门狗喂狗用IO
#define WTD_GPIO_Port	GPIOC
#define WTD_GPIO_Pin	GPIO_Pin_3

//定义SPI1引脚
#define SPI1_CLK_Port	GPIOB
#define SPI1_CLK_Pin	GPIO_Pin_3
#define SPI1_MISO_Port	GPIOB
#define SPI1_MISO_Pin	GPIO_Pin_4
#define SPI1_MOSI_Port	GPIOB
#define SPI1_MOSI_Pin	GPIO_Pin_5

//定义SPI_Flash芯片
#define SPI_Flash_Chip			W25Q16
#define SPI_Flash_SPI			SPI1
#define SPI_Flash_CS_GPIO_Port	GPIOD
#define SPI_Flash_CS_GPIO_Pin	GPIO_Pin_7


//串口1用DMA
#define USART1_TX_DMA_Channel	DMA_Channel_4
#define USART1_TX_DMA_Stream	DMA2_Stream7
#define USART1_RX_DMA_Channel	DMA_Channel_4
#define USART1_RX_DMA_Stream	DMA2_Stream2

//串口2用DMA
#define USART2_TX_DMA_Channel	DMA_Channel_4
#define USART2_TX_DMA_Stream	DMA1_Stream6
#define USART2_RX_DMA_Channel	DMA_Channel_4
#define USART2_RX_DMA_Stream	DMA1_Stream5

//串口3用DMA
#define USART3_TX_DMA_Channel	DMA_Channel_4
#define USART3_TX_DMA_Stream	DMA1_Stream3
#define USART3_RX_DMA_Channel	DMA_Channel_4
#define USART3_RX_DMA_Stream	DMA1_Stream1

//串口4用DMA
#define UART4_TX_DMA_Channel	DMA_Channel_4
#define UART4_TX_DMA_Stream		DMA1_Stream4
#define UART4_TX_DMA_Channel	DMA_Channel_4
#define UART4_RX_DMA_Stream		DMA1_Stream2

//串口5用DMA
#define UART5_TX_DMA_Channel	DMA_Channel_4
#define UART5_TX_DMA_Stream		DMA1_Stream7
#define UART5_TX_DMA_Channel	DMA_Channel_4
#define UART5_RX_DMA_Stream		DMA1_Stream0

