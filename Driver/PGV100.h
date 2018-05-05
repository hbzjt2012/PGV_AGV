#pragma once
#include "../HALayer/Uart.h"
#include "../HALayer/DMA.h"
#include "../HALayer/IO.h"
#include "../macros.h"
#include "./App/Position.h"

/*
* PGV传感器地址为0
* PGV传感器发送和接收数据都是以16进制格式收发
* PGV使用UART5，485通信
* 使用DMA发送（开启串口发送完成中断TC，用于设置485为接收状态）
* 使用DMA接收(开启传输完成中断，用于指示一帧数据接收完成)
* 未对警告信息进行处理
*
* 统一了色带、位置码带、Tag标签三者的坐标系（矩阵变换）
* 默认三者的坐标系与世界坐标系没有相对旋转、平移
* 以地面为参考系，PGV读头的相对位置 →x正 ↑正，逆时针正
*
* 安装座为y轴负方向
* 指示灯一侧为x轴正方向
* 逆时针为正
*
* 若色带、位置码带、Tag标签与世界坐标系之间有相对旋转、平移，需自行增加坐标变换
*/

extern "C" void UART5_IRQHandler(void);
extern "C" void DMA1_Stream0_IRQHandler(void);

class PGV_Class : private Uart_Base_Class
{
	friend void UART5_IRQHandler(void);
	friend void DMA1_Stream0_IRQHandler(void);

  public:
	//PGV传感器的指令

	typedef enum {
		Set_Dir_R,	 //选择右边颜色轨道
		Set_Dir_L,	 //选择左边颜色轨道
		Set_Dir_Best,  //选择对比度最好的颜色轨道
		Read_PGV_Data, //读PGV数据
		Set_Color_B,   //选择蓝色轨道
		Set_Color_R,   //选择红色轨道
		Set_Color_G	//选择绿色轨道
	} PGV_CMD_Mode;

	//对于传感器来说，优先级：Tag标签=位置码带>色带
	//软件上优先检查Tag标签，对Tag标签和位置码带不做区分，因此张贴时需注意
	typedef enum {
		Colored_Tape,	//色带
		Code_Tape,		 //位置码带
		Data_Matrix_Tag, //Tag标签
		NO_Target		 //没有识别到码带、色带、Tag标签
	} PGV_TARGET;		 //指示当前PGV传感器读取到的目标

	PGV_Class() : Uart_Base_Class(UART5), TX_DMA(DMA1_Stream7), RX_DMA(DMA1_Stream0) {}
	~PGV_Class() = default;
	void Init(uint32_t baudrate);
	void Send(PGV_CMD_Mode _cmd);
	bool Return_rx_flag(void) { return rx_flag; }
	void Clear_rx_flag(void) { rx_flag = false; }
	bool Analyze_Data(void);						   //解析数据
	inline PGV_CMD_Mode cmd(void) { return cmd_mode; } //返回上一次发送的指令


	Position_Class::Coordinate_Class&Cal_Coor(void);	//该函数需修改，当前为测试用

	//从读取的数据中获得的x,y,z偏差数值
	float x_deviation;		  //x偏差
	float y_deviation;		  //y偏差
	float angle_deviation;	//角度偏差,取值为0~360
	uint32_t tag_control_num; //存放Tag标签号或者控制码
	PGV_TARGET target;		  //指示当前读头读取到的目标种类

	

	bool warn_flag; //警告标志
	uint16_t warn;

  private:
	DMA_Base_Class TX_DMA;
	DMA_Base_Class RX_DMA;
	static IO_Class dir;

	/*struct
	* {
	*	uint8_t PGV_state[2];		//读头状态
	*	uint8_t x_date[4];			//x偏差数据
	*	uint8_t y_data[2];			//y偏差数据
	*	uint8_t reserve9_10[2];		//保留
	*	uint8_t angle_data[2];		//角度偏差数据
	*	uint8_t reserve13_14[2];	//保留
	*	uint8_t code[4];			//控制码和Tag标签号
	*	uint8_t warning[2];			//报警代码
	*	uint8_t xor;				//校验
	* }RX_data;
	*
	* 解析后获得的只经过正负处理过的偏移值
	*/

	static bool rx_flag;				//表明收到了一帧数据
	static uint16_t tx_cnt;				//发送字节的计数
	static uint16_t rx_cnt;				//接收字节的计数
	static uint8_t TX_buf[16];			//发送数据的缓冲区，若缓冲区满，则不会发送
	static volatile uint8_t RX_buf[32]; //接收数据的缓冲区
	//解析后获得的只经过正负处理过的偏移值
	int32_t x_temp;
	int32_t y_temp;
	int32_t angle_temp;
	bool rx_xor_flag;	  //校验标志
	PGV_CMD_Mode cmd_mode; //发送的指令

	Position_Class::Coordinate_Class coor;	//实际坐标

	static void TX_Dir(void); //设置485方向为发送
	static void RX_Dir(void); //设置485方向为接收
	void write(const char c) override;
	void flush(void); //发送缓存区内的数据
};