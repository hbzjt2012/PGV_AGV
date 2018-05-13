#pragma once

//路径跟随（与时间无关）

#ifndef M_PI
#define M_PI 3.14159265358979323846f //圆周率	
#endif // !M_PI

//定义车体左右前后轮子之间的距离
extern const float Parameter_Define::distance_of_wheel_x_axes; //左右两轮在X轴上的距离为534mm
extern const float Parameter_Define::distance_of_wheel_y_axes; //前后两轮在Y轴上的距离为534mm

//定义车轮直径
extern const float Parameter_Define::Mecanum_wheel_diameter; //定义麦克纳姆轮直径

//定义电机转速
extern float Parameter_Define::motor_max_rotationl_velocity; //电机最高转速3000rpm/min
extern float Parameter_Define::motor_min_rotationl_velocity; //电机最低转速100rpm/min

//定义轮子最大速度、最小速度和最大加速度
extern float Parameter_Define::wheel_max_angular_velocity;	//轮子最大角速度(°/s)
extern float Parameter_Define::wheel_min_angular_velocity;	//轮子最小角速度(°/s)

extern float Parameter_Define::wheel_max_line_velocity;     //轮子最大线速度(mm/s);
extern float Parameter_Define::wheel_min_line_velocity;      //轮子最小线速度(mm/s);

//定义车轮最大加减速时间
extern float Parameter_Define::wheel_acceleration_time;	//车轮最大加减速所需时间(从最低速到最高速)(单位s)
extern float Parameter_Define::wheel_acceleration_line;	//车轮最大线加速度(mm/s2)

extern float Parameter_Define::line_slowest_time = 20.0f;	//最低速移动的时间

//定义了车轮的默认前进方向
#define FRONT_LEFTT_DEFAULT_DIR false  //前左轮默认方向
#define FRONT_RIGHT_DEFAULT_DIR true   //前右轮默认方向
#define BEHIND_LEFTT_DEFAULT_DIR false //后左轮默认方向
#define BEHIND_RIGHT_DEFAULT_DIR true  //后右轮默认方向

//定义了电机和编码器的硬件参数
#define FRONT_LEFT_MOTOR_TIM_CHANNEL 4                        //TIM2_CH4
#define FRONT_LFET_MOTOR_DIR (IO_Class(GPIOE, GPIO_Pin_13))   //前左轮方向IO
#define FRONT_LFET_MOTOR_BRAKE (IO_Class(GPIOE, GPIO_Pin_14)) //前左轮刹车IO
#define FRONT_LFET_MOTOR_STOP (IO_Class(GPIOE, GPIO_Pin_15))  //前左轮启动IO
#define FRONT_LEFT_MOTOR (FRONT_LFET_MOTOR_DIR, FRONT_LFET_MOTOR_BRAKE, FRONT_LFET_MOTOR_STOP, FRONT_LEFTT_DEFAULT_DIR, TIM2)

#define FRONT_RIGHT_MOTOR_TIM_CHANNEL 3                        //TIM2_CH3
#define FRONT_RIGHT_MOTOR_DIR (IO_Class(GPIOE, GPIO_Pin_10))   //前右轮方向IO
#define FRONT_RIGHT_MOTOR_BRAKE (IO_Class(GPIOE, GPIO_Pin_11)) //前右轮刹车IO
#define FRONT_RIGHT_MOTOR_STOP (IO_Class(GPIOE, GPIO_Pin_12))  //前右轮启动IO
#define FRONT_RIGHT_MOTOR (FRONT_RIGHT_MOTOR_DIR, FRONT_RIGHT_MOTOR_BRAKE, FRONT_RIGHT_MOTOR_STOP, FRONT_RIGHT_DEFAULT_DIR, TIM2)

#define BEHIND_LEFT_MOTOR_TIM_CHANNEL 2                        //TIM12_CH2
#define BEHIND_LFET_MOTOR_DIR (IO_Class(GPIOD, GPIO_Pin_11))   //后左轮方向IO
#define BEHIND_LFET_MOTOR_BRAKE (IO_Class(GPIOD, GPIO_Pin_12)) //后左轮刹车IO
#define BEHIND_LFET_MOTOR_STOP (IO_Class(GPIOD, GPIO_Pin_13))  //后左轮启动IO
#define BEHIND_LEFT_MOTOR (BEHIND_LFET_MOTOR_DIR, BEHIND_LFET_MOTOR_BRAKE, BEHIND_LFET_MOTOR_STOP, BEHIND_LEFTT_DEFAULT_DIR, TIM12)

#define BEHIND_RIGHT_MOTOR_TIM_CHANNEL 1                       //TIM12_CH1
#define BEHIND_RIGHT_MOTOR_DIR (IO_Class(GPIOD, GPIO_Pin_8))   //后右轮方向IO
#define BEHIND_RIGHT_MOTOR_BRAKE (IO_Class(GPIOD, GPIO_Pin_9)) //后右轮刹车IO
#define BEHIND_RIGHT_MOTOR_STOP (IO_Class(GPIOD, GPIO_Pin_10)) //后右轮启动IO
#define BEHIND_RIGHT_MOTOR (BEHIND_RIGHT_MOTOR_DIR, BEHIND_RIGHT_MOTOR_BRAKE, BEHIND_RIGHT_MOTOR_STOP, BEHIND_RIGHT_DEFAULT_DIR, TIM12)

#define FRONT_LEFTT_ENCODER (Encoder_Class(TIM1))  //前左编码器
#define FRONT_RIGHT_ENCODER (Encoder_Class(TIM8))  //前右编码器
#define BEHIND_LEFTT_ENCODER (Encoder_Class(TIM3)) //后左编码器
#define BEHIND_RIGHT_ENCODER (Encoder_Class(TIM4)) //后右编码器

//定义了编码器参数
#define ENCODER_FIX_WHEEL true       //指示编码器和轮子固连在一起
#define REDUCTION_RATIO 30           //减速比
#define ENCODER_RESOLUTION_INIT 2000 //编码器的分辨率（线数）
extern const float Parameter_Define::encoder_resolution; 	//轮子最终分辨率（线数）
