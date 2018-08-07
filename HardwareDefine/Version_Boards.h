#pragma once
#include "../Configure.h"

#define CHECK_BOARD_VERSION(version) (BOARD_VERSION==BOARD_##version)
#define Interrupt_Function(a)	a

#define BOARD_VERSION_1_0 20171124	//第一版试验版，具体定义待修补
#define BOARD_VERSION_2_0 20180627	//第二版，使用了新电机，重新分配了全部IO

#if CHECK_BOARD_VERSION(VERSION_1_0)
#include "Version_1_0.h"
#elif CHECK_BOARD_VERSION(VERSION_2_0)
#include "Version_2_0.h"
#else
#error Check_Board_Version
#endif
