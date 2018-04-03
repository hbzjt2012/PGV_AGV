#pragma once
#ifndef _DELAY_H
#define _DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "misc.h"

void delay_init(void);
void delay_ms(uint16_t nms);      //延时ms，短延时
void delay_us(uint32_t nus);      //延时us
void delay_ms_long(uint16_t nms); //延时ms，长延时，多次调用短延时

#ifdef __cplusplus
}
#endif

#endif