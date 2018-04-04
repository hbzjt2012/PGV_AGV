#pragma once

#define FLOAT_DELTA 0.001f //当两个浮点数之间的差值小于FLOAT_DELTA，认为两个浮点数一致
#define M_PI 3.14159265358979323846f //圆周率
#define NOP() asm("nop")

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define RANGE(x, a, b) (MIN(MAX(x, a), b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

#define F_EQ(a,b) (ABS(a-b)<FLOAT_DELTA)

#define LEFT_SHIFT(a, n) (a << n)
#define RIGHT_SHIFT(a, n) (a >> n)

#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

// Macros for bit masks
#define _BV(n) (1 << n)
#define SBI(n, b) (n |= _BV(b))
#define CBI(n, b) (n &= ~_BV(b))

// Macros to support option testing
#define _CAT(a, ...) a##__VA_ARGS__
#define CAT_TWO(a, b) _CAT(a, b)

#define SWITCH_ENABLED_false 0
#define SWITCH_ENABLED_true 1
#define SWITCH_ENABLED_0 0
#define SWITCH_ENABLED_1 1
#define SWITCH_ENABLED_ 1
#define ENABLED(b) _CAT(SWITCH_ENABLED_, b)
#define DISABLED(b) (!_CAT(SWITCH_ENABLED_, b))
