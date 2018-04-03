#pragma once
#include <stm32f4xx_gpio.h>

//IO类

class IO_Class
{
public:
	IO_Class(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) : Port(GPIOx), Pin(GPIO_Pin) {}
	IO_Class() = default;
	virtual ~IO_Class() = default;

	void Init(GPIO_InitTypeDef *GPIO_InitStructure);
	void Init(GPIOMode_TypeDef Mode, GPIOOType_TypeDef GPIO_Type = GPIO_OType_PP, GPIOPuPd_TypeDef PuPd = GPIO_PuPd_NOPULL, GPIOSpeed_TypeDef GPIO_Speed = GPIO_Speed_50MHz);
	inline void Set(void) { Port->BSRRL = Pin; }				 //置位
	inline void Clear(void) { Port->BSRRH = Pin; }				 //清零
	inline bool Read(void) { return (bool)((Port->IDR) & Pin); } //返回IO口电平
	inline void Toggle(void) { Port->ODR ^= Pin; }				 //翻转IO口电平
	void Write(bool value) { value ? Set() : Clear(); }			 //写入电平

private:
	GPIO_TypeDef *Port; //GPIO基地址
	uint16_t Pin;
};