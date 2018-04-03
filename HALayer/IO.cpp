#include "IO.h"

void IO_Class::Init(GPIO_InitTypeDef *GPIO_InitStructure)
{
	GPIO_InitStructure->GPIO_Pin = Pin;
	GPIO_Init(Port, GPIO_InitStructure);
}

void IO_Class::Init(GPIOMode_TypeDef Mode, GPIOOType_TypeDef GPIO_Type, GPIOPuPd_TypeDef PuPd, GPIOSpeed_TypeDef GPIO_Speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = Mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_Type;
	GPIO_InitStructure.GPIO_PuPd = PuPd;
	Init(&GPIO_InitStructure);
}
