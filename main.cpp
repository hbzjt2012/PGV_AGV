#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "delay.h"


int main()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  delay_init();
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  for (;;)
  {
	  
	  GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_SET);
	  delay_ms(500);
	  GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);
	  delay_ms(500);
  }
}
