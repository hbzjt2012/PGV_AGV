#pragma once

#include "../HALayer/IO.h"

namespace KEY_Namespace
{
	enum KEY_STATE
	{
		NO_KEY,		//未检测到按键
		CHECK_KEY,  //检测到按键按下
		ENSURE_KEY, //确定按键按下
		RELEASE_KEY //按键松开
	};
}

class Key_Class : public IO_Class
{
public:
	Key_Class() {}
	Key_Class(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) : IO_Class(GPIOx, GPIO_Pin) {}
	~Key_Class() = default;

	void Scan(bool value); //按键扫描，当按键松开时，pb置1
	void Scan(void) { Scan(!Read()); }

	bool pb; //pb=true表示按键按下，pb=false表示按键未按下

private:
	enum KEY_Namespace::KEY_STATE key_state;
};