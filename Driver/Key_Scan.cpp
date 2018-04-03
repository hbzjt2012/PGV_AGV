#include "Key_Scan.h"

using namespace KEY_Namespace;

//************************************
// Method:    Scan
// FullName:  Key_Class::Scan
// Access:    public
// Returns:   void
// Parameter: bool value	1-检测到按键按下
// Description:	按键扫描程序
//************************************
void Key_Class::Scan(bool value)
{
	switch (key_state)
	{
	case NO_KEY:
		if (value)
		{
			key_state = CHECK_KEY;
		}
		break;
	case CHECK_KEY:
		key_state = value ? ENSURE_KEY : NO_KEY;
		break;
	case ENSURE_KEY:
		if (!value)
			key_state = RELEASE_KEY; //检测到按键松开
		break;
	case RELEASE_KEY:
		if (value)
			key_state = ENSURE_KEY;
		else
		{
			pb = true;
			key_state = NO_KEY; //按键未按下
		}
		break;
	default:
		break;
	}
}
