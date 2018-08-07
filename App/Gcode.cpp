#include "Gcode.h"
#include <cctype>

//************************************
// Method:    Parse
// FullName:  Gcode_Class::Parse
// Access:    public
// Returns:   int
//					解析正常，返回0;
//					出错，返回出错行号;
//					AGV地址错误，返回-1
// Parameter: const char * p
// Parameter: const int agv_add	AGV地址码
// Parameter: const int command_next_line 期望的下一条行号，若为0，则忽略行号检查
// Description:	解析Gcode指令
//************************************
int Gcode_Class::Parse(const char *p, const unsigned int agv_add, const int command_next_line)
{
	codenum = 0;
	char crc_command = 0; //从指令中获取的crc校验
	char crc_current = 0; //计算得出的crc校验

	while (*p < '*')
		++p;				//跳过不可打印字符
	if (toupper(*p) == 'N') //开头检测到'N'-行号
	{
		int command_line = 0;
		crc_current ^= *p;
		++p;
		do
		{
			crc_current ^= *p;
			command_line *= 10;
			command_line += *p++ - '0';
		} while ((*p >= '0') && (*p <= '9'));

		if ((command_next_line) && (command_line != command_next_line))
		{
			error = command_line;
			return error; //行号出错，返回期望行号
		}
	}

	while (*p < '*')
		++p;				//跳过不可打印字符
	if (toupper(*p) == 'A') //开头检测到'A'-AGV地址号
	{
		int command_agv_add = 0;
		crc_current ^= *p;
		++p;
		do
		{
			crc_current ^= *p;
			command_agv_add *= 10;
			command_agv_add += *p++ - '0';
		} while ((*p >= '0') && (*p <= '9')); //获取AGV地址

		if (command_agv_add != agv_add)
		{
			return -1; //AGV地址出错，返回-1
		}
	}

	while (*p < '*')
		++p; //跳过不可打印字符

	crc_current ^= *p;
	command_letter = *p; //保存当前指令码

	if ((command_letter != 'G') && (command_letter != 'M') && (command_letter != 'I'))
	{
		error = command_next_line;
		return error; //指令码出错
	}

	++p;
	do
	{
		crc_current ^= *p;
		codenum *= 10;
		codenum += *p++ - '0';
	} while ((*p >= '0') && (*p <= '9')); //保存指令号

	while (*p < '*')
		++p; //跳过不可打印字符

	int i = 0;
	for (i = 0; i < 59; i++)
	{
		while ((*p < '*') && (*p > 0))
			++p; //跳过不可打印字符
		if (*p == 0 || *p == '*' || *p == ';')
			break; //检测到末尾、校验指令或者注释

		crc_current ^= *p;

		command[i] = toupper(*p); //保存
		++p;
	}
	command[i] = 0; //保存指令

	if (*p == '*') //检测到校验指令
	{
		++p;
		do
		{
			crc_command *= 10;
			crc_command += *p++ - '0';
		} while ((*p >= '0') && (*p <= '9')); //保存crc校验

		if (crc_command != crc_current)
		{
			error = command_next_line;
			return error; //CRC校验出错，返回出错行号
		}
	}
	return 0;
}
