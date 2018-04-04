#pragma once

//用于接收处理Gcode指令
class Gcode_Class
{
  public:
	Gcode_Class() = default;
	~Gcode_Class() = default;

	int parse(const char *p, const int agv_add = 1, const int command_next_line = 0); //解析当前指令
	char *Return_Command(void) { return command; }
	char command_letter; // 指令码，如G, M, or I（见指令表格）
	int codenum;		 // 指令号，如123
	int error;			 //报错码
	enum
	{
		NO_PARSE,   //未执行
		IS_PARSING, //正在执行
		IS_PARSED   //执行完毕
	} Parse_State;

  private:
	char command[60]; //当前指令参数，60个字节足够保存指令
};
