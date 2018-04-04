#pragma once

/*
* 队列
* 先进先出，后进后出
* 未做溢出、空检测
*/
class Queue_Class
{
  public:
	Queue_Class(int length) : queue_length(length) {}
	~Queue_Class() = default;

	void Init(void);   //初始化队列
	int ENqueue(void); //入队,返回队尾下标
	int DEqueue(void); //出队，返回队头下标

	enum
	{
		BUFFER_EMPTY,	//缓存区空
		BUFFER_HAS_DATA, //缓存区有数据
		BUFFER_FULL		 //缓存区满
	} queue_state;

  private:
	const int queue_length; //队列长度
	int queue_head;			//队头元素下标
	int queue_tail;			//队尾元素下标
};
