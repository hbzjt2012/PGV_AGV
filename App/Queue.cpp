#include "Queue.h"
#include <cstring>

void Queue_Class::Init(void)
{
	queue_head = 0;
	queue_tail = 0;
	queue_state = BUFFER_EMPTY;
}

//************************************
// Method:    ENqueue
// FullName:  Queue_Class::ENqueue
// Access:    public
// Returns:   int
// Parameter: void
// Description:	入队，返回队尾下标
//************************************
int Queue_Class::ENqueue(void)
{
	int index = queue_tail;
	queue_tail = (queue_tail + 1) % queue_length;
	//int queue_index = queue_tail;
	//queue_tail = (queue_tail == (queue_length - 1)) ? 0 : queue_tail + 1;
	queue_state = (queue_head == queue_tail + 1) ? BUFFER_FULL : BUFFER_HAS_DATA;
	return index;
}

//************************************
// Method:    DEqueue
// FullName:  Queue_Class::DEqueue
// Access:    public
// Returns:   int
// Parameter: void
// Description:	出队，返回队头下标
//************************************
int Queue_Class::DEqueue(void)
{
	int index = queue_head;
	queue_head = (queue_head + 1) % queue_length;
	queue_state = (queue_head == queue_tail) ? BUFFER_EMPTY : BUFFER_HAS_DATA;
	//queue_head = (queue_head == queue_length - 1) ? 0 : queue_head + 1;
	return index;
}
