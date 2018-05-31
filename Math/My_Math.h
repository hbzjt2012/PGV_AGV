#pragma once
class My_Math_Class
{
public:
	static void HeapSort(float *source, int length);	//使用堆排序对浮点数组进行排序
	static void Swap(float &i, float &j);	//交换两者
private:
	static void Heapify(float *source, int i, int size);  // 从source[i]向下进行堆调整
	static int BuildHeap(float *source, int n);	//建堆
};
