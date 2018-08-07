#include "My_Math.h"

void My_Math_Class::HeapSort(float * source, int length)
{
	int heap_size = BuildHeap(source, length);    // 建立一个最大堆
	while (heap_size > 1)    // 堆（无序区）元素个数大于1，未完成排序
	{
		// 将堆顶元素与堆的最后一个元素互换，并从堆中去掉最后一个元素
		// 此处交换操作很有可能把后面元素的稳定性打乱，所以堆排序是不稳定的排序算法
		Swap(source[0], source[--heap_size]);
		Heapify(source, 0, heap_size);     // 从新的堆顶元素开始向下进行堆调整，时间复杂度O(logn)
	}
}

void My_Math_Class::Swap(float & i, float & j)
{
	float temp = i;
	i = j;
	j = temp;
}

//进行堆调整，从小到大排
void My_Math_Class::Heapify(float * source, int i, int size)
{
	int left_child = 2 * i + 1;         // 左孩子索引
	int right_child = 2 * i + 2;        // 右孩子索引
	int max = i;                        // 选出当前结点与其左右孩子三者之中的最大值
	if (left_child < size && source[left_child] > source[max])
		max = left_child;
	if (right_child < size && source[right_child] > source[max])
		max = right_child;
	if (max != i)
	{
		Swap(source[i], source[max]);                // 把当前结点和它的最大(直接)子节点进行交换
		Heapify(source, max, size);          // 递归调用，继续从当前结点向下进行堆调整
	}
}

int My_Math_Class::BuildHeap(float * source, int n)
{
	int heap_size = n;
	for (int i = heap_size / 2 - 1; i >= 0; i--) // 从每一个非叶结点开始向下进行堆调整
		Heapify(source, i, heap_size);
	return heap_size;
}
