// --- START OF FILE Core/Inc/allocators.h ---

#ifndef _ALLOCATORS_H_
#define _ALLOCATORS_H_

#include <stdlib.h> // for size_t

// =========================================================================
// [ 关键修正 ]
// 为 custom_memory_manager.c 中定义的、但未在任何头文件中声明的
// FreeRTOS 内存管理函数提供函数原型（声明）。
// 这样，任何包含此头文件的 .c 文件（如 allocators.c）
// 在调用这些函数时，编译器就知道它们的存在和正确的签名。
// =========================================================================
void * pvPortMalloc(size_t xWantedSize);
void   vPortFree(void *pv);
void * pvPortRealloc(void *pv, size_t xWantedSize);
void * pvPortCalloc(size_t num, size_t xWantedSize);
size_t getBlockSize(void *pv);
// =========================================================================


// --- micro-ROS 需要的分配器接口声明 ---
extern int absoluteUsedMemory;
extern int usedMemory;

void * __freertos_allocate(size_t size, void * state);
void   __freertos_deallocate(void * pointer, void * state);
void * __freertos_reallocate(void * pointer, size_t size, void * state);
void * __freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif // _ALLOCATORS_H_

// --- END OF FILE Core/Inc/allocators.h ---