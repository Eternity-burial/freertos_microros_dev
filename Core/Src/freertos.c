/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "usart.h" // 确保包含了 huart5 的头文件

// --- micro-ROS includes ---

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>


// --- ROS Message includes ---
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32.h>

// --- [关键修正] 包含 POSIX 头文件 ---
#include <time.h> // for clock_gettime
#include "unistd.h" // for usleep
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT    GPIOC
#define LED_PIN     GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId_t uartRxSemaphoreHandle;
const osSemaphoreAttr_t uartRxSemaphore_attributes = {
	.name = "uartRxSemaphore"
  };

// [新增] 用于存储从 UART 接收到的单个字符的缓冲区
uint8_t rx_char_buffer[1];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_TEST */
osThreadId_t UART_TESTHandle;
const osThreadAttr_t UART_TEST_attributes = {
  .name = "UART_TEST",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	/* [新增] 创建用于 UART 接收的二进制信号量 */
	uartRxSemaphoreHandle = osSemaphoreNew(1, 0, &uartRxSemaphore_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartTask02, NULL, &LEDTask_attributes);

  /* creation of UART_TEST */
  // UART_TESTHandle = osThreadNew(StartTask03, NULL, &UART_TEST_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	// micro-ROS configuration

	rmw_uros_set_custom_transport(
			true,
			(void *) &huart5,
			cubemx_transport_open,
			cubemx_transport_close,
			cubemx_transport_write,
			cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		while(1){
			HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
			osDelay(200);
		}
	}

	// micro-ROS app


	rcl_allocator_t allocator;
	allocator = rcl_get_default_allocator();

	//create init_options
	rclc_support_t support;
	// char* _argv[] = {NULL};

	volatile rcl_ret_t ret;

	// if (ret != RCL_RET_OK)
	// {
	// 	// 初始化失败，进入死循环并快速闪烁LED报错
	do {
		ret = rclc_support_init(&support, 0, NULL, &allocator);
		if (ret != RCL_RET_OK) {
			osDelay(1000); // 延时 1 秒再试
		}
	} while (ret != RCL_RET_OK);
	// }
	// create node
	// rclc_node_init_default(&node, "cubemx_node", "", &support);
	if (ret != RCL_RET_OK) {
		ret = rclc_support_init(&support, 0, NULL, &allocator);
	}

	rcl_publisher_t publisher;
	std_msgs__msg__Int32 msg;


	rcl_node_t node;
	ret = rclc_node_init_default(&node, "cubemx_node", "", &support);

	// if (ret != RCL_RET_OK)
	// {
	// 	// 初始化失败，进入死循环并快速闪烁LED报错
	// 	// 节点创建失败 (闪2次)
	// 	while(1){
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(700);
	// 	}
	// }
	// create publisher
	ret = rclc_publisher_init_default(
	  &publisher,
	  &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	  "cube_node");
	// if (ret != RCL_RET_OK)
	// {
	// 	// 发布者创建失败 (闪3次)
	// 	while(1){
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(500);
	// 	}
	// }
	msg.data = 0;

	for(;;)
	{
	    ret = rcl_publish(&publisher, &msg, NULL);
		// if (ret != RCL_RET_OK)
		// {
		// 	// 发布失败。在循环中，我们通常不希望系统卡死。
		// 	// 可以在这里做一个短暂的、不那么激进的报错提示，
		// 	// 比如让LED快速闪一下，然后继续尝试。
		// 	// 如果 Agent 断开连接，这里会持续报错。
		// 	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
		// 	osDelay(10);
		// 	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
		// }
		msg.data++;
		osDelay(100);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
  	// 翻转 LED 状态
  	HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  	// 延时 500ms，实现 1Hz 闪烁
  	osDelay(500);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the UART_TEST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
	/* [修改] 整个任务逻辑更新为回显功能 */
	/* USER CODE BEGIN StartTask03 */
	char tx_buffer[100];
	uint32_t heartbeat_counter = 0;

	// 1. 首次启动 UART 中断接收，准备接收 1 个字节到全局的 rx_char_buffer 中
	//    这是一个非阻塞函数，它会立即返回
	HAL_UART_Receive_IT(&huart5, rx_char_buffer, 1);

	/* Infinite loop */
	for(;;)
	{
		// 2. 尝试获取信号量，设置 2000ms (2秒) 的超时
		//    任务将在此处阻塞，直到中断发生或超时
		osStatus_t status = osSemaphoreAcquire(uartRxSemaphoreHandle, 2000);

		if (status == osOK)
		{
			// --- 情况 A: 成功获取信号量 ---
			// 这意味着 HAL_UART_RxCpltCallback 中断被触发，我们收到了一个字符

			// a. 构建回显消息
			sprintf(tx_buffer, "Echo: %c\r\n", rx_char_buffer[0]);

			// b. 将回显消息通过 UART5 发送回去
			HAL_UART_Transmit(&huart5, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

			// c. [关键步骤] 再次开启中断接收，为接收下一个字符做准备
			HAL_UART_Receive_IT(&huart5, rx_char_buffer, 1);
		}
		else if (status == osErrorTimeout)
		{
			// --- 情况 B: 等待超时 ---
			// 这意味着在过去的 2 秒内，我们没有收到任何来自 Linux 的数据

			// 发送一条心跳消息，证明 MCU 仍在正常运行
			sprintf(tx_buffer, "Heartbeat... count: %lu\r\n", heartbeat_counter++);
			HAL_UART_Transmit(&huart5, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
		}
	}
	/* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief Rx Transfer completed callback.
  * @param huart UART handle.
  * @retval None
  */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 	// 检查是否是 UART5 触发的中断
// 	if (huart->Instance == UART5)
// 	{
// 		// 释放信号量，以唤醒正在等待的 StartTask03
// 		osSemaphoreRelease(uartRxSemaphoreHandle);
// 	}
// }
/* USER CODE END Application */

