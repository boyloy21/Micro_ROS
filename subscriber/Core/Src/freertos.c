/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include "geometry_msgs/msg/twist.h"
#include <geometry_msgs/msg/vector3.h>

#include "usart.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float Red;
float Green;
float Blue;
float led1;
float led2;
float led3;
uint16_t led1_delay;
uint16_t led2_delay;

geometry_msgs__msg__Twist twist_msg;
rcl_subscription_t subscriber;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led1 */
osThreadId_t led1Handle;
const osThreadAttr_t led1_attributes = {
  .name = "led1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led2 */
osThreadId_t led2Handle;
const osThreadAttr_t led2_attributes = {
  .name = "led2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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

float map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void subscription_callback(const void *msgin) {
	const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
	led1= twist_msg->linear.x;
	led2= twist_msg->linear.y;
	led3= twist_msg->angular.z;

	Red=map(led1,0,255,0,65535);
	Green=map(led2,0,255,0,65535);
	Blue=map(led3,0,255,0,65535);

	TIM3->CCR1=Red;
	TIM3->CCR2=Green;
	TIM3->CCR3=Blue;

}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Led1Task(void *argument);
void Led2Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	//Create a Task handle
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	//Create Task lED
	led1Handle = osThreadNew(Led1Task, NULL, &led1_attributes);

	led2Handle = osThreadNew(Led2Task, NULL, &led2_attributes);

  /* USER CODE END Init */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/**
  * @}
  */

/**
  * @}
  */
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
	rmw_uros_set_custom_transport(
			    true,
			    (void *) &huart2,
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
			      printf("Error on default allocators (line %d)\n", __LINE__);
			  }

			  // micro-ROS App //
			  rclc_executor_t executor;
			  // Initialize micro-ROS allocator
			  rcl_allocator_t allocator;
			  allocator = rcl_get_default_allocator();

			  // Initialize support object
			  rclc_support_t support;
			  rclc_support_init(&support, 0, NULL, &allocator);

			  // Create node object
			  rcl_node_t node;
			  rclc_node_init_default(&node, "stm32f446re_node", "", &support);

			  //Create Subscription
			  const char * sub_topic_name = "/cmd_vel";
			  const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
			  rclc_subscription_init_default(
			      &subscriber,
			      &node,
				  sub_type_support,
				  sub_topic_name);

			   // create executor
			  rclc_executor_init(&executor, &support.context, 1, &allocator);
			  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);

			  // Spin executor to receive messages
			  rclc_executor_prepare(&executor);
			  rclc_executor_spin(&executor);

			    for(;;)
			    {

			    	rcl_ret_t ret;
			      ret = rcl_subscription_fini(&subscriber, &node);
			      if (ret != RCL_RET_OK)
			      {
			        printf("Error publishing (line %d)\n", __LINE__);
			      }
			      osDelay(10);
			    }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Led1Task */
/**
* @brief Function implementing the led1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led1Task */
void Led1Task(void *argument)
{
  /* USER CODE BEGIN Led1Task */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END Led1Task */
}

/* USER CODE BEGIN Header_Led2Task */
/**
* @brief Function implementing the Led2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led2Task */
void Led2Task(void *argument)
{
  /* USER CODE BEGIN Led2Task */
  /* Infinite loop */
  for(;;)
  {
	  //TIM3->CCR1= (twist_msg.x)*100 ;

    osDelay(1);
  }
  /* USER CODE END Led2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

