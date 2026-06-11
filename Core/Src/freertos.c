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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId RobotArm_TASKHandle;
osThreadId LED_TASKHandle;
osThreadId Pump_controlHandle;
osThreadId debug_TASKHandle;
osThreadId data_update_TASHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void arm_control_task(void const * argument);
void led_indicate_task(void const * argument);
void pump_control_task(void const * argument);
void usartr_debug_task(void const * argument);
void update_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RobotArm_TASK */
  osThreadDef(RobotArm_TASK, arm_control_task, osPriorityAboveNormal, 0, 1024);
  RobotArm_TASKHandle = osThreadCreate(osThread(RobotArm_TASK), NULL);

  /* definition and creation of LED_TASK */
  osThreadDef(LED_TASK, led_indicate_task, osPriorityIdle, 0, 256);
  LED_TASKHandle = osThreadCreate(osThread(LED_TASK), NULL);

  /* definition and creation of Pump_control */
  osThreadDef(Pump_control, pump_control_task, osPriorityAboveNormal, 0, 512);
  Pump_controlHandle = osThreadCreate(osThread(Pump_control), NULL);

  /* definition and creation of debug_TASK */
  osThreadDef(debug_TASK, usartr_debug_task, osPriorityAboveNormal, 0, 256);
  debug_TASKHandle = osThreadCreate(osThread(debug_TASK), NULL);

  /* definition and creation of data_update_TAS */
  osThreadDef(data_update_TAS, update_task, osPriorityAboveNormal, 0, 256);
  data_update_TASHandle = osThreadCreate(osThread(data_update_TAS), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_arm_control_task */
/**
* @brief Function implementing the RobotArm_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_arm_control_task */
__weak void arm_control_task(void const * argument)
{
  /* USER CODE BEGIN arm_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END arm_control_task */
}

/* USER CODE BEGIN Header_led_indicate_task */
/**
* @brief Function implementing the LED_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_indicate_task */
__weak void led_indicate_task(void const * argument)
{
  /* USER CODE BEGIN led_indicate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_indicate_task */
}

/* USER CODE BEGIN Header_pump_control_task */
/**
* @brief Function implementing the Pump_control_TA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pump_control_task */
__weak void pump_control_task(void const * argument)
{
  /* USER CODE BEGIN pump_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pump_control_task */
}

/* USER CODE BEGIN Header_usartr_debug_task */
/**
* @brief Function implementing the debug_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usartr_debug_task */
__weak void usartr_debug_task(void const * argument)
{
  /* USER CODE BEGIN usartr_debug_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usartr_debug_task */
}

/* USER CODE BEGIN Header_update_task */
/**
* @brief Function implementing the data_update_TAS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_update_task */
__weak void update_task(void const * argument)
{
  /* USER CODE BEGIN update_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END update_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
