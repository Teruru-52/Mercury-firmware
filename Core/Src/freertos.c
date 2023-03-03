/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include "main_exec.h"
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
osThreadId ControlTaskHandle;
osThreadId SpeakerTaskHandle;
osThreadId IRsensorTaskHandle;
osSemaphoreId ControlSemaphoreHandle;
osSemaphoreId SpeakerSemaphoreHandle;
osSemaphoreId IRsensorSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartControlTask(void const *argument);
void StartSpeakerTask(void const *argument);
void StartIRsensorTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ControlSemaphore */
  osSemaphoreDef(ControlSemaphore);
  ControlSemaphoreHandle = osSemaphoreCreate(osSemaphore(ControlSemaphore), 1);

  /* definition and creation of SpeakerSemaphore */
  osSemaphoreDef(SpeakerSemaphore);
  SpeakerSemaphoreHandle = osSemaphoreCreate(osSemaphore(SpeakerSemaphore), 1);

  /* definition and creation of IRsensorSemaphore */
  osSemaphoreDef(IRsensorSemaphore);
  IRsensorSemaphoreHandle = osSemaphoreCreate(osSemaphore(IRsensorSemaphore), 1);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityNormal, 0, 128);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SpeakerTask */
  osThreadDef(SpeakerTask, StartSpeakerTask, osPriorityNormal, 0, 128);
  SpeakerTaskHandle = osThreadCreate(osThread(SpeakerTask), NULL);

  /* definition and creation of IRsensorTask */
  osThreadDef(IRsensorTask, StartIRsensorTask, osPriorityNormal, 0, 128);
  IRsensorTaskHandle = osThreadCreate(osThread(IRsensorTask), NULL);

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
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  StartupProcess();
  /* Infinite loop */
  for (;;)
  {
    // StateProcess();
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the ControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const *argument)
{
  /* USER CODE BEGIN StartControlTask */
  HAL_TIM_Base_Start_IT(&htim7);
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(ControlSemaphoreHandle, osWaitForever);
    UpdateUndercarriage();
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartSpeakerTask */
/**
 * @brief Function implementing the SpeakerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSpeakerTask */
void StartSpeakerTask(void const *argument)
{
  /* USER CODE BEGIN StartSpeakerTask */
  HAL_TIM_Base_Start_IT(&htim13);
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(SpeakerSemaphoreHandle, osWaitForever);
    Notification();
  }
  /* USER CODE END StartSpeakerTask */
}

/* USER CODE BEGIN Header_StartIRsensorTask */
/**
 * @brief Function implementing the IRsensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIRsensorTask */
void StartIRsensorTask(void const *argument)
{
  /* USER CODE BEGIN StartIRsensorTask */
  HAL_TIM_Base_Start_IT(&htim1);
  /* Infinite loop */
  for (;;)
  {
    osSemaphoreWait(IRsensorSemaphoreHandle, osWaitForever);
    UpdateIRsensor();
  }
  /* USER CODE END StartIRsensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
