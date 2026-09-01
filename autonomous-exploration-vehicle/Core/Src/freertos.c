/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "moter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Turn calibration test that returns the car close to its starting heading. */
#define MOTER_TEST_TURN_SPEED_PERCENT 45
#define MOTER_TEST_START_DELAY_MS 2000
#define MOTER_TEST_RIGHT_TURN_MS 5000
#define MOTER_TEST_LEFT_TURN_MS 4700
#define MOTER_TEST_STOP_MS 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Debug variables can be read through ST-LINK while the robot is running. */
volatile int32_t g_moter_debug_left_encoder = 0;
volatile int32_t g_moter_debug_right_encoder = 0;
volatile int32_t g_moter_debug_pid_error = 0;
volatile int32_t g_moter_debug_left_delta = 0;
volatile int32_t g_moter_debug_right_delta = 0;
volatile int16_t g_moter_debug_left_pwm_x10 = 0;
volatile int16_t g_moter_debug_right_pwm_x10 = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void MoterTest_RunCommand(MoterCommand command, int16_t percent, uint32_t duration_ms);
static void MoterTest_UpdateDebug(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  Moter_Init();
  Moter_Stop();
  osDelay(MOTER_TEST_START_DELAY_MS);

  /* The same command API will later be called from Raspberry Pi CAN messages. */
  MoterTest_RunCommand(MOTER_COMMAND_TURN_RIGHT,
                       MOTER_TEST_TURN_SPEED_PERCENT,
                       MOTER_TEST_RIGHT_TURN_MS);
  osDelay(MOTER_TEST_STOP_MS);

  MoterTest_RunCommand(MOTER_COMMAND_TURN_LEFT,
                       MOTER_TEST_TURN_SPEED_PERCENT,
                       MOTER_TEST_LEFT_TURN_MS);

  /* Infinite loop */
  for(;;)
  {
    Moter_CommandUpdate();
    MoterTest_UpdateDebug();
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void MoterTest_RunCommand(MoterCommand command, int16_t percent, uint32_t duration_ms)
{
  Moter_CommandStart(command, percent, duration_ms);

  /* Update at 100 ms so timed commands can stop and PID can correct motion. */
  while (Moter_CommandIsRunning() != 0U)
  {
    osDelay(100);
    Moter_CommandUpdate();
    MoterTest_UpdateDebug();
  }
}

static void MoterTest_UpdateDebug(void)
{
  /* Keep the latest motor state visible in RAM for STM32CubeProgrammer reads. */
  g_moter_debug_left_encoder = Moter_GetLeftEncoder();
  g_moter_debug_right_encoder = Moter_GetRightEncoder();
  g_moter_debug_pid_error = Moter_GetStraightPidError();
  g_moter_debug_left_delta = Moter_GetStraightPidLeftDelta();
  g_moter_debug_right_delta = Moter_GetStraightPidRightDelta();
  g_moter_debug_left_pwm_x10 = Moter_GetLeftCommandPercentX10();
  g_moter_debug_right_pwm_x10 = Moter_GetRightCommandPercentX10();
}

/* USER CODE END Application */
