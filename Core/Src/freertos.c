/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "ZDT_Stepper.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  osDelay(500);
  uint8_t delay_time = 5;
  ZDT_Stepper_Read_version(1);
  osDelay(delay_time);
  ZDT_Stepper_Read_version(2);
  osDelay(delay_time);
  ZDT_Stepper_Read_version(3);
  osDelay(delay_time);
  ZDT_Stepper_Read_version(4);
  osDelay(delay_time);
  ZDT_Stepper_stop(0, SYNC_DISABLE);

  // ZDT_Stepper_Read_resistance_and_inductance(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_bus_voltage(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_bus_average_current(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_phase_current(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_encoder_raw_value(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_encoder_calibrated_value(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Set_T_position(1, CW, 200, 200, 200, 120, REL_POS_MODE, SYNC_DISABLE);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_target_position(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_current_speed(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_current_position(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_position_error(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_driver_temperature(1);
  // osDelay(delay_time);
  // ZDT_Stepper_Read_motor_status_flags(1);
  // ZDT_Stepper_trigger_encoder_calibration(1);
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // 加速部分
    osDelay(delay_time);

    // ZDT_Stepper_Set_T_position(1, CW, 200, 200, 200, 120, REL_POS_MODE, SYNC_ENABLE);
    // osDelay(delay_time);
    // ZDT_Stepper_Set_T_position(2, CW, 200, 200, 200, 120, REL_POS_MODE, SYNC_ENABLE);
    // osDelay(delay_time);
    // ZDT_Stepper_Set_T_position(3, CW, 200, 200, 200, 120, REL_POS_MODE, SYNC_ENABLE);
    // osDelay(delay_time);
    // ZDT_Stepper_Set_T_position(4, CW, 200, 200, 200, 120, REL_POS_MODE, SYNC_ENABLE);
    // osDelay(1000);
    // ZDT_Stepper_start_sync_motion(0);
    // osDelay(2000);
    ZDT_Stepper_stop(0, SYNC_DISABLE); // 立即停止
    // ZDT_Stepper_Set_Speed(0, CW, 200, 600, SYNC_DISABLE);

    // ZDT_Stepper_Set_Speed(1, CW, 200, 600, SYNC_DISABLE);
    // ZDT_Stepper_torque_control(1, CW, 200, 100, SYNC_DISABLE);

    // ZDT_Stepper_Set_T_position(1, CW, 200, 200, 200, 120, ABS_POS_MODE, SYNC_DISABLE);
    // osDelay(2000);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // ZDT_Stepper_Set_T_position(1, CCW, 200, 200, 200, 360 * 2, ABS_POS_MODE, SYNC_DISABLE);
    // ZDT_Stepper_stop(1, SYNC_DISABLE);
    // ZDT_Stepper_Set_Speed(1, CCW, 200, 600, SYNC_DISABLE);
    // osDelay(2000);
    // printf("Hello from defaultTask\n");
    // osDelay(20);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
