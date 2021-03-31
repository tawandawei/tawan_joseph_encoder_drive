/* USER CODE BEGIN Header */
/**
******************************************************************************
* File Name          : freertos.c
* Description        : Code for freertos applications
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tawan_joseph_encoder_drive.h"
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
Encoder_Struct encoder1;
DRV_Struct drive1 = {&TIM3->CCR3, &TIM3->CCR4, 1, DIR_STOP, 0, 0};
Controller_PI_Struct controller1 = {1.0,0.28,{0,0},{0,0},1,0,0,0};

/* USER CODE END Variables */
/* Definitions for myTask02_leds */
osThreadId_t myTask02_ledsHandle;
const osThreadAttr_t myTask02_leds_attributes = {
  .name = "myTask02_leds",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask01_motorC */
osThreadId_t myTask01_motorCHandle;
const osThreadAttr_t myTask01_motorC_attributes = {
  .name = "myTask01_motorC",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for myTimer01_motorC */
osTimerId_t myTimer01_motorCHandle;
const osTimerAttr_t myTimer01_motorC_attributes = {
  .name = "myTimer01_motorC"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask02_leds(void *argument);
void StartTask01_motorC(void *argument);
void Callback01_motorC(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
* @brief  FreeRTOS initialization
* @param  None
* @retval None
*/
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END Init */
  
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  
  /* Create the timer(s) */
  /* creation of myTimer01_motorC */
  myTimer01_motorCHandle = osTimerNew(Callback01_motorC, osTimerPeriodic, NULL, &myTimer01_motorC_attributes);
  
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  
  /* Create the thread(s) */
  /* creation of myTask02_leds */
  myTask02_ledsHandle = osThreadNew(StartTask02_leds, NULL, &myTask02_leds_attributes);
  
  /* creation of myTask01_motorC */
  myTask01_motorCHandle = osThreadNew(StartTask01_motorC, NULL, &myTask01_motorC_attributes);
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
  
}

/* USER CODE BEGIN Header_StartTask02_leds */
/**
* @brief  Function implementing the myTask02_leds thread.
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02_leds */
void StartTask02_leds(void *argument)
{
  /* USER CODE BEGIN StartTask02_leds */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1); //must not leave this loop empty
  }
  /* USER CODE END StartTask02_leds */
}

/* USER CODE BEGIN Header_StartTask01_motorC */
/**
* @brief Function implementing the myTask01_motorC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01_motorC */
void StartTask01_motorC(void *argument)
{
  /* USER CODE BEGIN StartTask01_motorC */
  osTimerStart(myTimer01_motorCHandle, 10); // 10 ms = 100 Hz
  //static uint32_t count_delay;
  /* Infinite loop */
  
  for(;;)
  {
    //count_delay++;
    osDelay(10);
  }
  
  /* USER CODE END StartTask01_motorC */
}

/* Callback01_motorC function */
void Callback01_motorC(void *argument)
{
  /* USER CODE BEGIN Callback01_motorC */
  //static uint32_t countX;
  //countX++;
  static float set_speed;
  static float control_speed;
  static float control_duty;
  static uint8_t enable_prev;
  // acquire data from encoder
  encoder1.cnt_current = TIM1->CNT; // update encoder current position
  calculate_encoder_data(&encoder1);
  //check drive enable status
  if(drive1.enable != enable_prev)
  {
    if(drive1.enable)
      drv_set_enable(&drive1, GPIOC, GPIO_PIN_13);
    else
      drv_set_disable(&drive1, GPIOC, GPIO_PIN_13);
    enable_prev = drive1.enable;
  }
  if(drive1.enable)
  {
    // calculate controlled motor speed
    update_controller_PI(&controller1, set_speed, encoder1.w_speed, &control_speed); //update control_speed
    control_duty = ctrlSpeed_to_dutyCycle(control_speed);
    //command controlled speed to the motor
    drv_set_dutycycle(control_duty, &drive1);
  }
    /* USER CODE END Callback01_motorC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
