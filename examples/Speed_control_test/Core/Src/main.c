/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENT_CNT_PER_REV 44

#define CNT_DIR_CW 1
#define CNT_DIR_CCW -1

#define ENC_DT 0.0125 // 12.5 ms (80 Hz)
#define ENC_DSTEP_MAX 30000
#define ENT_CNT_PER_REV 44 // JGA-370 Encoder 44 counts/rev
#define MOTOR_GEARRATIO (1/45.0f) // JGA-370 1:45 gearbox
#define CTRL_DT 0.0125

#define ABS(x)  ((x) > 0 ? (x) : -(x))

typedef enum
{
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
} Dir_1D_Enum;


// ENCODER
typedef struct
{
  uint16_t cnt_current;
  uint16_t cnt_prev;
  Dir_1D_Enum dir;
  int16_t dstep; // limit ENC_DSTEP_MAX < 30000 which (int16_t) = -32767 ~ 32767 so it could store dstep safely without overflow na ja!
  float w_speed;
} Encoder_Struct;

void calculate_encoder_data(Encoder_Struct *const e) //parse struct pointer into function
{
  if(e->cnt_current == e->cnt_prev) // motor stop
  {
    e->dstep = 0; // update dstep
    e->dir = DIR_STOP; // update dir 
    e->w_speed = 0;
  }
  else // motor in running
  {
    if(ABS(e->cnt_prev - ENC_DSTEP_MAX) > ENC_DSTEP_MAX) // overflow case: |current-prev| e.g. rw: |65534-320| or fw: |4000-63200|
    {
      if(e->cnt_current > e->cnt_prev) //reverse
        e->dstep = -1*(e->cnt_prev + (65535 - e->cnt_current));
      else //forward
        e->dstep = (65535 - e->cnt_prev) + e->cnt_current;
    }
    else // general case
      e->dstep = e->cnt_current - e->cnt_prev;
    e->dir = (e->dstep > 0) ? DIR_FORWARD : DIR_REVERSE; // update dir 
    e->cnt_prev = e->cnt_current; // update cnt_prev counter.
    e->w_speed = (float)e->dstep / ENC_DT * 60.0f / ENT_CNT_PER_REV * MOTOR_GEARRATIO;
  }
}

// MOTOR DRIVE
#define DRV_PWM_CCRMAX 3359
float duty_MIN = 14.2; // Joseph A. Experiment: motor stop after adjust pwm down from x % to 14.2%

typedef struct
{
  uint32_t volatile *const tim_pwm_fw; //store address of TIM3->CCR3 at initialize
  uint32_t volatile *const tim_pwm_rw; //store address of TIM3->CCR4 at initialize
  uint8_t enable;
  Dir_1D_Enum dir;
  float duty_prev;
  float pwm_ccr;
  float set_speed_prev;
} DRV_Struct;

void drv_set_dutycycle(float duty, DRV_Struct *const d)
{
  // d->dir = (duty > 0) ? DIR_FORWARD : (duty < 0) ? DIR_REVERSE : DIR_STOP;
  d->dir = (duty > duty_MIN) ? DIR_FORWARD : (duty < -duty_MIN) ? DIR_REVERSE : DIR_STOP; // Joseph A. Experiment: ABS(duty) > ABS(duty_min) = 14.2%// 
  d->pwm_ccr = ABS(duty) * DRV_PWM_CCRMAX / 100.0f;
  switch (d->dir)
  {
  case DIR_FORWARD:
    *d->tim_pwm_fw = (uint32_t)d->pwm_ccr;
    *d->tim_pwm_rw = 0;
    break;
  case DIR_REVERSE:
    *d->tim_pwm_fw = 0;
    *d->tim_pwm_rw = (uint32_t)d->pwm_ccr;
    break;
  case DIR_STOP:
    *d->tim_pwm_fw = 0;
    *d->tim_pwm_rw = 0;
    break;
  }
}


// SPEED I CONTROLLER
//#define SPEED_KP 1
//#define SPEED_KI 1

typedef struct
{
  float kp; //assign as variable instead of #define as pre-compile for on-board adjusting purpose!
  float ki; //assign as variable instead of #define as pre-compile for on-board adjusting purpose!
  float cum_error_I;
  float err; //error
  float acceptable_err_band;
  float cum_error_I_MIN_BOUND; // max reverse I term (-xx.xxx)
  float cum_error_I_MAX_BOUND; // max forward I term (yy.yyyy)
} Controller_I_Struct;

void update_controller_I(Controller_I_Struct *const c, float Vd, float Vm, float *const Vc) //Vd = set value, Vm = feedback (measured value), Vc = value for controlling as input
{
  c->err = Vd - Vm;
  float cum_error_I_now = c->cum_error_I + (c->ki * c->err * CTRL_DT);
  if ((cum_error_I_now < c->cum_error_I_MIN_BOUND) || (cum_error_I_now > c->cum_error_I_MAX_BOUND)) //limit Imax term
    cum_error_I_now = c->cum_error_I;
  else
    c->cum_error_I = cum_error_I_now; //update I error
  
  if (ABS(c->err) >= c->acceptable_err_band) //acceptable error
    *Vc = c->err*c->kp + cum_error_I_now;
}

// PI CONTROL

typedef struct
{
  float kp; //assign as variable instead of #define as pre-compile for on-board adjusting purpose!
  float ki; //assign as variable instead of #define as pre-compile for on-board adjusting purpose!
  float e[2]; //e[0]=e(k-2), e[1]=e(k-1)
  float u[2]; //u[0]=u(k-1), u[1]=u(k)
  uint8_t flag_init;
  float acceptable_err_band;
  float cum_error_I_MIN_BOUND; // max reverse I term (-xx.xxx)
  float cum_error_I_MAX_BOUND; // max forward I term (yy.yyyy)
} Controller_PI_Struct;


void update_controller_PI(Controller_PI_Struct *const c, float Vd, float Vm, float *const Vc) //Vd = set value, Vm = feedback (measured value), Vc = value for controlling as input
{
  if (c->flag_init)
  {
    c->e[0] = Vd;
    c->e[1] = Vd;
    c->u[0] = 0;
    c->flag_init = 0;
  }
  c->u[1] = (c->ki+c->kp)*c->e[1] - c->kp*c->e[0] + c->u[0];
  if (ABS(c->e[1]) >= c->acceptable_err_band) //acceptable error
    *Vc = c->u[1];
  //prepare for next step
  c->e[0] = c->e[1];
  c->u[0] = c->u[1];
  c->e[1] = Vd - Vm;
}


float ctrlSpeed_to_dutyCycle(float Vc)
{
  //float duty = Vc/12*100;
  float duty = Vc/120*100;
  /*float duty = Vc;
  if (duty > 100)
    duty = 100;
  else if(duty < 100)
    duty = -100;*/
  return duty;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Encoder_Struct encoder1;
DRV_Struct drive1 = {&TIM3->CCR3, &TIM3->CCR4, 1, DIR_STOP, 0, 0};
//Controller_I_Struct controller1 = {1.0, 50, 0, 0, 0.1, -120.0, 120.0};
Controller_PI_Struct controller1 = {1.0,0.28,{0,0},{0,0},1,0,0,0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    switch ((uint32_t)htim->Instance) {
    case (uint32_t)TIM6:
      {
        static float set_speed;
        static float control_speed;
        static float control_duty;
        // acquire data from encoder
        encoder1.cnt_current = TIM1->CNT; // update encoder current position
        calculate_encoder_data(&encoder1);
        // control the motor speed
        update_controller_PI(&controller1, set_speed, encoder1.w_speed, &control_speed); //update control_speed
        control_duty = ctrlSpeed_to_dutyCycle(control_speed);
        drv_set_dutycycle(control_duty, &drive1);
        break;
      } // TIM6
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /*static uint16_t speed_sv = 0;
  static uint16_t speed_sv_prev = 0;
  static int8_t speed_dir;*/
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*const uint32_t tick = HAL_GetTick();
    static uint32_t lastTick;
    if(tick - lastTick >= 10) // 20 ms (50 Hz)
    {
      
      
      lastTick = tick;
    }
*/
    /*
    if(speed_sv != speed_sv_prev)
    {
      if (speed_dir == 1)
      {
        TIM3->CCR3 = speed_sv;
        TIM3->CCR4 = 0;
      }
      if (speed_dir == -1)
      {
        TIM3->CCR4 = speed_sv;
        TIM3->CCR3 = 0;
      }
      speed_sv_prev = speed_sv;
    }*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
