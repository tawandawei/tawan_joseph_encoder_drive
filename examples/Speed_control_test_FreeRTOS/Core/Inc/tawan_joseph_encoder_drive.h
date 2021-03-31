/**
  ***********************************************************************************
  * @file           : tawan_encoder_drive.h
  * @author         : Tawan Thintawornkul, Joseph Aukamchung
  * @brief          : The Encoder close-loop motor speed control library. Containing
                      the encoder read, calculate rotational speed(rpm) of motor,
                      command and control speed to pwm h-bridge drive na ja!
  ***********************************************************************************
**/

#ifndef TAWAN_JOSEPH_ENCODER_DRIVE_H
#define TAWAN_JOSEPH_ENCODER_DRIVE_H

#include "tim.h" //stm32 hal tim library

/**
  ******************************************************************************
  * @brief          : Hardware selection (pre-processor)
  ******************************************************************************
**/
#define JOSEPH_HARDWARE
//#define TAWAN_HARDWARE

/**
  ******************************************************************************
  * @brief          : Custom math function
  ******************************************************************************
**/
#define ABS(x)  ((x) > 0 ? (x) : -(x))



/**
  ******************************************************************************
  * @brief          : Acquire encoder data and motor speed calculation
  ******************************************************************************
**/
#define CNT_DIR_CW 1
#define CNT_DIR_CCW -1

#define ENC_DT 0.01 // 10 ms (100 Hz)
#define ENC_DSTEP_MAX 30000
#define CTRL_DT 0.01


#ifdef JOSEPH_HARDWARE
#define ENT_CNT_PER_REV 48 // TS-25GA370H-10 Encoder 48 counts/rev
#define MOTOR_GEARRATIO (1/48.0f) // TS-25GA370H-10 1:48 gearbox
#else
#ifdef TAWAN_HARDWARE
#define ENT_CNT_PER_REV 44 // JGA-370 Encoder 44 counts/rev
#define MOTOR_GEARRATIO (1/45.0f) // JGA-370 1:45 gearbox
#else
#define ENT_CNT_PER_REV 44
#define MOTOR_GEARRATIO 1
#endif
#endif

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

void calculate_encoder_data(Encoder_Struct *const e);



/**
  ******************************************************************************
  * @brief          : Input command to H-Bridge PWM driver
  ******************************************************************************
**/

// MOTOR DRIVE
#define DRV_PWM_CCRMAX 3359
#define duty_MIN 14.2 // Joseph A. Experiment: motor stop after adjust pwm down from x % to 14.2%

typedef struct
{
  uint32_t volatile *const tim_pwm_fw; //store address of TIM3->CCR3 at initialize
  uint32_t volatile *const tim_pwm_rw; //store address of TIM3->CCR4 at initialize
  uint8_t enable;
  Dir_1D_Enum dir;
  float duty_prev;
  float pwm_ccr;
} DRV_Struct;

void drv_set_dutycycle(float duty, DRV_Struct *const d);

void drv_set_enable_toggle(DRV_Struct *const d, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void drv_set_enable(DRV_Struct *const d, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void drv_set_disable(DRV_Struct *const d, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
  ******************************************************************************
  * @brief          : Speed I Controller
  ******************************************************************************
**/

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

void update_controller_I(Controller_I_Struct *const c, float Vd, float Vm, float *const Vc); //Vd = set value, Vm = feedback (measured value), Vc = value for controlling as input




/**
  ******************************************************************************
  * @brief          : Speed PI Controller (Joseph A.)
  ******************************************************************************
**/

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

void update_controller_PI(Controller_PI_Struct *const c, float Vd, float Vm, float *const Vc); //Vd = set value, Vm = feedback (measured value), Vc = value for controlling as input
float ctrlSpeed_to_dutyCycle(float Vc);


#endif //TAWAN_JOSEPH_ENCODER_DRIVE_H