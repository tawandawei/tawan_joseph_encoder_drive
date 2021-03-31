/**
  ***********************************************************************************
  * @file           : tawan_joseph_encoder_drive.c
  * @author         : Tawan Thintawornkul (tawandawei@hotmail.com), Joseph Aukamchung
  * @brief          : The Encoder close-loop motor speed control library. Containing
                      the encoder read, calculate rotational speed(rpm) of motor,
                      command and control speed to pwm h-bridge drive na ja!
  ***********************************************************************************
**/

#include "tawan_joseph_encoder_drive.h"

/**
  ******************************************************************************
  * @brief          : Acquire encoder data and motor speed calculation
  ******************************************************************************
**/

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


/**
  ******************************************************************************
  * @brief          : Input command to H-Bridge PWM driver
  ******************************************************************************
**/

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

void drv_set_enable_toggle(DRV_Struct *const d, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  d->enable = !d->enable;
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, (GPIO_PinState)d->enable);
}

void drv_set_enable(DRV_Struct *const d, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void drv_set_disable(DRV_Struct *const d, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}


/**
  ******************************************************************************
  * @brief          : Speed I Controller
  ******************************************************************************
**/

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



/**
  ******************************************************************************
  * @brief          : Speed PI Controller (Joseph A.)
  ******************************************************************************
**/

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