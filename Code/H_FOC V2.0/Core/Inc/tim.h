/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief 初始化定时器
 */
void MX_TIM8_Init(void);

/**
 * @brief 初始化PWM输出
 */
void bsp_pwm_start(void);

/**
 * @brief 停止PWM输出
 */
void bsp_pwm_stop(void);

/**
 * @brief 设置三相PWM占空比
 * @param duty_u U相比较值
 * @param duty_v V相比较值
 * @param duty_w W相比较值
 */
void bsp_pwm_set_duty_three_phase(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

