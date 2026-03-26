/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.h
  * @brief   This file contains all the function prototypes for
  *          the cordic.c file
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
#ifndef __CORDIC_H__
#define __CORDIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CORDIC_Init(void);

/**
 * @brief 使用CORDIC计算正弦和余弦（替代arm_sin_cos_f32）- 输入为弧度
 * @param angle_rad 角度（弧度），范围-π到π
 * @param sin_val 输出正弦值（-1.0 ~ 1.0）
 * @param cos_val 输出余弦值（-1.0 ~ 1.0）
 */
inline void CORDIC_SinCos_Rad(float angle_rad, float *sin_val, float *cos_val);

/**
 * @brief 使用CORDIC计算正弦和余弦（替代arm_sin_cos_f32）- 输入为角度
 * @param angle_deg 角度（度），范围0~360
 * @param sin_val 输出正弦值（-1.0 ~ 1.0）
 * @param cos_val 输出余弦值（-1.0 ~ 1.0）
 */
inline void CORDIC_SinCos_Deg(float angle_deg, float *sin_val, float *cos_val);

#ifdef __cplusplus
}
#endif

#endif /* __CORDIC_H__ */

