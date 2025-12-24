/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"

// FOC电流采样结构体(原始数据)
typedef struct
{
  q15_t ia;    // A相电流
  q15_t ib;    // B相电流
  q15_t ic;    // C相电流
  q15_t va;    // A相电压
  q15_t vb;    // B相电压
  q15_t vc;    // C相电压
  q15_t vbus;  // 母线电压
} foc_data_t;

// FOC电流转换结构体
typedef struct
{
  float32_t ia;    // A相电流
  float32_t ib;    // B相电流
  float32_t ic;    // C相电流
} foc_data_i;

// FOC转换结构体
typedef struct
{
  float32_t va;    // A相电压
  float32_t vb;    // B相电压
  float32_t vc;    // C相电压
  float32_t vbus;  // 母线电压
  float32_t temp;  // 温度值
} foc_data_v;

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/**
 * @brief 获取原始电流数据
 * @return 原始电流数据结构体指针
 */
foc_data_t* bsp_adc_get_raw_data(void);

/**
 * @brief 获取处理后的电流数据
 * @return 处理后的电流数据结构体指针
 */
foc_data_i* bsp_adc_get_current_data(void);

/**
 * @brief 获取ADC2 5通道原始采样值（对外接口）
 * @return 5通道原始值数组指针
 */
uint16_t* bsp_adc2_get_raw_data(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

