/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

void CORDIC_Config_Rotation(void);

/* 定义常量，避免除法运算 */
#define RECIPROCAL_PI       0.3183098861837907f     // 1.0f / 3.141592653589793f
#define RECIPROCAL_180      0.0055555555555556f     // 1.0f / 180.0f
#define RECIPROCAL_Q31      4.656612873077393e-10f  // 1.0f / 2147483648.0f

/* CORDIC init function */
void MX_CORDIC_Init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  CORDIC_Config_Rotation();

}

/**
 * @brief 配置CORDIC为旋转模式（计算正弦/余弦）
 * @note  使用Q31格式，禁用缩放因子，6次迭代
 */
void CORDIC_Config_Rotation(void)
{
    // 关键：使用ROTATION模式（Function 0），不是COSINE模式（Function 1）
    // Rotation模式一次计算同时输出cos和sin
    LL_CORDIC_Config(CORDIC, 
                     LL_CORDIC_FUNCTION_COSINE,  // ← 修正：使用ROTATION，不是COSINE
                     LL_CORDIC_PRECISION_6CYCLES,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBREAD_2,           // ← 修正：读取2个结果（cos和sin）
                     LL_CORDIC_NBWRITE_1,          // 写入1个参数（角度）
                     LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);
}

/**
 * @brief 使用CORDIC计算正弦和余弦（替代arm_sin_cos_f32）
 * @param angle_rad 角度（弧度），范围-π到π
 * @param sin_val 输出正弦值（-1.0 ~ 1.0）
 * @param cos_val 输出余弦值（-1.0 ~ 1.0）
 * 
 * @note 严格按照STM32参考手册：先读cos，再读sin
 */
void CORDIC_SinCos_Rad(float angle_rad, float *sin_val, float *cos_val)
{
    // 角度归一化：弧度 -> [-1, 1] 对应 [-π, π]
    // CORDIC输入：arg = angle / π
    float arg = angle_rad * RECIPROCAL_PI;
    
    // 限制到[-1, 1]（防止溢出）
    if (arg > 1.0f) arg -= 2.0f;
    if (arg < -1.0f) arg += 2.0f;
    
    // 转换为Q31格式（-1.0对应0x80000000，+1.0对应0x7FFFFFFF）
    int32_t arg_q31 = (int32_t)(arg * 2147483647.0f);
    
    // 写入角度值到CORDIC
    CORDIC->WDATA = arg_q31;
    
    // 等待计算完成（RRDY标志置位）
    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC));
    
    *cos_val = (float)(int32_t)CORDIC->RDATA * RECIPROCAL_Q31;
    *sin_val = (float)(int32_t)CORDIC->RDATA * RECIPROCAL_Q31;
}

/**
 * @brief 使用CORDIC计算正弦和余弦 - 输入为角度（度）
 * @param angle_deg 角度（度），范围0~360或任意值
 * @param sin_val 输出正弦值
 * @param cos_val 输出余弦值
 */
void CORDIC_SinCos_Deg(float angle_deg, float *sin_val, float *cos_val)
{
    // 度 -> [-1, 1] 对应 [-180°, 180°]
    float arg = angle_deg * RECIPROCAL_180;
    
    // 限制到[-1, 1]
    while (arg > 1.0f) arg -= 2.0f;
    while (arg < -1.0f) arg += 2.0f;
    
    int32_t arg_q31 = (int32_t)(arg * 2147483647.0f);
    
    CORDIC->WDATA = arg_q31;
    
    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC));

    *cos_val = (float)(int32_t)CORDIC->RDATA * RECIPROCAL_Q31;
    *sin_val = (float)(int32_t)CORDIC->RDATA * RECIPROCAL_Q31;
}
