/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.c
  * @brief   This file provides code for the configuration
  *          of the DAC instances.
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
#include "dac.h"

// 定义DAC参考电压和分辨率
#define VREF 3.3f      // 参考电压3.3V
#define DAC_RES 4095.0f // 12位DAC的最大值

/**
  * @brief  设置DAC通道1输出电压
  * @param  voltage: 期望输出电压值 (0-3.3V)
  * @retval None
  */
void set_dac_channel1_voltage(float voltage)
{
    uint32_t dac_value;
    
    // 限制电压范围在0到3.3V之间
    if(voltage > VREF) voltage = VREF;
    if(voltage < 0) voltage = 0;
    
    // 计算对应的DAC数值
    dac_value = (uint32_t)((voltage / VREF) * DAC_RES);
    
    // 设置DAC输出
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, dac_value);
    LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
}

/**
  * @brief  设置DAC通道2输出电压
  * @param  voltage: 期望输出电压值 (0-3.3V)
  * @retval None
  */
void set_dac_channel2_voltage(float voltage)
{
    uint32_t dac_value;
    
    // 限制电压范围在0到3.3V之间
    if(voltage > VREF) voltage = VREF;
    if(voltage < 0) voltage = 0;
    
    // 计算对应的DAC数值
    dac_value = (uint32_t)((voltage / VREF) * DAC_RES);
    
    // 设置DAC输出
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, dac_value);
    LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_2);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
}

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* DAC1 init function */
void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**DAC1 GPIO Configuration
  PA4   ------> DAC1_OUT1
  PA5   ------> DAC1_OUT2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC channel OUT1 config
  */
  LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.TriggerSource2 = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  DAC_InitStruct.OutputConnection = LL_DAC_OUTPUT_CONNECT_GPIO;
  DAC_InitStruct.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_1);

  /** DAC channel OUT2 config
  */
  LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_2, LL_DAC_SIGNED_FORMAT_DISABLE);
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_2, &DAC_InitStruct);
  LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_2);
  LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_2);

  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
