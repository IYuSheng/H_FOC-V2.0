/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "foc_prase.h"

#define UART_TX_BUFFER_SIZE 512
#define UART_RX_BUFFER_SIZE 64

static char d_buffer[UART_TX_BUFFER_SIZE];
static char rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_complete = 0;

// 声明一个回调函数指针，用于处理接收到的数据
void (*uart_receive_callback)(char* data, uint8_t len) = NULL;

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 500000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART3);
  LL_USART_ConfigAsyncMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while(!(LL_USART_IsActiveFlag_TEACK(USART3)))
  {
  }

  // 设置中断优先级
  NVIC_SetPriority(USART3_IRQn, 4); // 使用较低的优先级值
  NVIC_EnableIRQ(USART3_IRQn);

  // 使能接收中断
  LL_USART_EnableIT_RXNE(USART3);
  LL_USART_EnableIT_ERROR(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
 * @brief UART3发送一个字节（阻塞方式）
 * @param data 要发送的数据
 */
static void bsp_uart_send_byte(uint8_t data)
{
  // 带超时的等待（避免硬件异常导致永久阻塞）
  uint32_t timeout = 0xFFFF;
  while ((!(LL_USART_IsActiveFlag_TXE(USART3))) && (timeout-- > 0));
  if (timeout > 0)
    {
      LL_USART_TransmitData8(USART3, data);
    }
}

/**
 * @brief UART3发送字符串（阻塞方式）
 * @param str 要发送的字符串
 */
void bsp_uart_send_string(char *str)
{
  while (*str)
    {
      bsp_uart_send_byte(*str++);
    }
}

/**
 * @brief UART3发送缓冲区数据（阻塞方式）
 * @param buffer 数据缓冲区
 * @param len 数据长度
 */
static inline void bsp_uart_send_buffer(uint8_t *buffer, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len; i++)
    {
      bsp_uart_send_byte(buffer[i]);
    }
}

/**
 * @brief 格式化打印调试信息
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void debug_log(const char *format, ...)
{
  va_list args;
  int len;

  // 格式化字符串
  va_start(args, format);
  len = vsnprintf(d_buffer, sizeof(d_buffer), format, args);
  va_end(args);

  // 确保不会超出缓冲区范围
  if (len > sizeof(d_buffer) - 3)
    {
      len = sizeof(d_buffer) - 3;
    }

  // 添加\r\n确保换行
  if (len > 0)
    {
      d_buffer[len++] = '\r';
      d_buffer[len++] = '\n';
      d_buffer[len] = '\0';
    }

  if (len > 0)
    {
      bsp_uart_send_buffer((uint8_t*)d_buffer, (uint16_t)len);
    }
}

/**
 * @brief 串口3中断处理函数
 */
void USART3_IRQHandler(void)
{
    // 检查是否是接收中断
    if (LL_USART_IsActiveFlag_RXNE(USART3))
    {
        // 读取接收数据
        char received_char = LL_USART_ReceiveData8(USART3);
        
        if (received_char == '\n')
        {
            if (rx_index > 0) // 如果有数据
            {
                // 添加字符串结束符
                rx_buffer[rx_index] = '\0';
                
                // 解析接收到的数据
                parse_received_data(rx_buffer, rx_index);
                
                // 重置接收索引
                rx_index = 0;
            }
        }
        else
        {
            // 将数据存入缓冲区
            if (rx_index < (UART_RX_BUFFER_SIZE - 1))
            {
                rx_buffer[rx_index++] = received_char;
            }
            else
            {
                // 缓冲区满，重置
                rx_index = 0;
            }
        }
    }
    // 检查错误标志
    else if (LL_USART_IsActiveFlag_PE(USART3) || 
             LL_USART_IsActiveFlag_FE(USART3) || 
             LL_USART_IsActiveFlag_NE(USART3) || 
             LL_USART_IsActiveFlag_ORE(USART3))
    {
        // 清除错误标志
        if (LL_USART_IsEnabledIT_PE(USART3)) LL_USART_ClearFlag_PE(USART3);
        if (LL_USART_IsEnabledIT_ERROR(USART3)) LL_USART_ClearFlag_FE(USART3);
        if (LL_USART_IsEnabledIT_ERROR(USART3)) LL_USART_ClearFlag_NE(USART3);
        if (LL_USART_IsEnabledIT_ERROR(USART3)) LL_USART_ClearFlag_ORE(USART3);

        debug_log("UART Error!");
        
        // 重置接收
        rx_index = 0;
    }
}
