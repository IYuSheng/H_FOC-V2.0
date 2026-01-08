/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc_encoder.h
  * @brief   AS5048A 编码器驱动头文件（SPI2 接口）
  *          适用于 FOC 电机控制，支持角度读取、零点校准、错误处理
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

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include "spi.h"
#include <math.h>
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "foc_conversion.h"

/* ================================= 硬件配置宏 ================================= */
// AS5048A CS 引脚配置（需根据实际硬件修改，此处示例为 PB12）
#define ENCODER_CS_PORT        GPIOB
#define ENCODER_CS_PIN         LL_GPIO_PIN_12
#define ENCODER_CS_LOW()       LL_GPIO_ResetOutputPin(ENCODER_CS_PORT, ENCODER_CS_PIN)
#define ENCODER_CS_HIGH()      LL_GPIO_SetOutputPin(ENCODER_CS_PORT, ENCODER_CS_PIN)

// AS5048A 寄存器地址（SPI 通信地址）
#define ENCODER_REG_NOP        0x0000          // 无操作寄存器
#define ENCODER_REG_CLEAR_ERR  0x0001          // 清除错误标志寄存器
#define ENCODER_REG_PROG_CTRL  0x0003          // 编程控制寄存器
#define ENCODER_REG_ZERO_HI    0x0016          // 零点位置高位寄存器（Bit13~Bit6）
#define ENCODER_REG_ZERO_LO    0x0017          // 零点位置低位寄存器（Bit5~Bit0）
#define ENCODER_REG_ANGLE      0x3FFF          // 角度输出寄存器（14位，含零点校准）
#define ENCODER_REG_DIAG_AGC   0x3FFD          // 诊断+AGC 寄存器

// 编程控制寄存器位定义
#define PROG_CTRL_PROG_EN      0x0001          // 编程使能位
#define PROG_CTRL_BURN         0x0008          // 烧录触发位
#define PROG_CTRL_VERIFY       0x0040          // 验证位

// 错误标志位定义（来自读数据帧 Bit14）
#define ENCODER_ERR_FLAG       0x4000          // 错误标志位（EF）

// 编码器参数
#define ENCODER_RESOLUTION     14              // 14位分辨率
#define ENCODER_MAX_VALUE      ((1 << ENCODER_RESOLUTION) - 1)  // 最大原始值：16384
#define ENCODER_ANGLE_SCALE    (360.0f / ENCODER_MAX_VALUE)  // 角度缩放因子（°/LSB）
#define ENCODER_ELECTRICAL_SCALE (ENCODER_ANGLE_SCALE * MOTOR_POLE_PAIRS)
#define SPEED_CALC_INTERVAL    0.001f       // 速度计算间隔（1ms，单位：秒）
#define SPEED_FILTER_K         0.2f        // 速度低通滤波系数（0~1，越小越平滑）
#define DIRECTION_CW           1            //电机方向
#define ENCODER_ZERO           25.9513f       //编码器零位（单位：度）
/* ================================= 数据类型定义 ================================= */
// 编码器状态枚举
typedef enum {
    ENCODER_STATUS_OK = 0,        // 正常
    ENCODER_STATUS_ERR_COMM,      // 通信错误
    ENCODER_STATUS_ERR_CORDIC,    // CORDIC 溢出错误
    ENCODER_STATUS_ERR_MAGNET     // 磁场强度异常（过强/过弱）
} encoder_status_t;

// 编码器数据结构体
typedef struct {
    uint16_t raw_angle;           // 原始角度值（0~16383）
    float    angle_raw_scaled;    // 机械角度（0~360°）
    float    mechanical_angle;    // 机械角度(累积值，可正可负)
    float    electrical_angle;    // 电角度（0~360°，需结合电机极对数计算）
    float    mechanical_speed;    // 机械角速度（°/s）
    uint8_t  error_flag;          // 错误标志（0=无错误，1=有错误）
    uint8_t  agc_value;           // AGC 值（反映磁场强度：0=强，255=弱）
} encoder_data_t;

// ================= 机械侧降阶隆博戈(二阶)观测器 =================
// 模型： theta_dot = omega, omega_dot = 0
// 观测器：
//  theta_hat(k+1) = theta_hat(k) + omega_hat(k)*dt + L1*e
//  omega_hat(k+1) = omega_hat(k) + L2*e
//  e = theta_meas - (theta_hat(k) + omega_hat(k)*dt)

typedef struct {
    float theta_hat;   // 内部角度估计(单位: deg)，仅用于观测器
    float omega_hat;   // 估计速度(单位: deg/s)，给控制用
    uint8_t inited;
} pos_luenberger_t;

extern encoder_data_t encoder_data;
extern pos_luenberger_t g_pos_obs;

/* ================================= 函数声明 ================================= */
/**
  * @brief  编码器初始化
  * @param  无
  * @retval encoder_status_t 初始化状态（ENCODER_STATUS_OK 为成功）
  */
encoder_status_t encoder_init(void);

/**
  * @brief  读取编码器原始角度值（14位，未缩放）
  * @param  无
  * @note  大概8.12us
  * @retval uint16_t 原始角度值（0~16383），读取失败返回 0xFFFF
  */
uint16_t encoder_read_raw_angle(void);

/**
  * @brief  读取编码器机械角度（0~360°，已含零点校准）
  * @param  无
  * @note  大概8.4us
  * @retval float 机械角度值（°），读取失败返回 -1.0f
  */
float encoder_read_mechanical_angle(void);

/**
  * @brief  读取编码器电角度
  * @note   将机械角度乘以极对数得到电角度，用于FOC控制中的磁场定向
  * @param  无
  * @retval float: 电角度值（度），失败返回-1.0f
  */
 float encoder_read_electrical_angle(void);

/**
  * @brief  读取编码器机械速度
  * @note   在PWM_FREQhz中断中执行，用于计算机械速度
  * @param  无
  * @retval float: 机械速度
  */
float encoder_get_mechanical_speed(void);

/**
  * @brief  设置编码器零位
  */
encoder_status_t encoder_set_zero_temp(void);

/**
  * @brief  清除编码器错误标志
  * @param  无
  * @retval 无
  */
void encoder_clear_error(void);

#endif  // FOC_ENCODER_H
