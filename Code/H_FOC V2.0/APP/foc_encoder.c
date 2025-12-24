/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc_encoder.c
  * @brief   AS5048A 编码器驱动源文件（SPI2 接口）
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

#include "foc_encoder.h"

static inline void delay_350ns(void) {
    // 基于170MHz CPU：1个__NOP() = 5.88ns，350ns需约60个__NOP()（留冗余）
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

/* ================================= 静态函数实现 ================================= */

/**
  * @brief  计算16位数据的偶校验位
  * @note   AS5048A要求命令和数据传输必须包含偶校验位
  *         校验位放在Bit15位置，使整个16位数据中1的个数为偶数
  * @param  data: 待计算的16位数据（Bit14~Bit0为有效数据）
  * @retval uint8_t: 计算出的校验位（0或1）
  */
static inline uint8_t calc_parity(uint16_t data) {
    uint8_t parity = 0;
    for (uint8_t i = 0; i < 15; i++) {
        if (data & (1 << i)) {
            parity ^= 1;
        }
    }
    return parity;
}

/**
  * @brief  SPI2发送并接收16位数据（阻塞模式）
  * @note   严格按照SPI时序进行数据传输，确保发送和接收操作完成
  * @param  tx_data: 待发送的16位数据
  * @retval uint16_t: 接收到的16位数据
  */
static inline uint16_t spi2_transfer16(uint16_t tx_data) {
    uint16_t rx_data = 0;
    
    // 等待发送缓冲区空
    while (!LL_SPI_IsActiveFlag_TXE(SPI2));
    LL_SPI_TransmitData16(SPI2, tx_data);
    
    // 等待接收完成
    while (!LL_SPI_IsActiveFlag_RXNE(SPI2));
    rx_data = LL_SPI_ReceiveData16(SPI2);
    
    // 等待传输完成
    while (LL_SPI_IsActiveFlag_BSY(SPI2));
    
    return rx_data;
}

/**
  * @brief  读取编码器指定寄存器值
  * @note   AS5048A读操作需要2次传输：
  *         1. 发送读命令，接收前一次操作的结果
  *         2. 发送NOP命令，接收读命令的响应数据
  * @param  reg_addr: 寄存器地址（Bit13~Bit0，Bit14为读写标志）
  * @retval uint16_t: 寄存器值（Bit13~Bit0），失败返回0xFFFF
  */
static inline uint16_t encoder_read_reg_correct(uint16_t reg_addr) {
    uint16_t read_cmd = 0;
    uint16_t nop_cmd = 0;
    uint16_t rx1 = 0, rx2 = 0;
    
    // 构造读命令：RWn=1（读），地址
    read_cmd = (reg_addr & 0x3FFF) | 0x4000;  // 置位bit14（RWn=1）
    read_cmd |= ((uint16_t)calc_parity(read_cmd) << 15);
    
    // NOP命令：地址0x0000，RWn=1
    nop_cmd = 0x4000 | ((uint16_t)calc_parity(0x4000) << 15);
    
    // CS在整个读操作期间保持低电平！
    ENCODER_CS_LOW();
    
    // 传输1：发送读命令，接收前一个命令的响应
    rx1 = spi2_transfer16(read_cmd);

    ENCODER_CS_HIGH();  // 这里在发送下一个16位数据时需先拉高再拉低
    delay_350ns();  // 满足T_CSnH≥350ns
    
    // 传输2：发送NOP命令，接收读命令的响应
    ENCODER_CS_LOW();
    rx2 = spi2_transfer16(nop_cmd);
    
    ENCODER_CS_HIGH();
    
    // 检查错误标志（bit14）
    if (rx2 & ENCODER_ERR_FLAG)
    {
        debug_log("error flag!");
        // 清除错误标志
        encoder_clear_error();
        return 0xFFFF;
    }
    
    // 返回有效数据（bit13-bit0）
    return (rx2 & ENCODER_REG_ANGLE);
}

/**
  * @brief  向编码器指定寄存器写入值
  * @note   AS5048A写操作需要2次传输：
  *         1. 发送写命令（地址信息）
  *         2. 发送写数据
  * @param  reg_addr: 寄存器地址（Bit13~Bit0，RWn=0表示写操作）
  * @param  data: 要写入的14位数据（Bit13~Bit0）
  * @retval encoder_status_t: 写入状态
  */
static inline encoder_status_t encoder_write_reg_correct(uint16_t reg_addr, uint16_t data) {
    uint16_t write_cmd = 0;
    uint16_t data_cmd = 0;
    uint16_t rx1 = 0, rx2 = 0;
    
    // 数据有效性检查
    if (data > 0x3FFF) {
        return ENCODER_STATUS_ERR_COMM;
    }
    
    // 构造写命令：RWn=0（写），地址
    write_cmd = (reg_addr & 0x3FFF);  // RWn=0
    write_cmd |= ((uint16_t)calc_parity(write_cmd) << 15);
    
    // 数据命令
    data_cmd = data;
    data_cmd |= ((uint16_t)calc_parity(data) << 15);
    
    // CS在整个写操作期间保持低电平！
    ENCODER_CS_LOW();
    
    // 传输1：发送写命令
    rx1 = spi2_transfer16(write_cmd);

    ENCODER_CS_HIGH();
    delay_350ns();  // 满足T_CSnH≥350ns
    
    // 传输2：发送数据
    ENCODER_CS_LOW();
    rx2 = spi2_transfer16(data_cmd);
    
    ENCODER_CS_HIGH();
    
    // 验证写入：读回并比较
    uint16_t readback = encoder_read_reg_correct(reg_addr);
    if (readback == data) {
        return ENCODER_STATUS_OK;
    } else {
        debug_log("[ERROR] expect:0x%04X,Actual:0x%04X\r\n", data, readback);
        return ENCODER_STATUS_ERR_COMM;
    }
}

/* ================================= 全局函数实现 ================================= */

/**
  * @brief  编码器初始化函数
  * @note   检查编码器通信是否正常，读取诊断寄存器验证连接
  * @param  无
  * @retval encoder_status_t: 初始化状态
  */
encoder_status_t encoder_init(void)
{   
    uint16_t diag = encoder_read_reg_correct(ENCODER_REG_DIAG_AGC);
    if (diag == 0xFFFF) {
        debug_log("[ERROR] read diag failed!");
        return ENCODER_STATUS_ERR_COMM;
    }
    debug_log("AGC: %d", diag & 0xFF);
    debug_log("OCF: %s", (diag & (1 << 8)) ? "success" : "fail");
    
    return ENCODER_STATUS_OK;
}

/**
  * @brief  读取编码器原始角度值
  * @note   读取14位原始角度值，范围0~16383（对应0~360度）
  * @param  无
  * @retval uint16_t: 原始角度值（0~16383），失败返回0xFFFF
  */
uint16_t encoder_read_raw_angle(void)
{
    return encoder_read_reg_correct(ENCODER_REG_ANGLE);
}

/**
  * @brief  读取编码器机械角度
  * @note   将14位原始角度转换为度数（0~360度）
  * @param  无
  * @retval float: 机械角度值（度），失败返回-1.0f
  */
float encoder_read_mechanical_angle(void)
{
    uint16_t raw_angle = encoder_read_raw_angle();
    if (raw_angle == 0xFFFF) {
        return -1.0f;
    }
    return (float)raw_angle * ENCODER_ANGLE_SCALE;
}

/**
  * @brief  临时设置零位（掉电不保存）
  * @note   将当前角度设置为新的零位，但不烧录到OTP中，重启后恢复
  * @param  无
  * @retval encoder_status_t: 设置状态
  */
encoder_status_t encoder_set_zero_temp(void)
{
    encoder_status_t ENCODER_STATUS_OK;
    // 写入零位寄存器为0
    encoder_write_reg_correct(ENCODER_REG_ZERO_HI, 0x00);
    encoder_write_reg_correct(ENCODER_REG_ZERO_LO, 0x00);
    // 激活清除操作
    encoder_read_reg_correct(ENCODER_REG_PROG_CTRL);
    // 读取当前原始角度作为新的零位
    uint16_t target_zero_angle = encoder_read_raw_angle();
    uint8_t zero_hi = (target_zero_angle >> 6) & 0xFF;
    uint8_t zero_lo = target_zero_angle & 0x3F;
    
    // 1. 写高位寄存器
    encoder_write_reg_correct(ENCODER_REG_ZERO_HI, zero_hi);
    ENCODER_STATUS_OK = encoder_write_reg_correct(ENCODER_REG_ZERO_HI, zero_hi);
    // 2. 写低位寄存器
    encoder_write_reg_correct(ENCODER_REG_ZERO_LO, zero_lo);
    ENCODER_STATUS_OK = encoder_write_reg_correct(ENCODER_REG_ZERO_LO, zero_lo);
    // 3. 读取编程控制寄存器，触发零位基准更新
    encoder_read_reg_correct(ENCODER_REG_PROG_CTRL);
    return ENCODER_STATUS_OK;
}

/**
  * @brief  清除编码器错误标志
  * @note   读取清除错误标志寄存器即可清除错误状态
  * @param  无
  * @retval 无
  */
void encoder_clear_error(void) {
    // 读取清除错误标志寄存器即可清除错误
    encoder_read_reg_correct(ENCODER_REG_CLEAR_ERR);
}
