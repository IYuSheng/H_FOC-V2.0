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

encoder_data_t encoder_data;

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
        // debug_log("error flag!");
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

pos_luenberger_t g_pos_obs = {0};

static inline void pos_obs_reset(float theta_meas_deg)
{
    g_pos_obs.theta_hat = theta_meas_deg;
    g_pos_obs.omega_hat = 0.0f;
    g_pos_obs.inited = 1;
}

// 50Hz 观测器带宽（你指定）
#define POS_OBS_BW_HZ  (50.0f)

// 由带宽生成增益（工程上常用的“二阶临界阻尼”形式）
// 连续极点放在 s = -omega_obs (重复极点)，离散近似：
// L1 ≈ 2*omega*dt, L2 ≈ omega^2*dt
static inline void pos_obs_update(float theta_meas_deg, float dt)
{
    if (!g_pos_obs.inited) {
        pos_obs_reset(theta_meas_deg);
        return;
    }

    const float omega = 2.0f * _PI * POS_OBS_BW_HZ; // rad/s（只是用于算增益）
    const float L1 = 2.0f * omega * dt;              // 无量纲
    const float L2 = (omega * omega) * dt;           // 1/s

    // 1) 预测（先用模型走一步）
    const float theta_pred = g_pos_obs.theta_hat + g_pos_obs.omega_hat * dt;

    // 2) 创新/残差（角度误差）
    const float e = theta_meas_deg - theta_pred;

    // 3) 校正
    g_pos_obs.theta_hat = theta_pred + L1 * e;
    g_pos_obs.omega_hat = g_pos_obs.omega_hat + L2 * e;
}

/**
  * @brief  编码器初始化函数并校准零点
  * @note   检查编码器通信是否正常，读取诊断寄存器验证连接
  * @param  无
  * @retval encoder_status_t: 初始化状态
  */
encoder_status_t encoder_init(void)
{
    uint16_t diag = encoder_read_reg_correct(ENCODER_REG_DIAG_AGC);
    if (diag == 0xFFFF)
    {
        debug_log("[ERROR] read diag failed!");
        return ENCODER_STATUS_ERR_COMM;
    }
    debug_log("AGC: %d", diag & 0xFF);
    debug_log("OCF: %s", (diag & (1 << 8)) ? "success" : "fail");

    // 校零编码器
    encoder_status_t enc_status;
    // enc_status = encoder_set_zero_temp();
    
    return enc_status;
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
  * @brief  读取编码器累计角度（格式：圈数×360 + 当前角度）
  * @note   正转：圈数递增，如1圈30°→1×360+30=390；反转：圈数递减，如-1圈30°→-1×360+30=-330
  * @retval float: 累计角度（圈数×360 + 当前角度）
  */
float encoder_read_mechanical_angle(void)
{
    static float last_raw = 0;
    float delta = encoder_data.angle_raw_scaled - last_raw;

    // 处理角度环绕伪差
    if (delta > 180) delta -= 360;
    else if (delta < -180) delta += 360;

    // 累加累计角度 & 更新上次原始角度
    encoder_data.mechanical_angle -= delta; // 正负跟电机接线相关
    last_raw = encoder_data.angle_raw_scaled;

    pos_obs_update(encoder_data.mechanical_angle, PWM_PERIOD_S);
    encoder_data.mechanical_speed = g_pos_obs.omega_hat;

    return encoder_data.mechanical_angle;
}

/**
  * @brief  读取编码器电角度
  * @note   将机械角度乘以极对数得到电角度，用于FOC控制中的磁场定向
  * @param  无
  * @retval float: 电角度值（度），失败返回-1.0f
  */
float encoder_read_electrical_angle(void)
{
    encoder_data.raw_angle = encoder_read_raw_angle();
    if (encoder_data.raw_angle == 0xFFFF) {
        return -1.0f;
    }
    encoder_data.angle_raw_scaled = (encoder_data.raw_angle * ENCODER_ANGLE_SCALE - ENCODER_ZERO);
    // 加负号修正编码器/电机极性反的问题
    encoder_data.electrical_angle = angle_normalize_360(DIRECTION_CW * encoder_data.angle_raw_scaled * MOTOR_POLE_PAIRS);
    return encoder_data.electrical_angle;
}

/**
  * @brief  读取编码器机械速度
  * @note   在PWM_FREQhz中断中执行，用于计算机械速度
  * @param  无
  * @retval float: 机械速度
  */
float encoder_get_mechanical_speed(void)
{
    // 静态变量：保持中断执行时的状态（仅初始化一次）
    static uint32_t count = 0;                // 中断执行计数
    static float last_angle_raw_scaled = 0.0f;// 上一次的单圈机械角度（0-360°）
    static uint8_t is_first_run = 1;          // 首次运行标记（避免初始速度异常）
    static float filtered_speed = 0.0f;       // 滤波后的速度（减少抖动）

    // 1. 计算1ms对应的中断执行次数（如PWM_FREQ=10kHz，则count_threshold=10）
    const uint32_t count_threshold = (uint32_t)(PWM_FREQ * SPEED_CALC_INTERVAL);

    // 2. 首次运行：初始化上一次角度，速度置0
    if (is_first_run) {
        last_angle_raw_scaled = encoder_data.angle_raw_scaled;
        encoder_data.mechanical_speed = 0.0f;
        filtered_speed = 0.0f;
        is_first_run = 0;
        return 0.0f;
    }

    // 3. 每1ms计算一次速度（达到计数阈值才计算）
    count++;
    if (count >= count_threshold) {
        // 3.1 计算角度差（处理0-360°环绕伪差）
        float delta_angle = (last_angle_raw_scaled - encoder_data.angle_raw_scaled);
        if (delta_angle > 180.0f) {
            delta_angle -= 360.0f;  // 正转环绕：359°→1° → 真实差+2°
        } else if (delta_angle < -180.0f) {
            delta_angle += 360.0f;  // 反转环绕：1°→359° → 真实差-2°
        }

        // 3.2 计算机械速度（核心公式：速度=角度差/时间间隔，单位°/s）
        float raw_speed = delta_angle / SPEED_CALC_INTERVAL;

        // 3.3 低通滤波（减少速度抖动，提升稳定性）
        filtered_speed = SPEED_FILTER_K * raw_speed + (1 - SPEED_FILTER_K) * filtered_speed;

        // 3.4 更新状态：上一次角度 + 重置计数
        last_angle_raw_scaled = encoder_data.angle_raw_scaled;
        count = 0;

        // 3.5 保存到全局变量，供外部调用
        encoder_data.mechanical_speed = filtered_speed;
    }

    // 4. 返回最新的机械速度（未到计算周期时返回上一次值）
    return encoder_data.mechanical_speed;
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
