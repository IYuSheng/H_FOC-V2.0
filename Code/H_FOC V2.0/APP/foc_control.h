#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "arm_math.h"
#include "foc_setting.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "Config.h"
#include "foc_conversion.h"
#include "foc_encoder.h"
#include "foc_prase.h"
#include <math.h>

typedef struct {
    float pos_cmd;    // 规划位置输出 (deg)
    float vmax;       // 最大速度 (deg/s)
    uint8_t inited;
} pos_ramp_t;

static pos_ramp_t g_ramp = {
    .pos_cmd = 0.0f,
    .vmax = 1000.0f,
    .inited = 0
};

/**
 * @brief FOC初始化，锁定电机至零位并校零编码器
 */
void foc_start_init(void);

/**
 * @brief FOC设置外部参数
 */
void foc_control_set(void);

/**
 * @brief FOC控制主函数
 */
void foc_control(void);

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void);

#endif /* __FOC_CONTROL_H */
