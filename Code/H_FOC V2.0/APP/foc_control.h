#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "arm_math.h"
#include "foc_setting.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "Config.h"
#include "foc_conversion.h"
#include "foc_encoder.h"

/**
 * @brief FOC初始化，锁定电机至零位并校零编码器
 */
void foc_start_init(void);

/**
 * @brief FOC控制主函数
 */
void foc_control(void);

float foc_get_angle(void);
uint16_t get_svpwm(void);
void foc_debug(void);

#endif /* __FOC_CONTROL_H */
