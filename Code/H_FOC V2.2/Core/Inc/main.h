#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FOC_Init.h"
#include "foc_control.h"

/**
 * @brief 电机控制结构体
 */
typedef struct
{
    FOC_STATUS foc_state;        // 当前状态
    FOC_FAULT  fault_type;       // 当前故障类型
    float temp_max;              // 最高温度阈值（℃）
    float vbus_max;              // 母线最高电压阈值（V）
    float vbus_min;              // 母线最低电压阈值（V）
    float current_max;           // 最大电流阈值（A）
} Motor_Control_t;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
