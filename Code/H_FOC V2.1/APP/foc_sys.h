/* Sys_Timer.h 文件 */
#ifndef __FOC_SYS_H
#define __FOC_SYS_H

#include "main.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "foc_encoder.h"
#include <config.h>
#include <stdbool.h>

/**
 * @brief FOC定时任务结构体
 */
typedef struct
{
    volatile bool task_update_vbus;
    volatile bool task_update_three_phase_voltage;
    volatile bool task_update_temperature;
    volatile bool task_sys_common;
} FOC_TASK;

extern FOC_TASK foc_task;

#define SYSTICK_FREQUENCY_HZ                    1000         // 系统滴答频率，1000Hz = 1ms
#define TASK_VBUS_UPDATE_FREQUENCY_HZ           10           // 母线电压更新频率，10Hz = 100ms
#define TASK_THREE_VOLTAGE_UPDATE_FREQUENCY_HZ  1000         // 三相端电压更新频率，1000Hz = 1ms
#define TASK_TEMPERATURE_UPDATE_FREQUENCY_HZ    200          // 温度更新频率，200Hz = 5ms

void Error_Handler(void);


#endif /* __FOC_SYS_H */
