#ifndef FOC_COGGING_COMPENSATION_H
#define FOC_COGGING_COMPENSATION_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "foc_control.h"

#define COG_TABLE_SIZE      360     // 每度一个点
#define COG_CAL_ROUNDS      5       // 采集圈数（5圈平均）
#define COG_CAL_SPEED       500.0f   // 标定速度（度/秒），建议10-20

typedef enum {
    COG_CAL_RUNNING = 0,    // 匀速转动采集中
    COG_CAL_PROCESSING,     // 数据处理
    COG_CAL_FINISHED,       // 完成
    COG_CAL_VERIFY          // 验证模式（持续旋转）
} CogCalState_t;

typedef struct {
    float table[COG_TABLE_SIZE];        // 最终结果
    float accum[COG_TABLE_SIZE];        // 累加器
    uint16_t count[COG_TABLE_SIZE];     // 每点采样计数
    
    CogCalState_t state;
    int last_idx;                       // 上次角度索引（用于检测过零）
    uint32_t total_samples;             // 总采样数
    uint8_t compensation_enabled;       // 补偿使能标志
    float current_compensation;         // 当前补偿值
} CoggingComp_t;

extern CoggingComp_t g_cogging;

void cogging_calibration_start(void);
void cogging_calibration_update(float mech_angle, float iq_actual, float *target_speed);

#endif
