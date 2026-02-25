#ifndef FOC_VIBRATION_SWEEP_H
#define FOC_VIBRATION_SWEEP_H

#include <stdint.h>
#include <math.h>

typedef struct {
    uint8_t state;          // 0:空闲, 1:触发, 2:记录中, 3:完成, 4:打印中
    float start_pos;
    float target_step;
    uint32_t sample_cnt;
    float pos_data[500];    // 1kHz * 1000ms
    float iq_data[500];
    uint32_t print_idx;     // 打印索引
} resonance_test_t;

// EI整形器结构
typedef struct {
    float A[3];         // 三个脉冲幅值
    uint32_t delay[3];  // 三个脉冲延迟（采样点数）
    float buffer[1000]; // 延迟线（根据频率调整大小）
    uint32_t buf_idx;
    uint32_t buf_size;
    uint8_t init;
} EI_Shaper_t;

extern EI_Shaper_t g_ei_shaper;
extern resonance_test_t g_res_test;

/**
 * @brief 触发共振频率测试
 */
void resonance_test_trigger(void);

void ei_shaper_init(float freq_hz, float V, float sample_time_s);
float ei_shaper_process(float input);

#endif
