#ifndef __FOC_FILTER_H
#define __FOC_FILTER_H

#include "foc_control.h"

typedef struct {
    // 二阶巴特沃斯滤波器系数
    float b0, b1, b2;  // 分子系数
    float a1, a2;      // 分母系数（a0归一化为1）
    
    // 历史值
    float x[2];        // 输入历史 [n-1, n-2]
    float y[2];        // 输出历史 [n-1, n-2]
    
    float cutoff_freq; // 截止频率(Hz)
    float sample_freq; // 采样频率(Hz)
} ButterworthLPF_t;

/* ===================== 陷波滤波器  ===================== */

typedef struct {
    float fc;           // 中心频率(Hz)
    float bw;           // 带宽(Hz)
    float b[3];         // 分子系数
    float a[3];         // 分母系数 (a[0]=1)
    float x[3];         // 输入历史
    float y[3];         // 输出历史
    uint8_t init;
} NotchFilter_t;

/**
 * @brief 二阶巴特沃斯低通滤波器初始化
 * @param filt 滤波器结构体指针
 * @param cutoff_freq 截止频率(Hz)
 * @param sample_freq 采样频率(Hz)
 * 
 * 传递函数：H(s) = ωc? / (s? + √2·ωc·s + ωc?)
 */
void butterworth_init(ButterworthLPF_t *filt, float cutoff_freq, float sample_freq);

/**
 * @brief 二阶巴特沃斯滤波器更新
 * @param filt 滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 * 
 * 差分方程：y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 */
float butterworth_filter(ButterworthLPF_t *filt, float input);

// fc: 中心频率(你的2.14Hz), bw: 带宽(建议0.5Hz), sample_time_ms: 采样周期(1ms)
void notch_filter_init(NotchFilter_t *nf, float fc, float bw, float sample_time_ms);

// 处理一个采样点
float notch_filter_process(NotchFilter_t *nf, float input);

void notch_filter_reset(NotchFilter_t *nf);

#endif
