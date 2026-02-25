#include "foc_filter.h"

/* ===================== 巴特沃斯滤波器 ===================== */

/**
 * @brief 二阶巴特沃斯低通滤波器初始化
 * @param filt 滤波器结构体指针
 * @param cutoff_freq 截止频率(Hz)
 * @param sample_freq 采样频率(Hz)
 * 
 * 传递函数：H(s) = ωc? / (s? + √2·ωc·s + ωc?)
 */
void butterworth_init(ButterworthLPF_t *filt, float cutoff_freq, float sample_freq)
{
    filt->cutoff_freq = cutoff_freq;
    filt->sample_freq = sample_freq;
    
    // 预扭曲频率（双线性变换）
    float wc = 2.0f * PI * cutoff_freq;
    float T = 1.0f / sample_freq;
    float wa = (2.0f / T) * tanf(wc * T / 2.0f);  // 预扭曲后的角频率
    
    // 二阶巴特沃斯系数计算
    float sqrt2 = 1.41421356f;
    float Q = wa * wa;
    float D = 4.0f + 2.0f * sqrt2 * wa * T + Q * T * T;
    
    // 分子系数（归一化，使直流增益为1）
    filt->b0 = Q * T * T / D;
    filt->b1 = 2.0f * Q * T * T / D;
    filt->b2 = Q * T * T / D;
    
    // 分母系数（a0归一化为1，所以这里是-a1/D, -a2/D）
    filt->a1 = (2.0f * Q * T * T - 8.0f) / D;
    filt->a2 = (4.0f - 2.0f * sqrt2 * wa * T + Q * T * T) / D;
    
    // 清零历史值
    filt->x[0] = filt->x[1] = 0.0f;
    filt->y[0] = filt->y[1] = 0.0f;
}

/**
 * @brief 二阶巴特沃斯滤波器更新
 * @param filt 滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 * 
 * 差分方程：y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 */
float butterworth_filter(ButterworthLPF_t *filt, float input)
{
    // 计算当前输出
    float output = filt->b0 * input 
                 + filt->b1 * filt->x[0] 
                 + filt->b2 * filt->x[1]
                 - filt->a1 * filt->y[0] 
                 - filt->a2 * filt->y[1];
    
    // 更新历史值（移位）
    filt->x[1] = filt->x[0];
    filt->x[0] = input;
    filt->y[1] = filt->y[0];
    filt->y[0] = output;
    
    return output;
}

/* ===================== 陷波滤波器 ===================== */

void notch_filter_init(NotchFilter_t *nf, float fc, float bw, float sample_time_ms)
{
    if (!nf || fc <= 0 || bw <= 0) return;
    
    nf->fc = fc;
    nf->bw = bw;
    
    float Ts = sample_time_ms / 1000.0f;
    float fs = 1.0f / Ts;
    
    // 双线性变换设计陷波滤波器
    float w0 = 2.0f * PI * fc;
    float Q = fc / bw;
    
    // 预扭曲
    float w0_pw = 2.0f * fs * tanf(w0 / (2.0f * fs));
    
    float alpha = w0_pw / (2.0f * Q);
    float c = cosf(w0 / fs);
    
    // 计算系数
    float b0 = 1.0f;
    float b1 = -2.0f * c;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * c;
    float a2 = 1.0f - alpha;
    
    // 归一化
    nf->b[0] = b0 / a0;
    nf->b[1] = b1 / a0;
    nf->b[2] = b2 / a0;
    nf->a[0] = 1.0f;
    nf->a[1] = a1 / a0;
    nf->a[2] = a2 / a0;
    
    notch_filter_reset(nf);
    nf->init = 1;
}

float notch_filter_process(NotchFilter_t *nf, float input)
{
    if (!nf || !nf->init) return input;
    
    // 移位
    nf->x[2] = nf->x[1];
    nf->x[1] = nf->x[0];
    nf->x[0] = input;
    
    nf->y[2] = nf->y[1];
    nf->y[1] = nf->y[0];
    
    // 差分方程
    float output = nf->b[0] * nf->x[0] 
                 + nf->b[1] * nf->x[1] 
                 + nf->b[2] * nf->x[2]
                 - nf->a[1] * nf->y[1] 
                 - nf->a[2] * nf->y[2];
    
    nf->y[0] = output;
    return output;
}

void notch_filter_reset(NotchFilter_t *nf)
{
    if (!nf) return;
    memset(nf->x, 0, sizeof(nf->x));
    memset(nf->y, 0, sizeof(nf->y));
}
