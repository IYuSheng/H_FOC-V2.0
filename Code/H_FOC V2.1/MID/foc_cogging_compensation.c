#include "foc_cogging_compensation.h"

CoggingComp_t g_cogging = {0};

void cogging_calibration_start(void)
{
    memset(&g_cogging, 0, sizeof(g_cogging));
    g_cogging.state = COG_CAL_RUNNING;
    g_cogging.last_idx = -1;
    g_cogging.compensation_enabled = 1;  // 默认关闭，验证时可开启
}

void cogging_calibration_update(float mech_angle, float iq_actual, float *target_speed)
{
    switch(g_cogging.state) {
        case COG_CAL_RUNNING: {
            // 设置匀速转动
            *target_speed = COG_CAL_SPEED;
            
            // 计算当前角度索引（0-359）
            int idx = (int)(mech_angle) % 360;
            if (idx < 0) idx += 360;
            
            // 当角度进入新的整数度时记录
            if (idx != g_cogging.last_idx) {
                g_cogging.accum[idx] += iq_actual;
                g_cogging.count[idx]++;
                g_cogging.total_samples++;
                g_cogging.last_idx = idx;
            }
            
            // 采集够指定圈数
            if (g_cogging.total_samples >= 360 * COG_CAL_ROUNDS) {
                g_cogging.state = COG_CAL_PROCESSING;
                *target_speed = 0;  // 短暂停止，准备处理数据
            }
            break;
        }
        
        case COG_CAL_PROCESSING: {
            float sum = 0;
            
            // 1. 计算每点平均
            for (int i = 0; i < 360; i++) {
                if (g_cogging.count[i] > 0) {
                    g_cogging.table[i] = g_cogging.accum[i] / g_cogging.count[i];
                } else {
                    g_cogging.table[i] = 0;
                }
                sum += g_cogging.table[i];
            }
            
            // 2. 去均值
            float mean = sum / 360.0f;
            for (int i = 0; i < 360; i++) {
                g_cogging.table[i] -= mean;
                if (g_cogging.table[i] > 0.5f) g_cogging.table[i] = 0.5f;
                if (g_cogging.table[i] < -0.5f) g_cogging.table[i] = -0.5f;
            }
            
            // 3. 打印结果
            // debug_log("%d, %.4f", i, g_cogging.table[i] * 1000.0f);
            
            g_cogging.state = COG_CAL_VERIFY;
            g_cogging.compensation_enabled = 1;
            break;
        }
        
        case COG_CAL_VERIFY: {
            // 验证模式：持续以恒定速度旋转
            *target_speed = COG_CAL_SPEED;
            
            // 计算齿槽补偿值（查表+插值）
            float iq_cog = 0.0f;
            if (g_cogging.compensation_enabled) {
                float pos = fmodf(mech_angle, 360.0f);
                if (pos < 0) pos += 360.0f;
                int idx = (int)pos;
                float frac = pos - (float)idx;
                int next = (idx + 1) % 360;
                iq_cog = g_cogging.table[idx] * (1.0f - frac) + g_cogging.table[next] * frac;
            }
            
            // 输出补偿值
            // 注意：这里不直接修改target_q，而是返回补偿值
            // 通过全局变量或指针传递补偿值
            g_cogging.current_compensation = iq_cog;
            
            break;
        }
        
        case COG_CAL_FINISHED: {
            // 完全停止
            *target_speed = 0;
            break;
        }
    }
}

// 获取当前补偿值（供foc_control调用）
float cogging_get_compensation(float mech_angle)
{
    if (g_cogging.state != COG_CAL_VERIFY || !g_cogging.compensation_enabled) {
        return 0.0f;
    }
    
    float pos = fmodf(mech_angle, 360.0f);
    if (pos < 0) pos += 360.0f;
    int idx = (int)pos;
    float frac = pos - (float)idx;
    int next = (idx + 1) % 360;
    
    return g_cogging.table[idx] * (1.0f - frac) + g_cogging.table[next] * frac;
}
