#include "foc_vibration_sweep.h"
#include "foc_control.h"

resonance_test_t g_res_test = {0};
EI_Shaper_t g_ei_shaper;

/**
 * @brief 触发共振频率测试
 */
void resonance_test_trigger(void)
{
    g_res_test.state = 1;
    g_res_test.start_pos = encoder_data.mechanical_angle;
    g_res_test.target_step = 45.0f;  // 阶跃45度
    g_res_test.sample_cnt = 0;
    
    // 临时降低位置环增益
    foc_position_pi_init(0.2, 0.0f, 0.0f, 0.0f);
    
    // 立即设置目标位置（产生阶跃）
    foc_ctrl.target_position = g_res_test.start_pos + g_res_test.target_step;
}

/**
 * @brief EI (Extra Insensitive) 输入整形器初始化
 * 脉冲序列:
 *     A1          A2          A3
 *     |           |           |
 *     v           v           v
 *  ----|-----------|-----------|----> 时间
 *     t1=0        t2=Td       t3=2*Td
 * 
 * @param freq_hz      系统共振频率(Hz)
 * @param V            允许的残留振荡百分比(0~1)，建议0.05~0.1
 *                     V=0时退化为ZV整形器（理论零残留）
 * @param sample_time_ms 控制周期(ms)，如1kHz控制频率则为1.0ms
 */
void ei_shaper_init(float freq_hz, float V, float sample_time_s)
{
    // 计算延迟时间
    float Td = 0.5f / freq_hz;
    
    g_ei_shaper.A[0] = (1.0f + V) / 4.0f;
    g_ei_shaper.A[1] = (1.0f - V) / 2.0f;
    g_ei_shaper.A[2] = (1.0f + V) / 4.0f;
    
    // 转换为采样点数 (假设1ms周期)
    g_ei_shaper.delay[0] = 0;
    g_ei_shaper.delay[1] = (uint32_t)(Td / sample_time_s);
    g_ei_shaper.delay[2] = 2 * g_ei_shaper.delay[1];
    
    g_ei_shaper.buf_size = g_ei_shaper.delay[2] + 10; // 留点余量
    memset(g_ei_shaper.buffer, 0, sizeof(g_ei_shaper.buffer));
    g_ei_shaper.buf_idx = 0;
    g_ei_shaper.init = 1;
}

// EI整形处理
float ei_shaper_process(float input)
{
    if (!g_ei_shaper.init) return input;
    
    // 存入当前输入
    g_ei_shaper.buffer[g_ei_shaper.buf_idx] = input;
    
    // 计算三个脉冲的贡献
    float output = 0;
    for (int i = 0; i < 3; i++) {
        int32_t read_idx = (int32_t)g_ei_shaper.buf_idx - (int32_t)g_ei_shaper.delay[i];
        if (read_idx < 0) read_idx += g_ei_shaper.buf_size;
        output += g_ei_shaper.A[i] * g_ei_shaper.buffer[read_idx];
    }
    
    // 更新索引
    g_ei_shaper.buf_idx++;
    if (g_ei_shaper.buf_idx >= g_ei_shaper.buf_size) {
        g_ei_shaper.buf_idx = 0;
    }
    
    return output;
}
