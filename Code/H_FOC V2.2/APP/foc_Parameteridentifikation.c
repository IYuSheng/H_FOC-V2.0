#include "foc_Parameteridentifikation.h"

typedef enum {
    PARAM_ID_IDLE = 0,       // 待机
    PARAM_ID_ALIGN_LOCK,     // 用D轴直流电压把转子拉到固定电角度（0°）
    PARAM_ID_RS_SETTLE,      // Rs：直流注入等待稳定
    PARAM_ID_RS_SAMPLE,      // Rs：采样平均电流
    PARAM_ID_LD_HF_RUN,      // Ld：D轴注入高频正弦（零均值）
    PARAM_ID_LQ_HF_RUN,      // Lq：Q轴注入高频正弦，同时D轴加少量锁定电压
    PARAM_ID_FLUX_RUN,       // 磁链辨识运行（开环旋转）
    PARAM_ID_COMPLETED,      // 辨识完成
    PARAM_ID_ABORT           // 辨识中止
} ParamIdentState_t;

typedef struct {
    ParamIdentState_t st;
    uint8_t running;

    // 配置参数
    float v_align;      // 对齐/锁定用D轴直流电压
    float v_rs;         // Rs测试D轴直流电压
    float hf_freq;      // 高频测试频率
    float v_hf;         // 高频注入幅值
    float v_lock_d;     // Lq测试时的D轴锁定电压（抑制转子抖动）
    float v_flux;       // 磁链计算时的注入电压

    // 时间/计数
    uint32_t tick;
    uint32_t st_tick;

    uint32_t align_ticks;
    uint32_t rs_settle_ticks;
    uint32_t rs_sample_ticks;

    uint32_t hf_skip_ticks;      // 跳过前若干周期
    uint32_t hf_sample_ticks;    // 采样若干周期

    uint32_t flux_sample_ticks;  // 磁链计算采样周期

    // 累计统计
    float i_sum_abs;     // Rs：|i_d| 绝对值均值更稳
    uint32_t i_cnt;

    float v2_sum;        // L：电压平方和（RMS）
    float i2_sum;        // L：电流平方和（RMS）
    uint32_t vi_cnt;

    // 磁链数据
    float flux_linkage_sum;
    uint32_t flux_linkage_cnt;

    // 输出/测量使用的固定角度
    float ident_angle_deg;  // 这里用 0°：d=alpha, q=beta
} motor_ident_t;

// 辨识结果
static motor_param_t  g_motor_param = {0};
static motor_ident_t  g_ident = {0};

// 把(out_d,out_q,angle) 走 invPark + SVPWM + PWM输出（辨识/开环/闭环共用）
static inline void foc_voltage_output(float out_d, float out_q, float angle_deg)
{
    // 估算最大可用相电压矢量幅值（SVPWM线性区近似：Vmax ≈ Vbus/sqrt(3)）
    float vbus = foc_voltage_data.vbus;
    float v_limit = vbus * 0.577350269f * 0.95f; // 留5%裕量

    float mag = sqrtf(out_d*out_d + out_q*out_q);
    if (mag > v_limit && mag > EPS_F) {
        float k = v_limit / mag;
        out_d *= k;
        out_q *= k;
    }

    foc_ctrl.out_d = out_d;
    foc_ctrl.out_q = out_q;
    foc_ctrl.angle = angle_deg;

    float sin_theta, cos_theta;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, vbus);
    svpwm_duty_calc(&svpwm);

    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

// 辨识：初始化并启动
void foc_motor_param_ident_start(float v_align, float v_rs, float hf_freq, float v_hf, float v_lock_d)
{
    memset(&g_ident, 0, sizeof(g_ident));
    g_ident.running = 1;
    g_ident.st = PARAM_ID_ALIGN_LOCK;

    g_ident.v_align  = v_align;
    g_ident.v_rs     = v_rs;
    g_ident.hf_freq  = hf_freq;
    g_ident.v_hf     = v_hf;
    g_ident.v_lock_d = v_lock_d;

    g_ident.ident_angle_deg = 0.0f;

    // 时间参数（按你PWM频率换算）
    g_ident.align_ticks     = (uint32_t)(PWM_FREQ * 0.20f); // 200ms对齐锁定
    g_ident.rs_settle_ticks = (uint32_t)(PWM_FREQ * 0.30f); // 300ms等待电流稳定
    g_ident.rs_sample_ticks = (uint32_t)(PWM_FREQ * 0.05f); // 50ms采样
    g_ident.flux_sample_ticks = (uint32_t)(PWM_FREQ * 5.0f); // 5000ms采样

    // 跳过2周期，采样5周期
    float ticks_per_cycle = PWM_FREQ / hf_freq;
    g_ident.hf_skip_ticks   = (uint32_t)(ticks_per_cycle * 2.0f);
    g_ident.hf_sample_ticks = (uint32_t)(ticks_per_cycle * 5.0f);

    // 运行前先清空结果标志
    g_motor_param.valid = 0;

    debug_log("[PARAM_ID] start: v_align=%.3f v_rs=%.3f f=%.1fHz v_hf=%.3f v_lock_d=%.3f",
              v_align, v_rs, hf_freq, v_hf, v_lock_d);
}

uint8_t foc_motor_param_ident_is_running(void)
{
    return g_ident.running;
}

// 在PWM中断里调用：执行一步自辨识
void foc_motor_parameter_ident_step(void)
{
    // 1) 采样电流（辨识角度固定0°：d=alpha, q=beta）
    clark_transform(&foc_current_data, &alpha_beta);

    float sin_theta, cos_theta;
    arm_sin_cos_f32(g_ident.ident_angle_deg, &sin_theta, &cos_theta);
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    // 2) 状态机
    switch (g_ident.st)
    {
        case PARAM_ID_ALIGN_LOCK:
        {
            // 用D轴直流电压把转子“拉住”到 0° 附近
            foc_voltage_output(g_ident.v_align, 0.0f, g_ident.ident_angle_deg);

            if (++g_ident.st_tick >= g_ident.align_ticks) {
                g_ident.st_tick = 0;
                g_ident.st = PARAM_ID_RS_SETTLE;
                debug_log("[PARAM_ID] ALIGN done -> RS_SETTLE");
            }
        } break;

        case PARAM_ID_RS_SETTLE:
        {
            // Rs测试：施加D轴直流电压，等待电流进入稳态
            foc_voltage_output(g_ident.v_rs, 0.0f, g_ident.ident_angle_deg);

            if (++g_ident.st_tick >= g_ident.rs_settle_ticks) {
                g_ident.st_tick = 0;
                g_ident.i_sum_abs = 0.0f;
                g_ident.i_cnt = 0;
                g_ident.st = PARAM_ID_RS_SAMPLE;
                debug_log("[PARAM_ID] RS settle done -> RS_SAMPLE");
            }
        } break;

        case PARAM_ID_RS_SAMPLE:
        {
            // 继续保持直流注入并采样 |i_d|
            foc_voltage_output(g_ident.v_rs, 0.0f, g_ident.ident_angle_deg);

            float id = foc_ctrl.abc_dq.current_d;
            g_ident.i_sum_abs += fabsf(id);
            g_ident.i_cnt++;

            if (++g_ident.st_tick >= g_ident.rs_sample_ticks) {
                float id_avg = g_ident.i_sum_abs / (float)(g_ident.i_cnt + 1e-9f);

                if (id_avg < 0.02f) { // 电流太小：可能电压太低/采样比例问题
                    g_ident.st = PARAM_ID_ABORT;
                    debug_log("[PARAM_ID] RS abort: id_avg too small (%.4f)", id_avg);
                    break;
                }

                g_motor_param.Rs = g_ident.v_rs / id_avg;

                // 进入Ld测试
                g_ident.st_tick = 0;
                g_ident.v2_sum = 0.0f;
                g_ident.i2_sum = 0.0f;
                g_ident.vi_cnt = 0;
                g_ident.st = PARAM_ID_LD_HF_RUN;

                debug_log("[PARAM_ID] Rs=%.6f (V=%.3f, |Id|avg=%.4f) -> LD_HF",
                          g_motor_param.Rs, g_ident.v_rs, id_avg);
            }
        } break;

        case PARAM_ID_LD_HF_RUN:
        {
            // D轴高频正弦注入（零均值），Q轴为0（几乎无平均转矩）
            float t = (float)g_ident.st_tick / PWM_FREQ;
            float vd = g_ident.v_hf * sinf(_2PI * g_ident.hf_freq * t);

            foc_voltage_output(vd, 0.0f, g_ident.ident_angle_deg);

            // 跳过前2周期，采样后5周期
            if (g_ident.st_tick >= g_ident.hf_skip_ticks &&
                g_ident.st_tick < (g_ident.hf_skip_ticks + g_ident.hf_sample_ticks))
            {
                float id = foc_ctrl.abc_dq.current_d;
                g_ident.v2_sum += vd * vd;
                g_ident.i2_sum += id * id;
                g_ident.vi_cnt++;
            }

            g_ident.st_tick++;

            if (g_ident.st_tick >= (g_ident.hf_skip_ticks + g_ident.hf_sample_ticks)) {
                float v_rms = sqrtf(g_ident.v2_sum / (float)(g_ident.vi_cnt + 1e-9f));
                float i_rms = sqrtf(g_ident.i2_sum / (float)(g_ident.vi_cnt + 1e-9f));

                if (i_rms < 0.02f) {
                    g_ident.st = PARAM_ID_ABORT;
                    debug_log("[PARAM_ID] LD abort: i_rms too small (%.4f)", i_rms);
                    break;
                }

                float Z = v_rms / i_rms;
                float w = _2PI * g_ident.hf_freq;

                float Z2 = Z * Z;
                float R2 = g_motor_param.Rs * g_motor_param.Rs;
                float tmp = Z2 - R2;
                if (tmp < 0.0f) tmp = 0.0f; // 防负数

                g_motor_param.Ld = sqrtf(tmp) / (w + EPS_F);

                // 进入Lq测试
                g_ident.st_tick = 0;
                g_ident.v2_sum = 0.0f;
                g_ident.i2_sum = 0.0f;
                g_ident.vi_cnt = 0;
                g_ident.st = PARAM_ID_LQ_HF_RUN;

                debug_log("[PARAM_ID] Ld=%.9f (v_rms=%.4f i_rms=%.4f Z=%.4f) -> LQ_HF",
                          g_motor_param.Ld, v_rms, i_rms, Z);
            }
        } break;

        case PARAM_ID_LQ_HF_RUN:
        {
            // 为了抑制Lq测试时的转子抖动：D轴加锁定电压，Q轴注入高频正弦
            float t = (float)g_ident.st_tick / PWM_FREQ;
            float vq = g_ident.v_hf * sinf(_2PI * g_ident.hf_freq * t);

            foc_voltage_output(g_ident.v_lock_d, vq, g_ident.ident_angle_deg);

            if (g_ident.st_tick >= g_ident.hf_skip_ticks &&
                g_ident.st_tick < (g_ident.hf_skip_ticks + g_ident.hf_sample_ticks))
            {
                float iq = foc_ctrl.abc_dq.current_q;
                g_ident.v2_sum += vq * vq;
                g_ident.i2_sum += iq * iq;
                g_ident.vi_cnt++;
            }

            g_ident.st_tick++;

            if (g_ident.st_tick >= (g_ident.hf_skip_ticks + g_ident.hf_sample_ticks)) {
                float v_rms = sqrtf(g_ident.v2_sum / (float)(g_ident.vi_cnt + 1e-9f));
                float i_rms = sqrtf(g_ident.i2_sum / (float)(g_ident.vi_cnt + 1e-9f));

                if (i_rms < 0.02f) {
                    g_ident.st = PARAM_ID_ABORT;
                    debug_log("[PARAM_ID] LQ abort: i_rms too small (%.4f)", i_rms);
                    break;
                }

                float Z = v_rms / i_rms;
                float w = _2PI * g_ident.hf_freq;

                float Z2 = Z * Z;
                float R2 = g_motor_param.Rs * g_motor_param.Rs;
                float tmp = Z2 - R2;
                if (tmp < 0.0f) tmp = 0.0f;

                g_motor_param.Lq = sqrtf(tmp) / (w + EPS_F);
                g_motor_param.Ls = 0.5f * (g_motor_param.Ld + g_motor_param.Lq);

                g_ident.st = PARAM_ID_FLUX_RUN;

                debug_log("[PARAM_ID] DONE: Rs=%.6f  Ld=%.9f  Lq=%.9f  Ls=%.9f",
                          g_motor_param.Rs, g_motor_param.Ld, g_motor_param.Lq, g_motor_param.Ls);
            }
        } break;

        case PARAM_ID_FLUX_RUN:
        {
            float Uq = 0.5f;
            float Ud = 0.0f;

            foc_ctrl.out_q = Uq;
            foc_ctrl.out_d = Ud;

            clark_transform(&foc_current_data, &alpha_beta);

            foc_ctrl.angle = encoder_data.electrical_angle;

            float sin_theta, cos_theta;
            arm_sin_cos_f32(foc_ctrl.angle, &sin_theta, &cos_theta);

            // 将alpha-beta坐标系电流转换为DQ坐标系电流
            park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

            inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

            // 扇区判断
            svpwm.sector = svpwm_sector_calc(&alpha_beta);

            // 计算基本矢量与零矢量作用时间
            svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);

            svpwm_duty_calc(&svpwm);

            // 输出PWM到定时器
            bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);

            // 假设电流恒定不变，I_d 和 I_q
            float Id = foc_ctrl.abc_dq.current_d;  // 当前 D 轴电流
            float Iq = foc_ctrl.abc_dq.current_q;  // 当前 Q 轴电流

            // 获取电机电角速度 (rad/s)，从编码器数据中读取
            float we = encoder_data.electrical_speed;  // 电角速度（rad/s）

            // 使用公式计算磁链
            // U_q = R_s * I_q + w_e * L_d * I_d + w_e * flux_linkage
            // => flux_linkage = (U_q - R_s * I_q - w_e * L_d * I_d) / w_e
            float flux_linkage = 0.0f;
            flux_linkage = (Uq - g_motor_param.Rs * Iq - we * g_motor_param.Ld * Id) / we;

            // 累计磁链值(跳过前2秒，等待稳定)
            if(g_ident.flux_linkage_cnt >= (PWM_FREQ * 2.0f))
            {
            g_ident.flux_linkage_sum += flux_linkage;
            }

            g_ident.flux_linkage_cnt++;

            // 计算平均磁链
            if (g_ident.flux_linkage_cnt >= g_ident.flux_sample_ticks) {
                float avg_flux_linkage = g_ident.flux_linkage_sum / ((float)g_ident.flux_linkage_cnt - PWM_FREQ * 2.0f);
                debug_log("Flux Linkage = %.8f", avg_flux_linkage);
                g_motor_param.flux_linkage = avg_flux_linkage;

                // 重置计数器和和
                g_ident.flux_linkage_sum = 0.0f;
                g_ident.flux_linkage_cnt = 0;
                g_motor_param.valid = 1;
                g_ident.st = PARAM_ID_COMPLETED;
            }
        } break;

        case PARAM_ID_COMPLETED:
        {
            // 停止注入
            foc_voltage_output(0.0f, 0.0f, g_ident.ident_angle_deg);
            g_ident.running = 0;
            g_ident.st = PARAM_ID_IDLE;
        } break;

        case PARAM_ID_ABORT:
        default:
        {
            foc_voltage_output(0.0f, 0.0f, g_ident.ident_angle_deg);
            g_ident.running = 0;
            g_ident.st = PARAM_ID_IDLE;
            debug_log("[PARAM_ID] ABORT");
        } break;
    }

    g_ident.tick++;
}
