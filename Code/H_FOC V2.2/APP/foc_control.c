#include "foc_control.h"

static inline void foc_open_loop_control(float target_speed, float target_outq);
static inline void foc_current_control(float target_d, float target_q);
static inline void foc_speed_control(float target_speed);
static inline void foc_position_control(float target_position);

static inline void pos_ramp_reset(float current_pos_deg);
static inline float pos_ramp_update(float target_pos_deg, float current_pos_deg, float dt);

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void)
{
    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q, encoder_data.electrical_speed, g_flux_obs.omega, foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.target_position, encoder_data.mechanical_angle, encoder_data.mechanical_speed, foc_ctrl.target_q);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.abc_dq.current_q, encoder_data.mechanical_speed, foc_ctrl.target_q, foc_ctrl.target_speed);
    // debug_log("%d, %d, %d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_current_data.ia, foc_current_data.ib, foc_current_data.ic, foc_ctrl.angle);
    // debug_log("%.4f", foc_voltage_data.vbus);
    // FDCAN_SendFloat4Data(foc_ctrl.target_position, encoder_data.mechanical_angle, foc_ctrl.abc_dq.current_q, foc_ctrl.target_q);
    
    // 电磁转矩计算
    // foc_ctrl.Te = 1.5f * MOTOR_POLE_PAIRS * (MOTOR_FLUX_LINKAGE * foc_ctrl.abc_dq.current_q);
    // debug_log("%.4f", foc_ctrl.Te);

    debug_log("%.4f, %.4f",g_flux_obs.theta, encoder_data.electrical_angle);

    #if FOC_TEST_ENABLE // 扫频测试
    // 简单实现q轴正弦波扫频输出
    static float sine_angle = 0.0f;
    const float amplitude = 0.1f;  // 0.2V峰峰值的一半
    
    // 计算当前正弦值并设置到q轴输出
    foc_ctrl.target_q = amplitude * sinf(sine_angle);
    
    // 角度每次增加1度（弧度制）
    sine_angle += 0.0174533f;  // 1度 = π/180 弧度
    
    // 限制角度在0-2π范围内
    if(sine_angle >= 6.283185f) {
        sine_angle -= 6.283185f;
    }
    #endif
}

/**
 * @brief FOC初始化，锁定电机至零位
 */
void foc_start_init(void)
{   
    foc_ctrl.target_speed = -2.0f;   // 设置目标速度(°/s)
    foc_ctrl.out_q = 0.6f; // 设置q轴输出电压(开环用)
    foc_ctrl.out_d = 0.0f; // 设置d轴输出电压(开环用)
    foc_ctrl.target_q = 0.18f;  // 设置目标Q轴电流
    foc_ctrl.target_d = 0.0f;  // 设置目标D轴电流
    foc_ctrl.target_position = -53.0f;  // 设置目标位置
    // 初始化电流环PI参数
    foc_current_pi_init(I_D_P_GAIN, I_Q_P_GAIN, I_I_GAIN, I_I_LIMIT);
    // 初始化速度环PI参数
    foc_speed_pi_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);
    // 初始化位置环PI参数
    foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN, POSITION_I_LIMIT);

    // 锁定电机至零位
    get_foc_bus_voltage(); // 获取电机母线电压
    debug_log("Vbus: %.2f V", foc_voltage_data.vbus);
    // 电流偏置校准 
    adc1_current_offset_calibrate();

    // 初始化磁链观测器
    flux_observer_init(&g_flux_obs);
    
    #if 0   // 电机零位校准时启用
    LL_TIM_EnableIT_CC4(TIM8);
    foc_ctrl.out_q = 0.5f; // 初始q轴电压
    //  将电角度设为90度(直接当作驱动电机角度)锁定电机至零位
    float sin_theta, cos_theta;
    arm_sin_cos_f32(90.0f, &sin_theta, &cos_theta);
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    HAL_Delay(2000); // 等待电机稳定
    debug_log("%f", encoder_data.mechanical_angle);
    #endif

    #if FOC_PARAMETER_IDENTIFICATION_ENABLE // 电机参数自辨识启用
    foc_motor_param_ident_start(1.0f, 0.5f, 600.0f, 0.5f, 0.05f);
    #endif
}

/**
 * @brief FOC外环控制主函数
 */
void foc_control(void)
{
    // foc_speed_control(foc_ctrl.target_speed);
    // foc_position_control(foc_ctrl.target_position);
    // foc_position_MIT_control(foc_ctrl.target_position);
}

/**
 * @brief FOC内环电流环外部调用接口
 */
void foc_current_in_control(void)
{
    #if FOC_PARAMETER_IDENTIFICATION_ENABLE
    // 调用电机参数自辨识步骤
    if (foc_motor_param_ident_is_running())
    {
        foc_motor_parameter_ident_step(); // 执行辨识步骤
    }
    #else
    // foc_open_loop_control(foc_ctrl.target_speed, foc_ctrl.out_q);
    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
    #endif
}

/**
 * @brief FOC速度开环控制
 */
static inline void foc_open_loop_control(float target_speed, float target_outq)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器
    foc_ctrl.out_q = target_outq;

    clark_transform(&foc_current_data, &alpha_beta);

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（°/s）
    arm_scale_f32(&target_speed, SPEED_FACTOR360, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize_360(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. Park变换（αβ→DQ） **************************/
    float sin_theta, cos_theta;
    arm_sin_cos_f32(foc_ctrl.angle, &sin_theta, &cos_theta);

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    /************************** 3. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    /************************** 4. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);

    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC电流闭环控制函数
 */
static inline void foc_current_control(float target_d, float target_q)
{
    // 将ABC坐标系采集电压电流转换为alpha-beta坐标系电压电流
    clark_transform(&foc_current_data, &alpha_beta);

    foc_ctrl.angle = g_flux_obs.theta; // 读取编码器电角度

    float sin_theta, cos_theta;
    arm_sin_cos_f32(foc_ctrl.angle, &sin_theta, &cos_theta);

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    // // 滤波处理DQ轴电流，用于前馈解耦
    // static float last_current_d = 0.0f;
    // static float last_current_q = 0.0f;
    // foc_ctrl.abc_dq.current_d_filt = foc_ctrl.abc_dq.current_d * 0.1f + last_current_d * 0.9f;
    // foc_ctrl.abc_dq.current_q_filt = foc_ctrl.abc_dq.current_q * 0.1f + last_current_q * 0.9f;
    // last_current_d = foc_ctrl.abc_dq.current_d;
    // last_current_q = foc_ctrl.abc_dq.current_q;

    // 电流环前馈解耦
    float vd_ff = -encoder_data.electrical_speed * MOTOR_INDUCTANCE_Lq * foc_ctrl.abc_dq.current_q;
    float vq_ff =  encoder_data.electrical_speed * (MOTOR_INDUCTANCE_Ld * foc_ctrl.abc_dq.current_d + MOTOR_FLUX_LINKAGE);

    // 电流环PID计算
    float vd_pi = foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    float vq_pi = foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    foc_ctrl.out_d = vd_pi + vd_ff;
    foc_ctrl.out_q = vq_pi + vq_ff;

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    // 观测器更新
    flux_observer_update(&g_flux_obs, 
                         alpha_beta.alpha, alpha_beta.beta,  // 电压（上次输出）
                         alpha_beta.alpha_i, alpha_beta.beta_i,  // 电流
                         PWM_PERIOD_S);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);

    // 计算得到PWM定时器比较值
    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器 
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC速度闭环控制
 */
static inline void foc_speed_control(float target_speed)
{
    // ===== 重力补偿前馈 =====
    // float theta = deg2rad(encoder_data.mechanical_angle);
    // const float K_G = 0.38f;
    // const float TH0 = MOTOR_LOW;
    // float iq_grav = K_G * sinf(theta - TH0);

    float iq_fb = foc_speed_pid_calculate(target_speed, encoder_data.mechanical_speed);
    foc_ctrl.target_q = iq_fb;
}

/**
 * @brief FOC位置闭环（外环位置环 + 内环电流环）
 * @param target_position 目标位置（机械弧度，范围：0~360）
 */
static inline void foc_position_control(float target_position)
{
    /************************** 位置环PI控制 **************************/
    // 梯形速度规划
    // g_ramp.pos_cmd = pos_ramp_update(target_position,
    //                               encoder_data.mechanical_angle,
    //                               POSITION_LOOP_DT);

    float iq_fb = foc_position_pid_calculate(target_position, encoder_data.mechanical_angle);

    // ===== 重力补偿前馈 =====
    float theta = deg2rad(encoder_data.mechanical_angle);
    const float K_G = 2.8f;
    const float TH0 = MOTOR_LOW;
    float iq_grav = K_G * sinf(theta - TH0);

    // 计算最终目标Q轴电流
    foc_ctrl.target_q = iq_fb + iq_grav;
}

static inline void pos_ramp_reset(float current_pos_deg)
{
    g_ramp.pos_cmd = current_pos_deg;
    g_ramp.inited = 1;
}

static inline float pos_ramp_update(float target_pos_deg, float current_pos_deg, float dt)
{
    if (!g_ramp.inited) {
        pos_ramp_reset(current_pos_deg);
    }

    float e = target_pos_deg - g_ramp.pos_cmd;

    // 到点就锁死（避免末位抖）
    const float POS_EPS = 0.05f;
    if (fabsf(e) <= POS_EPS) {
        g_ramp.pos_cmd = target_pos_deg;
        return g_ramp.pos_cmd;
    }

    // 单步最多走 vmax*dt
    float step = g_ramp.vmax * dt;
    if (e > 0.0f) {
        g_ramp.pos_cmd += (e > step) ? step : e;
    } else {
        g_ramp.pos_cmd += (e < -step) ? -step : e;
    }

    return g_ramp.pos_cmd;
}
