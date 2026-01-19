#include "foc_control.h"

static void foc_open_loop_control(float target_speed, float target_outq);
static void foc_current_control(float target_d, float target_q);
static void foc_speed_control(float target_speed);
static void foc_position_control(float target_position);
static void foc_position_MIT_control(float target_position);

static inline void pos_ramp_reset(float current_pos_deg);
static inline float pos_ramp_update(float target_pos_deg, float current_pos_deg, float dt);

// ==================== MIT impedance (hybrid) control ====================
// 这个结构就是“关节性格”：硬/柔本质就是 K (刚度) 和 B (阻尼) 的不同
typedef struct {
    float K;          // position stiffness  [Iq/deg]   （你这里输出是Iq，所以单位是“每度产生多少Iq”）
    float B;          // velocity damping    [Iq/(deg/s)]
    float I_limit;    // Iq 输出限幅（保护&手感一致）
    float deadband;   // 位置死区（靠近目标不再抖）
} mit_impedance_t;

// 你可以先给三套“手感档位”，后面再精调
static const mit_impedance_t IMP_SOFT = {
    .K = 0.05f,
    .B = 0.003f,
    .I_limit = 2.0f,
    .deadband = 0.15f, // deg
};

static const mit_impedance_t IMP_NORMAL = {
    .K = 0.12f,
    .B = 0.006f,
    .I_limit = 3.0f,
    .deadband = 0.10f,
};

static const mit_impedance_t IMP_HARD = {
    .K = 0.30f,
    .B = 0.015f,
    .I_limit = 4.0f,
    .deadband = 0.08f,
};

// 当前生效的阻抗参数（运行时切换它即可）
static mit_impedance_t g_imp = {0};
static uint8_t g_imp_inited = 0;

// 软切换用：防止你切档位时“啪一下”或抖一下
static inline float lp1(float x, float y, float alpha) {
    // y = alpha*x + (1-alpha)*y
    return alpha * x + (1.0f - alpha) * y;
}

// 外部调用：设置“硬/柔模式”
// mode: 0=soft, 1=normal, 2=hard
void mit_set_mode(uint8_t mode)
{
    const mit_impedance_t *src = &IMP_NORMAL;
    if (mode == 0) src = &IMP_SOFT;
    else if (mode == 2) src = &IMP_HARD;

    // 第一次直接生效
    if (!g_imp_inited) {
        g_imp = *src;
        g_imp_inited = 1;
        return;
    }

    // 之后建议：慢慢靠过去（软切换），避免突变
    // alpha 越小，切换越慢越柔；先 0.05~0.2 试
    const float alpha = 0.10f;
    g_imp.K        = lp1(src->K,        g_imp.K,        alpha);
    g_imp.B        = lp1(src->B,        g_imp.B,        alpha);
    g_imp.I_limit  = lp1(src->I_limit,  g_imp.I_limit,  alpha);
    g_imp.deadband = lp1(src->deadband, g_imp.deadband, alpha);
}

// FOC控制变量
foc_control_t foc_ctrl;
AlphaBetaTypeDef alpha_beta;
SVPWM_t svpwm;
float speed_err;

uart_parsed_data_t* data; // 串口解析数据

/**
 * @brief FOC设置外部参数
 */
void foc_control_set(void)
{
    // 获取串口解析数据
    data = get_parsed_data();
    // 检查各个值是否有效并使用
    if(data->target_position_valid) {
      foc_ctrl.target_position = data->target_position;
        data->target_position_valid = 0;
    }
    if(data->target_speed_valid) {
        foc_ctrl.target_speed = data->target_speed;
        data->target_speed_valid = 0;
    }
    if(data->current_q_valid) {
        data->current_q_valid = 0;
    }
    if(data->target_q_valid) {
        foc_ctrl.target_q = data->target_q;
        data->target_q_valid = 0;
    }
    mit_set_mode(0);
}

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void)
{
    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", alpha_beta.beta_i, alpha_beta.alpha_i, encoder_data.electrical_angle, foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
    debug_log("%.4f, %.4f, %.4f, %.4f", g_ramp.pos_cmd, encoder_data.mechanical_angle, encoder_data.mechanical_speed, foc_ctrl.target_q);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.abc_dq.current_q, encoder_data.mechanical_speed, foc_ctrl.target_q, foc_ctrl.target_speed);
    // debug_log("%d, %d, %d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    // debug_log("%.4f", foc_voltage_data.vbus);
    // FDCAN_SendFloat4Data(foc_ctrl.target_position, encoder_data.mechanical_angle, foc_ctrl.abc_dq.current_q, foc_ctrl.target_q);
}

/**
 * @brief FOC初始化，锁定电机至零位
 */
void foc_start_init(void)
{   
    foc_ctrl.target_speed = 120.0f;   // 设置目标速度(°/s)
    foc_ctrl.out_q = 0.2f; // 设置q轴输出电压(开环用)
    foc_ctrl.out_d = 0.0f; // 设置d轴输出电压(开环用)
    foc_ctrl.target_q = 0.14f;  // 设置目标Q轴电流
    foc_ctrl.target_d = 0.0f;  // 设置目标D轴电流
    foc_ctrl.target_position = 140.0f;  // 设置目标位置
    // 初始化电流环PI参数
    foc_current_pi_init(I_D_P_GAIN, I_Q_P_GAIN, I_I_GAIN, I_I_LIMIT);
    // 初始化速度环PI参数
    foc_speed_pi_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);
    // 初始化位置环PI参数
    foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN, POSITION_I_LIMIT);

    // 锁定电机至零位
    get_foc_bus_voltage(); // 获取电机母线电压
    // foc_ctrl.out_q = 1.5f; // 初始q轴电压
    // inv_park_transform_f32(&foc_ctrl, &alpha_beta, 90.0f);  //将电角度设为90度(直接当作驱动电机角度)锁定电机至零位
    // svpwm.sector = svpwm_sector_calc(&alpha_beta);
    // svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);
    // svpwm_duty_calc(&svpwm);
    // bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    // HAL_Delay(2000); // 等待电机稳定
}

/**
 * @brief FOC控制主函数
 */
void foc_control(void)
{
    // foc_open_loop_control(foc_ctrl.target_speed, foc_ctrl.out_q);
    // foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
    // foc_speed_control(foc_ctrl.target_speed);
    foc_position_control(foc_ctrl.target_position);
    // foc_position_MIT_control(foc_ctrl.target_position);
}

/**
 * @brief FOC速度开环控制
 */
static void foc_open_loop_control(float target_speed, float target_outq)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器
    foc_ctrl.out_q = target_outq;

    clark_transform(&foc_current_data,&foc_voltage_data, &alpha_beta);
    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl);

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（°/s）
    arm_scale_f32(&target_speed, SPEED_FACTOR360, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize_360(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

    /************************** 3. SVPWM核心：扇区判断→作用时间→占空比 **************************/
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
static void foc_current_control(float target_d, float target_q)
{
    // 将ABC坐标系采集电压电流转换为alpha-beta坐标系电压电流
    clark_transform(&foc_current_data,&foc_voltage_data, &alpha_beta);

    foc_ctrl.angle = encoder_data.electrical_angle; // 读取编码器电角度

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl);

    // 电流环PID计算
    foc_ctrl.out_d = foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    foc_ctrl.out_q = foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

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
static void foc_speed_control(float target_speed)
{
    static uint32_t speed_count = 0;          // 速度环执行计数

    speed_count++;

    if(speed_count >= SPEED_LOOP_COUNT)
    {
    // ===== 重力补偿前馈 =====
    // float theta = deg2rad(encoder_data.mechanical_angle);
    // const float K_G = 0.38f;
    // const float TH0 = MOTOR_LOW;
    // float iq_grav = K_G * sinf(theta - TH0);

    float iq_fb = foc_speed_pid_calculate(target_speed, encoder_data.mechanical_speed);
    foc_ctrl.target_q = iq_fb;
    speed_count = 0;
    }

    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
}

/**
 * @brief FOC位置闭环（外环位置环 + 内环电流环）
 * @param target_position 目标位置（机械弧度，范围：0~360）
 */
static void foc_position_control(float target_position)
{
    static uint32_t count = 0;

    count++;

    /************************** 1. 位置环PI控制 **************************/
    if(count >= POSITION_LOOP_COUNT)
    {
    // 梯形速度规划
    g_ramp.pos_cmd = pos_ramp_update(target_position,
                                  encoder_data.mechanical_angle,
                                  POSITION_LOOP_DT);

    float iq_fb = foc_position_pid_calculate(g_ramp.pos_cmd, encoder_data.mechanical_angle);

    // ===== 重力补偿前馈 =====
    float theta = deg2rad(encoder_data.mechanical_angle);
    const float K_G = 0.38f;
    const float TH0 = MOTOR_LOW;
    float iq_grav = K_G * sinf(theta - TH0);

    // 计算最终目标Q轴电流
    foc_ctrl.target_q = iq_fb + iq_grav;

    count = 0;
    }

    /************************** 2. 内环电流环（跟踪目标Q轴电流） **************************/
    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
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

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

/**
 * @brief MIT 混合/阻抗控制输出 (Iq)
 * @param q_cmd      规划/期望位置（deg）—— 你已经有梯形规划输出
 * @param q          实际位置（deg）
 * @param qd_cmd     规划/期望速度（deg/s）—— 如果你的pos_ramp能输出速度更好，没有就传0
 * @param qd         实际速度（deg/s）—— 建议用你现在更干净的观测速度/滤波速度
 * @return Iq 命令（跟电流环对接）
 */
static inline float mit_impedance_iq(float q_cmd, float q, float qd_cmd, float qd)
{
    // 1) 位置误差
    float e = q_cmd - q;

    // 2) 目标附近死区：减少“临界抖动” + 编码器噪声导致的来回咬合
    if (fabsf(e) < g_imp.deadband) {
        e = 0.0f;
    }

    // 3) MIT 阻抗核心：Iq = K*(q_cmd-q) + B*(qd_cmd-qd)
    float iq = g_imp.K * e + g_imp.B * (qd_cmd - qd);

    // 4) 输出限幅（保护电机/减少尖叫/限制硬度）
    iq = clampf(iq, -g_imp.I_limit, g_imp.I_limit);

    return iq;
}

/**
 * @brief FOC位置闭环（外环位置+阻抗，内环电流）
 */
static void foc_position_MIT_control(float target_position)
{
    static uint32_t count = 0;

    count++;

    if(count >= POSITION_LOOP_COUNT)
    {
        // ========= 0) 选择阻抗档位（你可以用串口/按键/CAN切） =========
        // 举例：根据某个状态切档（你自己替换）
        // mit_set_mode(0); // soft
        // mit_set_mode(1); // normal
        // mit_set_mode(2); // hard
        // 注意：mit_set_mode 内部是“软切换”，不会突变

        // ========= 1) 梯形/加速度轨迹规划：把目标变成平滑 q_cmd =========
        // 你现在只有 pos_cmd，就先这么用
        g_ramp.pos_cmd = pos_ramp_update(target_position,
                                         encoder_data.mechanical_angle,
                                         POSITION_LOOP_DT);

        // 如果你的 ramp 能顺便算出速度（推荐），就用它作为 qd_cmd
        // 否则先给 0，阻尼项变成 -B*qd（也能用）
        float q_cmd  = g_ramp.pos_cmd;
        float qd_cmd = 0.0f;                 // TODO: 若你有 g_ramp.vel_cmd 就填它

        // ========= 2) 取实际速度（建议用观测/滤波后更干净的） =========
        float q     = encoder_data.mechanical_angle;
        float qd    = encoder_data.mechanical_speed;  // 或 g_pos_obs.omega_hat

        // ========= 3) MIT 阻抗（混合控制）输出 Iq =========
        // 注意：这一步替代你 foc_position_pid_calculate 的“外环”
        float iq_imp = mit_impedance_iq(q_cmd, q, qd_cmd, qd);

        // ========= 4) 重力补偿前馈（你已有） =========
        float theta = deg2rad(q); // q是deg
        const float K_G = 0.38f;
        const float TH0 = MOTOR_LOW;
        float iq_grav = K_G * sinf(theta - TH0);

        // ========= 5) 合成最终 Iq =========
        foc_ctrl.target_q = iq_imp + iq_grav;

        // 位置环计数归零
        count = 0;
    }

    // ========= 6) 内环电流环照旧 =========
    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
}
