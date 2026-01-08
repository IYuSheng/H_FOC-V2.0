#include "foc_control.h"

static void foc_open_loop_control(float target_speed, float target_outq);
static void foc_current_control(float target_d, float target_q);
static void foc_speed_control(float target_speed);
static void foc_position_control(float target_position);

static inline void pos_ramp_reset(float current_pos_deg);
static inline float pos_ramp_update(float target_pos_deg, float current_pos_deg, float dt);


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
}

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void)
{
    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", alpha_beta.beta_i, alpha_beta.alpha_i, encoder_data.electrical_angle, foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
    debug_log("%.4f, %.4f, %.4f, %.4f", g_ramp.pos_cmd, encoder_data.mechanical_angle, encoder_data.mechanical_speed, position_pid.w_ref);
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

    if(speed_count > SPEED_LOOP_COUNT)
    {
    // ===== 重力补偿前馈 =====
    float theta = deg2rad(encoder_data.mechanical_angle);
    const float K_G = 0.38f;
    const float TH0 = MOTOR_LOW;    //rags
    float iq_grav = K_G * sinf(theta - TH0);

    float iq_fb = foc_speed_pid_calculate(target_speed, encoder_data.mechanical_speed);
    foc_ctrl.target_q = iq_fb + iq_grav;
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
    if(count > POSITION_LOOP_COUNT)
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
