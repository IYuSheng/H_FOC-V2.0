#include "foc_trajectory_planning.h"

/**
 * @brief S型曲线规划器初始化
 * 设置运动学约束参数，但不启动运动
 */
void s_curve_planner_init(SCurve_Planner_t *planner, 
                          float vmax, float amax, float jmax, 
                          float freq_t)
{
    planner->v_max = vmax;
    planner->a_max = amax;
    planner->j_max = jmax;
    planner->freq_t = freq_t;
    
    planner->state = S_CURVE_IDLE;
    planner->current_pos = 0.0f;
    planner->current_vel = 0.0f;
    planner->current_acc = 0.0f;
    planner->is_busy = 0;
}

/**
 * @brief 计算S曲线各段时间参数
 * 遵循7段S型曲线理论，处理三种情况：
 * 1. 完整7段（有匀速）
 * 2. 无匀速但能达到a_max（有T2）
 * 3. 连a_max都达不到（T2=0，纯S型）
 */
uint8_t s_curve_set_target(SCurve_Planner_t *planner, 
                           float current_pos, 
                           float target_pos)
{
    float dist = target_pos - current_pos;
    float S = fabsf(dist);  // 总位移
    
    if (S < 0.01f) {
        planner->state = S_CURVE_IDLE;
        return 0;
    }
    
    planner->direction = (dist > 0.0f) ? 1.0f : -1.0f;
    planner->total_dist = S;
    planner->target_pos = target_pos;
    planner->current_pos = current_pos;
    planner->current_vel = 0.0f;
    planner->current_acc = 0.0f;
    planner->step_counter = 0;
    planner->is_busy = 1;
    
    float v_max = planner->v_max;
    float a_max = planner->a_max;
    float j_max = planner->j_max;
    float dt = planner->freq_t;
    
    // 先计算标准T1（能达到a_max的加加速时间）
    float T1_std = a_max / j_max;
    float T3_std = T1_std;

    float temp_1 = j_max * T1_std * T1_std;
    float temp_2 = temp_1 * T1_std;
    
    // T1段末速度：v1 = 0.5 * J * T1^2
    float v1_std = 0.5f * temp_1;
    
    // T1段位移：s1 = (1/6) * J * T1^3
    float s1_std = (1.0f/6.0f) * temp_2;
    float s3_std = (5.0f/6.0f) * temp_2;
    
    // 计算能达到a_max时需要的总最小距离（T2=0，纯S型，加速+减速）
    // S_min_a = 2 * (s1 + s3) = 2 * (J * T1^3)
    float accel_dist_min = temp_2;
    float S_min_a = 2.0f * accel_dist_min;
    
    // 计算能达到v_max时的总加速距离（有T2）
    // v_max = v1 + a_max*T2 + v1 = 2*v1 + a_max*T2  =>  T2 = (v_max - 2*v1)/a_max
    float T2_std = (v_max - 2.0f * v1_std) / a_max;
    if (T2_std < 0) T2_std = 0;  // 达不到v_max
    
    // 匀加速段T2的位移：s2 = (v1+0.5*T2*a_max)*T2
    float s2_full = (v1_std + 0.5 * T2_std * a_max) * T2_std; // 匀加速阶段位移
    float S_accel_full = s2_full + accel_dist_min;          // 加速段总距离
    float S_total_full = 2.0f * S_accel_full;               // 加减速总距离
    
    // 判断属于哪种情况
    float T1, T2, T3, T4, T5, T6, T7;
    float v_max_actual = v_max;  // 实际能达到的最大速度
    
    if (S >= S_total_full) {
        // ===== Case 1: 完整7段，有匀速段T4 =====
        T1 = T3 = T1_std;
        T2 = T2_std;
        T4 = (S - S_total_full) / v_max;
        v_max_actual = v_max;
    }
    else if (S >= S_min_a)
    {
        // 无匀速，但能达到a_max（有T2，但比T2_std短, 加速时间短，达不到v_max）
        // 需要求解 T2 使得 S/2 = s1_std + s2(T2) + s3_std
        // s2 = v1*T2 + 0.5*a_max*T2^2
        // 解二次方程：0.5*a_max*T2^2 + v1_std*T2 + (s1_std+s3_std - S/2) = 0
        float c = s1_std + s3_std - 0.5 * S;
        // 判别式
        float discriminant = v1_std * v1_std - 2.0f * a_max * c;
        if (discriminant >= 0) {
            T2 = (-v1_std + sqrtf(discriminant)) / a_max;  // 取正根
            if (T2 < 0) T2 = 0;
        } else {
            T2 = 0;  // 数值误差保护
        }
        T1 = T3 = T1_std;
        T4 = 0;
        // 实际最大速度
        v_max_actual = 2.0f * v1_std + a_max * T2;
    }
    else
    {
        // ===== Case 3: 连a_max都达不到（T2=0，纯S型）=====
        // 总距离 S = 2 * (J * T1^3)
        // 所以 T1 = cbrt(S / (2 * j_max))
        T1 = cbrtf(S / (2.0f * j_max)); //立方根处理
        T3 = T1;
        T2 = 0;
        T4 = 0;
        // 实际最大速度
        v_max_actual = j_max * T1 * T1;
    }
    
    // 减速段时间（对称）
    T5 = T3;
    T6 = T2;
    T7 = T1;
    
    // 保存参数
    planner->T1 = T1; planner->T2 = T2; planner->T3 = T3; planner->T4 = T4;
    planner->T5 = T5; planner->T6 = T6; planner->T7 = T7;
    planner->v_max = v_max_actual;  // 更新为实际能达到的速度
    
    // 计算各段步数（用于update函数）
    planner->stage_steps = (uint32_t)(T1 / dt) + 1;  // T1段步数，+1保险
    
    // 启动
    planner->state = S_CURVE_T1;
    planner->step_counter = 0;
    planner->current_jerk = planner->direction * j_max;
    
    return 0;
}

/**
 * @brief S型曲线实时更新
 * 在位置环中每周期调用（如1kHz）
 * 根据当前所处阶段（T1-T7）计算下一步的运动学量
 */
float s_curve_update(SCurve_Planner_t *planner)
{
    if (planner->state == S_CURVE_IDLE) {
        return planner->current_pos;
    }
    
    float dt = planner->freq_t;
    float dir = planner->direction;
    
    // 阶段状态机
    switch (planner->state) {
        case S_CURVE_T1: {  // 加加速段: a从0增加到a_max
            float t = planner->step_counter * dt;
            float T = planner->T1;
            
            if (t >= T) {
                // 进入T2
                planner->state = S_CURVE_T2;
                planner->step_counter = 0;
                planner->current_acc = dir * planner->a_max;
                planner->current_jerk = 0.0f;
            } else {
                // a = J*t, v = 0.5*J*t^2, p = (1/6)*J*t^3
                planner->current_jerk = dir * planner->j_max;
                planner->current_acc = dir * planner->j_max * t;
                planner->current_vel = dir * 0.5f * planner->j_max * t * t;
                planner->current_pos += planner->current_vel * dt + 0.5f * planner->current_acc * dt * dt;
                planner->step_counter++;
            }
            break;
        }
        
        case S_CURVE_T2: {  // 匀加速段: a = a_max
            float t = planner->step_counter * dt;
            float T = planner->T2;
            
            if (t >= T || T < dt) { // T2可能为0
                planner->state = S_CURVE_T3;
                planner->step_counter = 0;
            } else {
                planner->current_jerk = 0.0f;
                planner->current_acc = dir * planner->a_max;
                planner->current_vel = planner->current_vel + dir * planner->a_max * dt;
                planner->current_pos = planner->current_pos + planner->current_vel * dt 
                                     + 0.5f * planner->current_acc * dt * dt;
                planner->step_counter++;
            }
            break;
        }
        
        case S_CURVE_T3: {  // 减加速段: a从a_max减到0
            float t = planner->step_counter * dt;
            float T = planner->T3;
            
            if (t >= T) {
                planner->state = S_CURVE_T4;
                planner->step_counter = 0;
                planner->current_acc = 0.0f;
                planner->current_jerk = 0.0f;
            } else {
                // a = a_max - J*t (相对于T1结束时刻)
                planner->current_jerk = -dir * planner->j_max; // 负Jerk
                planner->current_acc = dir * planner->a_max - dir * planner->j_max * t;
                planner->current_vel = planner->current_vel + planner->current_acc * dt;
                planner->current_pos = planner->current_pos + planner->current_vel * dt 
                                     + 0.5f * planner->current_acc * dt * dt;
                planner->step_counter++;
            }
            break;
        }
        
        case S_CURVE_T4: {  // 匀速段: a=0, v=v_max
            float t = planner->step_counter * dt;
            float T = planner->T4;
            
            if (t >= T || T < dt) {
                planner->state = S_CURVE_T5;
                planner->step_counter = 0;
                planner->current_acc = 0.0f;
            } else {
                planner->current_jerk = 0.0f;
                planner->current_acc = 0.0f;
                planner->current_vel = dir * planner->v_max;
                planner->current_pos = planner->current_pos + planner->current_vel * dt;
                planner->step_counter++;
            }
            break;
        }
        
        case S_CURVE_T5: {  // 加减速段: a从0减到-a_max (负向增大)
            float t = planner->step_counter * dt;
            float T = planner->T5;
            
            if (t >= T) {
                planner->state = S_CURVE_T6;
                planner->step_counter = 0;
                planner->current_acc = -dir * planner->a_max;
                planner->current_jerk = 0.0f;
            } else {
                planner->current_jerk = -dir * planner->j_max; // 负Jerk
                planner->current_acc = -dir * planner->j_max * t;
                planner->current_vel = planner->current_vel + planner->current_acc * dt;
                planner->current_pos = planner->current_pos + planner->current_vel * dt 
                                     + 0.5f * planner->current_acc * dt * dt;
                planner->step_counter++;
            }
            break;
        }
        
        case S_CURVE_T6: {  // 匀减速段: a = -a_max
            float t = planner->step_counter * dt;
            float T = planner->T6;
            
            if (t >= T || T < dt) {
                planner->state = S_CURVE_T7;
                planner->step_counter = 0;
            } else {
                planner->current_jerk = 0.0f;
                planner->current_acc = -dir * planner->a_max;
                planner->current_vel = planner->current_vel - dir * planner->a_max * dt;
                planner->current_pos = planner->current_pos + planner->current_vel * dt 
                                     + 0.5f * planner->current_acc * dt * dt;
                planner->step_counter++;
            }
            break;
        }
        
        case S_CURVE_T7: {  // 减减速段: a从-a_max增到0
            float t = planner->step_counter * dt;
            float T = planner->T7;
            
            if (t >= T) {
                // 完成
                planner->state = S_CURVE_IDLE;
                planner->current_pos = planner->target_pos; // 精确到位
                planner->current_vel = 0.0f;
                planner->current_acc = 0.0f;
                planner->current_jerk = 0.0f;
                planner->is_busy = 0;
            } else {
                planner->current_jerk = dir * planner->j_max; // 正Jerk
                planner->current_acc = -dir * planner->a_max + dir * planner->j_max * t;
                planner->current_vel = planner->current_vel + planner->current_acc * dt;
                // 最后一段用简单积分避免残余误差
                float step = planner->current_vel * dt + 0.5f * planner->current_acc * dt * dt;
                planner->current_pos = planner->current_pos + step;
                planner->step_counter++;
            }
            break;
        }
        
        default:
            break;
    }
    
    return planner->current_pos;
}
