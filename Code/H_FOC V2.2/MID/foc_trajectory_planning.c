/**
 * @file foc_trajectory_planning.c
 * @brief S型曲线轨迹规划器实现（6次多项式版本）
 * 
 * 实现基于6次多项式的S型曲线轨迹规划，将传统的7段S曲线
 * （加加速-匀加速-减加速-匀速-加减速-匀减速-减减速）简化为3段：
 * - 加速段（ACCEL）：T1+T2+T3 合并为6次多项式
 * - 匀速段（COAST）：T4 线性插值  
 * - 减速段（DECEL）：T5+T6+T7 合并为6次多项式
 * 
 * 数学基础：
 * - 位置公式：p(t) = p0 + T * (A*t^6 + B*t^5 + C*t^4 + F*t)
 * - 速度公式：v(t) = 6A*t^5 + 5B*t^4 + 4C*t^3 + F
 * - 系数满足边界条件：t=0时v=v_start, t=1时v=v_end, 加速度连续且两端为0
 */

#include "foc_trajectory_planning.h"

/**
 * @brief 初始化S曲线规划器
 * @param planner 规划器结构体指针
 * @param vmax    最大速度限制 (单位：deg/s 或 rad/s)
 * @param amax    最大加速度限制 (单位：deg/s²)
 * @param jmax    最大加加速度(Jerk)限制 (单位：deg/s³)
 * @param freq_t  控制周期 (单位：s，如1kHz则填0.001)
 * 
 * @note 初始化后状态为 IDLE，需调用 set_target 启动规划
 */
void s_curve_planner_init(SCurve_Planner_t *planner, float vmax, float amax, float jmax, float freq_t)
{
    planner->v_max = vmax;
    planner->a_max = amax;
    planner->j_max = jmax;
    planner->freq_t = freq_t;
    planner->state = S_CURVE_IDLE;
    planner->is_busy = 0;
}

/**
 * @brief 设置新的目标位置并生成轨迹规划
 * @param planner     规划器结构体指针
 * @param current_pos 当前实际位置（编码器反馈值）
 * @param target_pos  目标位置
 * @return 0: 成功启动规划, 1: 距离过短无需规划
 * 
 * @details 算法流程：
 * 1. 计算理论7段S曲线参数（T1加加速段，T2匀加速段）
 * 2. 判断是否能达到设定速度v_max：
 *    - 长距离：梯形速度曲线（有匀速段）
 *    - 短距离：三角形速度曲线（无匀速段，重新计算v_peak）
 * 3. 计算6次多项式系数 A1,B1,C1,F1（加速）和 A2,B2,C2,F2（减速）
 * 
 * 系数推导（加速段 0→v_peak）：
 * - 设归一化时间 τ = t/t_acc ∈ [0,1]
 * - 速度曲线：v(τ) = 6A·τ^5 + 5B·τ^4 + 4C·τ^3 + F
 * - 边界条件：v(0)=0, v(1)=v_peak, v'(0)=0, v'(1)=0
 * - 解得：A = v_peak, B = -3v_peak, C = 2.5v_peak, F = 0
 * 
 * 位移计算：
 * - s(τ) = t_acc * ∫v dτ = t_acc * (A·τ^6 + B·τ^5 + C·τ^4 + F·τ)
 */
uint8_t s_curve_set_target(SCurve_Planner_t *planner, float current_pos, float target_pos)
{
    float dist = target_pos - current_pos;
    float S = fabsf(dist);
    
    /* 距离过短，直接视为到达 */
    if (S < 0.001f) {
        planner->state = S_CURVE_IDLE;
        planner->is_busy = 0;
        return 0;
    }
    
    /* 确定运动方向：+1.0为正转，-1.0为反转 */
    float dir = (dist > 0.0f) ? 1.0f : -1.0f;
    planner->direction = dir;
    planner->pos_init = current_pos;
    planner->pos_end = target_pos;
    planner->is_busy = 1;
    planner->tick = 0;  /* 全局时间计数器，不随阶段重置 */
    
    float v_max_set = planner->v_max;
    float a_max = planner->a_max;
    float j_max = planner->j_max;
    
    /* ========== 步骤1：计算理论S曲线参数 ========== */
    /* T1: 加加速段时间（Jerk从0加到J_max所需时间） */
    float T1 = a_max / j_max;
    /* v1: T1结束时的速度（加加速段贡献的速度） */
    float v1 = 0.5f * j_max * T1 * T1;
    /* T2: 匀加速段时间（若v_max足够大） */
    float T2 = (v_max_set - 2.0f * v1) / a_max;
    
    float v_peak, t_acc;
    
    if (T2 >= 0) {
        /* 情况A：能达到设定速度v_max，梯形速度曲线 */
        v_peak = v_max_set;
        t_acc = 2.0f * T1 + T2;  /* T1(加加速) + T2(匀加速) + T3(减加速=T1) */
    } else {
        /* 情况B：达不到v_max，纯S型（三角形速度曲线） */
        /* 重新计算T1：由 S = j_max * T1^3 推导（纯S型总位移公式） */
        T1 = cbrtf(S / j_max);
        t_acc = 2.0f * T1;       /* 纯加减速，无匀加速段 */
        v_peak = 0.5f * j_max * T1 * T1;
    }
    
    /* ========== 步骤2：距离约束检查 ========== */
    /* 理论加减速总位移（对称三角形面积）：dist_needed = v_peak * t_acc */
    float dist_needed = v_peak * t_acc;
    
    if (dist_needed > S) {
        /* 短行程：无法达到v_peak，重新规划三角形轨迹 */
        /* 由 S = 0.125 * j_max * t_acc^3 反解 t_acc */
        t_acc = 2.0f * cbrtf(S / j_max);
        v_peak = S / t_acc;      /* 三角形面积公式：S = 0.5 * base * height */
        planner->t_vel = 0.0f;   /* 无匀速段 */
    } else {
        /* 长行程：有匀速段 */
        planner->t_vel = (S - dist_needed) / v_peak;
    }
    
    /* ========== 步骤3：保存运动参数 ========== */
    planner->vel_max = dir * v_peak;    /* 带方向的最大速度 */
    planner->t_acc = t_acc;             /* 加速段时间 */
    planner->t_dec = t_acc;             /* 减速段时间（对称） */
    planner->t_total = t_acc + planner->t_vel + t_acc;  /* 总时间 */
    
    /* 关键路点位置（用于阶段切换时强制对齐，消除累积误差） */
    planner->pos_accel_end = current_pos + dir * (0.5f * v_peak * t_acc);
    planner->pos_coast_end = planner->pos_accel_end + dir * (v_peak * planner->t_vel);
    
    /* ========== 步骤4：计算6次多项式系数 ========== */
    /* 加速段系数（0 → v_peak） */
    float v_diff = v_peak;
    planner->A1 = v_diff;           /* 6次项系数 */
    planner->B1 = -3.0f * v_diff;   /* 5次项系数 */
    planner->C1 = 2.5f * v_diff;    /* 4次项系数 */
    planner->F1 = 0.0f;             /* 1次项系数（初始速度） */
    
    /* 减速段系数（v_peak → 0） */
    planner->A2 = -v_diff;          /* 6次项系数 */
    planner->B2 = 3.0f * v_diff;    /* 5次项系数 */
    planner->C2 = -2.5f * v_diff;   /* 4次项系数 */
    planner->F2 = v_diff;           /* 1次项系数（初始速度=v_peak） */
    
    planner->state = S_CURVE_ACCEL;
    
    return 0;
}

/**
 * @brief S曲线轨迹更新函数（周期性调用）
 * @param planner 规划器结构体指针
 * @return 当前规划的目标位置
 * 
 * @warning 必须在固定周期调用（如1kHz），时间基准由 freq_t 决定
 * 
 * 状态机：
 * - ACCEL：加速段，使用6次多项式插值，t ∈ [0, t_acc]
 * - COAST：匀速段，线性插值，t ∈ [t_acc, t_acc+t_vel]
 * - DECEL：减速段，使用6次多项式插值，t ∈ [t_acc+t_vel, t_total]
 * - IDLE：规划完成，保持最终位置
 */
float s_curve_update(SCurve_Planner_t *planner)
{
    /* 规划完成，保持最终位置 */
    if (planner->state == S_CURVE_IDLE) {
        return planner->current_pos;
    }
    
    float dt = planner->freq_t;
    planner->tick++;        /* 全局tick递增，不重置，确保时间连续 */
    float t = planner->tick * dt;  /* 绝对时间 */
    
    switch (planner->state) {
        case S_CURVE_ACCEL: {
            /* 检查是否进入匀速段或减速段 */
            if (t >= planner->t_acc) {
                /* 阶段切换：强制对齐到理论路点，消除积分误差 */
                planner->state = (planner->t_vel > dt) ? S_CURVE_COAST : S_CURVE_DECEL;
                planner->current_pos = planner->pos_accel_end;
                planner->current_vel = planner->vel_max;
                planner->current_acc = 0.0f;
            } else {
                /* 归一化时间 τ = t / t_acc ∈ [0,1] */
                float t_ratio = t / planner->t_acc;
                t_ratio = fminf(t_ratio, 1.0f);  /* 限幅保护，防止数值溢出 */
                
                /* 预计算幂次，减少乘法运算 */
                float t2 = t_ratio * t_ratio;
                float t3 = t2 * t_ratio;
                float t4 = t3 * t_ratio;
                float t5 = t4 * t_ratio;
                float t6 = t5 * t_ratio;
                
                /* 计算相对位移：Δs = t_acc * (A1·τ^6 + B1·τ^5 + C1·τ^4 + F1·τ) */
                float delta_pos = planner->t_acc * (planner->A1 * t6 + planner->B1 * t5 + planner->C1 * t4 + planner->F1 * t_ratio);
                planner->current_pos = planner->pos_init + planner->direction * delta_pos;
                
                /* 计算速度：v = d(s)/dt = (ds/dτ) * (dτ/dt) = (多项式) / t_acc */
                planner->current_vel = planner->direction * (6.0f * planner->A1 * t5 + 5.0f * planner->B1 * t4 + 4.0f * planner->C1 * t3 + planner->F1);
                
                /* 计算加速度：a = dv/dt */
                planner->current_acc = planner->direction * (30.0f * planner->A1 * t4 + 20.0f * planner->B1 * t3 + 12.0f * planner->C1 * t2) / planner->t_acc;
            }
            break;
        }
        
        case S_CURVE_COAST: {
            /* 检查是否进入减速段 */
            if (t >= planner->t_acc + planner->t_vel) {
                planner->state = S_CURVE_DECEL;
                planner->current_pos = planner->pos_coast_end;
            } else {
                /* 匀速段：线性插值 p = p_accel_end + v_max * (t - t_acc) */
                float t_coast = t - planner->t_acc;  /* 在匀速段内的时间 */
                planner->current_pos = planner->pos_accel_end + planner->vel_max * t_coast;
                planner->current_vel = planner->vel_max;
                planner->current_acc = 0.0f;
            }
            break;
        }
        
        case S_CURVE_DECEL: {
            /* Tb: 减速开始绝对时间, Tc: 结束绝对时间 */
            float Tb = planner->t_acc + planner->t_vel;
            float Tc = planner->t_total;
            
            /* 检查是否到达终点 */
            if (t >= Tc) {
                planner->state = S_CURVE_IDLE;
                planner->current_pos = planner->pos_end;  /* 精确到达目标 */
                planner->current_vel = 0.0f;
                planner->current_acc = 0.0f;
                planner->is_busy = 0;
            } else {
                /* 归一化时间 y = (t - Tb) / t_dec ∈ [0,1] */
                float t_dec = t - Tb;
                float T_dec = planner->t_dec;
                float y = t_dec / T_dec;
                y = fminf(y, 1.0f);  /* 限幅保护 */
                
                /* 预计算幂次 */
                float y2 = y * y;
                float y3 = y2 * y;
                float y4 = y3 * y;
                float y5 = y4 * y;
                float y6 = y5 * y;
                
                /* 计算相对位移（从pos_coast_end开始） */
                float delta_pos = T_dec * (planner->A2 * y6 + planner->B2 * y5 + planner->C2 * y4 + planner->F2 * y);
                planner->current_pos = planner->pos_coast_end + planner->direction * delta_pos;
                
                /* 计算速度（递减至0） */
                planner->current_vel = planner->direction * (6.0f * planner->A2 * y5 + 5.0f * planner->B2 * y4 + 4.0f * planner->C2 * y3 + planner->F2);
                
                /* 计算加速度 */
                planner->current_acc = planner->direction * (30.0f * planner->A2 * y4 + 20.0f * planner->B2 * y3 + 12.0f * planner->C2 * y2) / T_dec;
            }
            break;
        }
        
        default:
            break;
    }
    
    return planner->current_pos;
}
