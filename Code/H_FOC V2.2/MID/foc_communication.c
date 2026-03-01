#include "foc_communication.h"

uart_parsed_data_t* data; // 串口解析数据
uint8_t g_local_node_id = 1;  // 本关节ID

/**
 * @brief 发送当前关节状态（在1kHz控制循环中调用）
 */
void CAN_ReportStatus(float pos, float vel, float cur, float temp)
{
    joint_status_t status;
    
    // 浮点转定点（注意单位转换）
    status.position = FLOAT_TO_FIX(pos, POS_SCALE);      // 度 -> 0.0001°/LSB
    status.velocity = FLOAT_TO_FIX(vel, VEL_SCALE);      // rpm -> 0.001rpm/LSB
    status.current  = FLOAT_TO_FIX16(cur, CUR_SCALE);    // A -> 0.001A/LSB (用int16节省空间)
    status.temperature = FLOAT_TO_FIX16(temp, TEMP_SCALE);
    status.status = 0x0001;  // bit0: 使能状态
    status.error_code = 0;
    
    FDCAN_SendJointStatus(g_local_node_id, &status);
}

/**
 * @brief 解析并执行控制指令（在接收处理中调用）
 */
void CAN_ExecuteCommand(joint_control_t *cmd)
{
    float target_pos = FIX_TO_FLOAT(cmd->target_pos, POS_SCALE);
    float target_vel = FIX_TO_FLOAT(cmd->target_vel, VEL_SCALE);
    float target_cur = FIX_TO_FLOAT(cmd->target_cur, CUR_SCALE);
    
    switch (cmd->control_mode) {
        case 0:  // 位置模式
            // foc_set_position(target_pos);
            break;
        case 1:  // 速度模式
            // foc_set_velocity(target_vel);
            break;
        case 2:  // 力矩模式（电流）
            // foc_set_current(target_cur);
            break;
    }
}

/**
 * @brief 主循环调用（1ms周期）
 */
void CAN_Process(void)
{
    // 处理接收队列
    FDCAN_ProcessRxQueue();
    
}

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
