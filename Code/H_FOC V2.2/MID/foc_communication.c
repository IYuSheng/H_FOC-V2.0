#include "foc_communication.h"

uart_parsed_data_t* data; // 串口解析数据
uint8_t g_local_node_id = JOINT_ID;  // 本关节ID
joint_status_t remote_joint_status[JOINT_ID_MAX];  // 存储其他关节的状态

/**
 * @brief 发送当前关节状态
 */
void CAN_ReportStatus(float pos, float vel, float cur, float temp)
{
    joint_status_t status;
    
    // 浮点转定点（注意单位转换）
    status.position = FLOAT_TO_FIX(pos);
    status.velocity = FLOAT_TO_FIX(vel);
    status.current  = FLOAT_TO_FIX16(cur);
    status.temperature = FLOAT_TO_FIX16(temp);
    status.status = 0x0001;  // bit0: 使能状态
    status.error_code = 0;
    
    FDCAN_SendJointStatus(g_local_node_id, &status);
}

/**
 * @brief 解析并执行控制指令
 */
void CAN_ExecuteCommand(joint_control_t *cmd)
{
    float target_pos = FIX_TO_FLOAT(cmd->target_pos);
    float target_vel = FIX_TO_FLOAT(cmd->target_vel);
    float target_cur = FIX16_TO_FLOAT(cmd->target_cur);

    debug_log("%.4f", target_pos);
    
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

void FDCAN_RxCallback(can_frame_t *frame)
{
    uint8_t type = CAN_ID_GET_TYPE(frame->id);

    uint8_t src = CAN_ID_GET_SRC(frame->id);
    
    switch (type) {
        case MSG_TYPE_CONTROL:
        
            // 非主节点发送控制指令，不执行控制
            if (src != JOINT_ID_MASTER) return;
            // 检查数据长度是否足够
            if (frame->len >= sizeof(joint_control_t)) {
                joint_control_t *cmd = (joint_control_t*)frame->data;
                CAN_ExecuteCommand(cmd);
            }
            break;

        case MSG_TYPE_STATUS:
            if (frame->len >= sizeof(joint_status_t))
            {
                memcpy(&remote_joint_status[src], frame->data, sizeof(joint_status_t));
                    
                joint_status_t *status = (joint_status_t*)frame->data;
                // float pos_f = FIX_TO_FLOAT(status->position);
                // float vel_f = FIX_TO_FLOAT(status->velocity);
                // float cur_f = FIX16_TO_FLOAT(status->current);
                // float temp_f = FIX16_TO_FLOAT(status->temperature);

                // debug_log("%d, %.4f, %.4f, %.4f, %.4f",
                //           src, pos_f, vel_f, cur_f, temp_f);

                foc_ctrl.motor2_data.motor2_mechanical_angle = FIX_TO_FLOAT(status->position);
                foc_ctrl.motor2_data.motor2_mechanical_speed = FIX_TO_FLOAT(status->velocity);
            }
            break;

        default:
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
    if(data->t_q12_valid) {
        foc_ctrl.target_q = data->q1;
        cart_out.iq2 = data->q2;
        data->t_q12_valid = 0;
    }
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
