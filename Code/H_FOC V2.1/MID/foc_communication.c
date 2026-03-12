#include "foc_communication.h"

uart_parsed_data_t* data; // 串口解析数据
uint8_t g_local_node_id = JOINT_ID;  // 本关节ID
joint_status_t remote_joint_status[JOINT_ID_MAX];  // 存储其他关节的状态

/**
 * @brief 发送当前关节状态
 */
void CAN_ReportStatus(float pos, float vel)
{
    joint_status_t status;
    
    // 浮点转定点（当前只发送位置与速度）
    status.position = FLOAT_TO_FIX(pos);
    status.velocity = FLOAT_TO_FIX(vel);
    
    FDCAN_SendJointStatus(g_local_node_id, &status);
}

/**
 * @brief 解析并执行控制指令
 */
void CAN_ExecuteCommand(joint_control_t *cmd)
{
    float target_pos = FIX_TO_FLOAT(cmd->target_pos);
    float target_cur = FIX16_TO_FLOAT(cmd->target_cur);
    // debug_log("pos=%.4f cur=%.2f mode=%u",
    //           target_pos,
    //           target_cur,
    //           cmd->control_mode);
    
    switch (cmd->control_mode) {
        case CONTROL_MODE_POSITION:
            foc_ctrl.target_position = target_pos;
            break;
        case CONTROL_MODE_SPEED:

            break;
        case CONTROL_MODE_CURRENT:
            foc_ctrl.target_q = target_cur;
            break;
        default:
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
            if (src >= JOINT_ID_MAX) {
                return;
            }
            if (frame->len >= sizeof(joint_status_t))
            {
                memcpy(&remote_joint_status[src], frame->data, sizeof(joint_status_t));
                    
                joint_status_t *status = (joint_status_t*)frame->data;
                // float pos_f = FIX_TO_FLOAT(status->position);
                // float vel_f = FIX_TO_FLOAT(status->velocity);
                // debug_log("%d, %.4f, %.4f", src, pos_f, vel_f);

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
    FDCAN_Service();
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
