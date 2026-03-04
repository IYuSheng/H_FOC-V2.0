#include "foc_communication.h"
#include <string.h>

uart_parsed_data_t *data;
uint8_t g_local_node_id = JOINT_ID;
joint_status_t remote_joint_status[JOINT_NODE_COUNT];

void CAN_ReportStatus(float pos, float vel)
{
    joint_status_t status;
    status.position = FLOAT_TO_FIX(pos);
    status.velocity = FLOAT_TO_FIX(vel);

    (void)FDCAN_SendJointStatus(g_local_node_id, &status);
}

static void CAN_ExecuteCommand(joint_control_t *cmd)
{
    float target_pos = FIX_TO_FLOAT(cmd->target_pos);
    float target_vel = FIX_TO_FLOAT(cmd->target_vel);
    float target_cur = FIX16_TO_FLOAT(cmd->target_cur);

    switch (cmd->control_mode) {
        case 0:
            /* Position mode */
            /* foc_set_position(target_pos); */
            (void)target_pos;
            break;
        case 1:
            /* Speed mode */
            /* foc_set_velocity(target_vel); */
            (void)target_vel;
            break;
        case 2:
            /* Torque mode (Iq command) */
            foc_ctrl.target_q = target_cur;
            break;
        default:
            break;
    }
}

void FDCAN_RxCallback(can_frame_t *frame)
{
    uint8_t type = (uint8_t)CAN_ID_GET_TYPE(frame->id);

    switch (type) {
        case MSG_TYPE_CONTROL:
            /* In classic CAN single frame, 12-byte joint_control_t is not expected. */
            if (frame->len >= sizeof(joint_control_t)) {
                CAN_ExecuteCommand((joint_control_t *)frame->data);
            }
            break;

        case MSG_TYPE_STATUS:
            if (frame->len == sizeof(joint_status_t)) {
                uint8_t src = (uint8_t)CAN_ID_GET_SRC(frame->id);
                if (src <= JOINT_ID_MAX) {
                    memcpy(&remote_joint_status[src], frame->data, sizeof(joint_status_t));

                    /* Optional decode for debug */
                    joint_status_t *status = (joint_status_t *)frame->data;
                    float pos_f = FIX_TO_FLOAT(status->position);
                    float vel_f = FIX_TO_FLOAT(status->velocity);
                    (void)pos_f;
                    (void)vel_f;
                    /* debug_log("%u, %.4f, %.4f", src, pos_f, vel_f); */
                }
            }
            break;

        default:
            break;
    }
}

void CAN_Process(void)
{
    FDCAN_ProcessRxQueue();
}

void foc_control_set(void)
{
    data = get_parsed_data();

    if (data->target_position_valid) {
        foc_ctrl.target_position = data->target_position;
        data->target_position_valid = 0;
    }

    if (data->target_speed_valid) {
        foc_ctrl.target_speed = data->target_speed;
        data->target_speed_valid = 0;
    }

    if (data->current_q_valid) {
        data->current_q_valid = 0;
    }

    if (data->target_q_valid) {
        foc_ctrl.target_q = data->target_q;
        data->target_q_valid = 0;
    }
}
