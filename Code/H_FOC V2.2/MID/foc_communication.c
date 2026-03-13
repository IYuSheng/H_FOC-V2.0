#include "foc_communication.h"
#include <string.h>

uart_parsed_data_t* data;
uint8_t g_local_node_id = JOINT_ID;
joint_status_t remote_joint_status[JOINT_ID_MAX];
canfd_joint_cmd_t g_canfd_joint_cmd = {0};

void CAN_ReportStatus(void)
{
    joint_status_t status;

    status.position = FLOAT_TO_FIX(encoder_data.mechanical_angle);
    status.velocity = FLOAT_TO_FIX(encoder_data.mechanical_speed);
    status.current = FLOAT_TO_FIX(foc_ctrl.abc_dq.current_q_filt);
    status.temperature = FLOAT_TO_FIX(foc_voltage_data.temp);
    status.acceleration = FLOAT_TO_FIX(encoder_data.mechanical_accsd);

    FDCAN_SendJointStatus_Master(g_local_node_id, &status);
}

void CAN_ExecuteCommand(joint_control_t *cmd)
{
    memcpy(&joint_control, cmd, sizeof(joint_control));

    g_canfd_joint_cmd.target_current = FIX_TO_FLOAT(cmd->target_current);
    g_canfd_joint_cmd.target_angle = FIX_TO_FLOAT(cmd->target_angle);
    g_canfd_joint_cmd.target_velocity = FIX_TO_FLOAT(cmd->target_velocity);
    g_canfd_joint_cmd.target_acceleration = FIX_TO_FLOAT(cmd->target_acceleration);
    g_canfd_joint_cmd.target_jerk = FIX_TO_FLOAT(cmd->target_jerk);
    g_canfd_joint_cmd.stiffness = FIX_TO_FLOAT(cmd->stiffness);
    g_canfd_joint_cmd.damping = FIX_TO_FLOAT(cmd->damping);
    g_canfd_joint_cmd.control_mode = cmd->control_mode;
    g_canfd_joint_cmd.enabled = 1U;

    foc_ctrl.target_position = g_canfd_joint_cmd.target_angle;
    foc_ctrl.target_speed = g_canfd_joint_cmd.target_velocity;
}

void FDCAN_RxCallback(can_frame_t *frame)
{
    uint8_t type = CAN_ID_GET_TYPE(frame->id);
    uint8_t src = CAN_ID_GET_SRC(frame->id);

    switch (type) {
        case MSG_TYPE_CONTROL:
            if (src != JOINT_ID_MASTER) {
                return;
            }
            if (frame->len >= sizeof(joint_control_t)) {
                joint_control_t *cmd = (joint_control_t *)frame->data;
                CAN_ExecuteCommand(cmd);
            }
            break;

        case MSG_TYPE_STATUS:
            if (src >= JOINT_ID_MAX) {
                return;
            }
            if (frame->len >= sizeof(joint_status_t)) {
                joint_status_t *status = (joint_status_t *)frame->data;

                memcpy(&remote_joint_status[src], frame->data, sizeof(joint_status_t));
                foc_ctrl.motor2_data.motor2_mechanical_angle = FIX_TO_FLOAT(status->position);
                foc_ctrl.motor2_data.motor2_mechanical_speed = FIX_TO_FLOAT(status->velocity);
            }
            break;

        default:
            break;
    }
}

void CAN_Process(void)
{
    FDCAN_ProcessRxQueue();
    FDCAN_Service();
}

void foc_control_set(void)
{
    data = get_parsed_data();

    if (data->t_q12_valid) {
        g_canfd_joint_cmd.enabled = 0U;
        foc_ctrl.target_q = data->q1;
        cart_out.iq2 = data->q2;
        data->t_q12_valid = 0;
    }
    if (data->target_position_valid) {
        g_canfd_joint_cmd.enabled = 0U;
        foc_ctrl.target_position = data->target_position;
        data->target_position_valid = 0;
    }
    if (data->target_speed_valid) {
        g_canfd_joint_cmd.enabled = 0U;
        foc_ctrl.target_speed = data->target_speed;
        data->target_speed_valid = 0;
    }
    if (data->current_q_valid) {
        g_canfd_joint_cmd.enabled = 0U;
        data->current_q_valid = 0;
    }
    if (data->target_q_valid) {
        g_canfd_joint_cmd.enabled = 0U;
        foc_ctrl.target_q = data->target_q;
        data->target_q_valid = 0;
    }
}
