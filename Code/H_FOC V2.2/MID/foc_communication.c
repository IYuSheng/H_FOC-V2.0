#include "foc_communication.h"

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
