#include "foc_prase.h"
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "foc_conversion.h"

// 全局变量存储解析的数据
static uart_parsed_data_t parsed_data = {0};

/**
 * @brief 解析接收到的数据
 * @param data 接收到的字符串数据
 * @param len 数据长度
 */
void parse_received_data(char* data, uint8_t len)
{
    (void)len; // 避免未使用参数警告
    // debug_log("Received data: %s", data);
    // 检查是否包含 "target_position:" 字符串
    char* tpos_start = strstr(data, "target_position:");
    if(tpos_start != NULL) {
        // 找到冒号后的位置
        tpos_start += strlen("target_position:");
        
        // 提取冒号后的浮点数
        char* end_ptr;
        float value = strtof(tpos_start, &end_ptr);
        
        // 检查是否成功解析了一个数字
        if(end_ptr != tpos_start) {
            parsed_data.target_position = value;
            parsed_data.target_position_valid = 1;
        }
    }
    
    // 解析 target_speed
    char* mangle_start = strstr(data, "target_speed:");
    if(mangle_start != NULL) {
        mangle_start += strlen("target_speed:");
        char* end_ptr;
        float value = strtof(mangle_start, &end_ptr);
        
        if(end_ptr != mangle_start) {
            parsed_data.target_speed = value;
            parsed_data.target_speed_valid = 1;
        }
    }
    
    // 解析 current_q
    char* cq_start = strstr(data, "current_q:");
    if(cq_start != NULL) {
        cq_start += strlen("current_q:");
        char* end_ptr;
        float value = strtof(cq_start, &end_ptr);
        
        if(end_ptr != cq_start) {
            parsed_data.current_q = value;
            parsed_data.current_q_valid = 1;
        }
    }
    
    // 解析 target_q
    char* tq_start = strstr(data, "target_q:");
    if(tq_start != NULL) {
        tq_start += strlen("target_q:");
        char* end_ptr;
        float value = strtof(tq_start, &end_ptr);
        
        if(end_ptr != tq_start) {
            parsed_data.target_q = value;
            parsed_data.target_q_valid = 1;
        }
    }
    // 解析 "MIT:" 其他电机数据
    if (data[0] == 'M' && data[1] == 'I' && data[2] == 'T' && data[3] == ':') {
        // 提取电角度、机械角度、机械速度
        float electrical_angle = (float)((data[4] | (data[5] << 8)) / 10000.0);
        float mechanical_angle = (float)((data[6] | (data[7] << 8)) / 10000.0);
        float mechanical_speed = (float)((data[8] | (data[9] << 8)) / 10000.0);

        // 保存解析的值
        foc_ctrl.motor2_data.motor2_electrical_angle = electrical_angle;
        foc_ctrl.motor2_data.motor2_mechanical_angle = mechanical_angle;
        foc_ctrl.motor2_data.motor2_mechanical_speed = mechanical_speed;
        
        // debug_log("1");

        // 进行后续的控制计算（如阻抗控制等）
    }
}

/**
 * @brief 获取解析后的数据
 * @return 返回解析数据的指针
 */
uart_parsed_data_t* get_parsed_data(void)
{
    return &parsed_data;
}
