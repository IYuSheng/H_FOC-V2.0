#ifndef __FOC_PARSE_H
#define __FOC_PARSE_H

#include <stdint.h>

// 定义用于存储解析数据的结构体
typedef struct {
    float target_position;
    uint8_t target_position_valid;  // 标记该值是否已接收
    float target_speed;
    uint8_t target_speed_valid;
    float current_q;
    uint8_t current_q_valid;
    float target_q;
    uint8_t target_q_valid;
} uart_parsed_data_t;

// 解析接收到的数据
void parse_received_data(char* data, uint8_t len);

// 获取解析后的数据
uart_parsed_data_t* get_parsed_data(void);

#endif // __FOC_PARSE_H
