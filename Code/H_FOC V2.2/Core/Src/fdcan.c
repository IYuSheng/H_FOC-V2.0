#include "fdcan.h"
#include "string.h"
#include "stdio.h"

/* ===================== 私有变量 ===================== */

FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[64];  // 发送缓冲区

// 接收环形队列
static volatile can_frame_t rx_queue[FDCAN_RX_QUEUE_SIZE];
static volatile uint8_t rx_write_idx = 0;
static volatile uint8_t rx_read_idx = 0;
static volatile uint32_t rx_drop_count = 0;

/* ===================== 内部函数 ===================== */

/**
 * @brief 长度转DLC
 */
static uint32_t len_to_dlc(uint8_t len)
{
    if (len <= 8) return len;
    else if (len <= 12) return FDCAN_DLC_BYTES_12;
    else if (len <= 16) return FDCAN_DLC_BYTES_16;
    else if (len <= 20) return FDCAN_DLC_BYTES_20;
    else if (len <= 24) return FDCAN_DLC_BYTES_24;
    else if (len <= 32) return FDCAN_DLC_BYTES_32;
    else if (len <= 48) return FDCAN_DLC_BYTES_48;
    else return FDCAN_DLC_BYTES_64;
}

/**
 * @brief DLC转长度
 */
static uint8_t dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}

/* ===================== 初始化函数 ===================== */

void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    
    // 时钟配置：170MHz（假设PCLK1=170M）
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    
    // 启用CAN FD + BRS（数据域加速）
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    
    // 仲裁域：1Mbps (170MHz / (10 * 17))
    // 采样点：82.35% ((1+13)/17)
    hfdcan1.Init.NominalPrescaler = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 2;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 3;
    
    // 数据域：5Mbps (170MHz / (2 * 17))
    // 采样点：82.35%
    hfdcan1.Init.DataPrescaler = 2;
    hfdcan1.Init.DataSyncJumpWidth = 2;
    hfdcan1.Init.DataTimeSeg1 = 13;
    hfdcan1.Init.DataTimeSeg2 = 3;
    
    // 过滤器配置
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
    FDCAN_Config_Filter();
    FDCAN_Start();
}

/**
 * @brief 配置接收过滤器
 * @note 接收所有标准帧（软件过滤更灵活）
 */
void FDCAN_Config_Filter(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    
    // 标准ID掩码模式：接收所有帧（0x000 & 0x000 = 接收所有）
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;  // 接收所有ID
    sFilterConfig.FilterID2 = 0x000;  // 掩码全0表示忽略所有位
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // 全局过滤器：拒绝非匹配帧和远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                                FDCAN_REJECT,           // 拒绝非匹配标准帧
                                FDCAN_REJECT,           // 拒绝非匹配扩展帧
                                FDCAN_REJECT_REMOTE,    // 拒绝标准远程帧
                                FDCAN_REJECT_REMOTE);   // 拒绝扩展远程帧
}

void FDCAN_Start(void)
{
    // 启动FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
    // 开启FIFO0新消息中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== 发送函数 ===================== */

/**
 * @brief 通用数据包发送（支持64字节）
 * @param src_id   源节点ID (0-15)
 * @param dst_id   目标节点ID (0-15, 0x0F为广播)
 * @param msg_type 消息类型 (MSG_TYPE_XXX)
 * @param data     数据指针
 * @param len      数据长度 (0-64字节)
 * @return 0:成功, -1:失败
 */
int8_t FDCAN_SendPacket(uint8_t src_id, uint8_t dst_id, uint8_t msg_type, 
                        uint8_t *data, uint8_t len)
{
    if (len > 64) return -1;
    
    // 检查TX FIFO空间
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        return -1;  // 队列满
    }
    
    // 构造帧ID: [SrcID(4bit)][DstID(4bit)][Type(4bit)][Reserve(3bit)]
    uint32_t can_id = CAN_ID_MAKE(src_id, dst_id, msg_type);
    
    // 配置发送头
    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;        // 开启数据域加速
    tx_header.FDFormat = FDCAN_FD_CAN;             // CAN FD模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    // 复制数据到发送缓冲区
    if (len > 0 && data != NULL) {
        memcpy(tx_data, data, len);
    }
    
    // 发送（填充剩余字节为0，避免未定义数据）
    if (len < 64 && len > 8) {
        memset(&tx_data[len], 0, 64 - len);
    }
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
        return -1;
    }
    
    return 0;
}

/**
 * @brief 发送关节状态（广播自身状态）
 * @param joint_id 本关节ID
 * @param status   状态数据结构
 * @return 0:成功
 */
int8_t FDCAN_SendJointStatus(uint8_t joint_id, joint_status_t *status)
{
    // 直接发送结构体（已pack）
    return FDCAN_SendPacket(joint_id, JOINT_ID_BROADCAST, MSG_TYPE_STATUS,
                           (uint8_t*)status, sizeof(joint_status_t));
}

/**
 * @brief 发送控制指令（主节点调用）
 * @param dst_id 目标关节ID
 * @param cmd    控制指令结构
 * @return 0:成功
 */
int8_t FDCAN_SendControlCmd(uint8_t dst_id, joint_control_t *cmd)
{
    return FDCAN_SendPacket(JOINT_ID_MASTER, dst_id, MSG_TYPE_CONTROL,
                           (uint8_t*)cmd, sizeof(joint_control_t));
}

/* ===================== 接收处理 ===================== */

/**
 * @brief 接收中断回调（ISR中调用）
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) return;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
    
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
        return;
    }
    
    // 计算下一个写索引
    uint8_t next_write = (rx_write_idx + 1) % FDCAN_RX_QUEUE_SIZE;
    
    // 检查队列满
    if (next_write == rx_read_idx) {
        rx_drop_count++;
        return;
    }
    
    // 填充队列
    can_frame_t *frame = (can_frame_t*)&rx_queue[rx_write_idx];
    frame->id = header.Identifier;
    frame->is_extended = (header.IdType == FDCAN_EXTENDED_ID);
    frame->len = dlc_to_len(header.DataLength);
    memcpy((void*)frame->data, data, frame->len);
    
    // 更新写索引
    rx_write_idx = next_write;
}

/**
 * @brief 主循环调用，处理接收队列
 * @note 非ISR环境，可安全处理复杂逻辑
 */
void FDCAN_ProcessRxQueue(void)
{
    while (rx_read_idx != rx_write_idx) {
        // 取出帧
        can_frame_t frame;
        memcpy(&frame, (void*)&rx_queue[rx_read_idx], sizeof(can_frame_t));
        rx_read_idx = (rx_read_idx + 1) % FDCAN_RX_QUEUE_SIZE;
        
        // 解析ID
        uint8_t src = CAN_ID_GET_SRC(frame.id);
        uint8_t dst = CAN_ID_GET_DST(frame.id);
        uint8_t type = CAN_ID_GET_TYPE(frame.id);
        
        // 检查是否为本节点消息（广播或指定本节点）
        extern uint8_t g_local_node_id;
        if (dst != JOINT_ID_BROADCAST && dst != g_local_node_id) {
            continue;  // 不是发给自己的
        }
        
        // 根据消息类型处理
        switch (type) {
            case MSG_TYPE_STATUS: {
                if (frame.len >= sizeof(joint_status_t)) {
                    joint_status_t *status = (joint_status_t*)frame.data;
                    // 这里可以调用应用层回调处理其他关节的状态
                    // 例如：更新关节位置表
                }
                break;
            }
            
            case MSG_TYPE_CONTROL: {
                if (frame.len >= sizeof(joint_control_t)) {
                    joint_control_t *cmd = (joint_control_t*)frame.data;
                    // 应用层处理控制指令
                    // 例如：设置目标位置
                }
                break;
            }
            
            case MSG_TYPE_SYNC: {
                // 同步帧处理（多关节轨迹同步）
                // 可以触发执行缓存的轨迹点
                break;
            }
            
            case MSG_TYPE_ERROR: {
                // 错误处理
                break;
            }
        }
        
        // 调用用户回调（可选）
        FDCAN_RxCallback(&frame);
    }
}

/* ===================== 工具函数 ===================== */

uint8_t FDCAN_GetTxFreeLevel(void)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
}

uint32_t FDCAN_GetRxDropCount(void)
{
    return rx_drop_count;
}

/* ===================== 弱定义回调（用户可重写） ===================== */

__attribute__((weak)) void FDCAN_RxCallback(can_frame_t *frame)
{
    (void)frame;
    // 用户可在其他文件实现此函数处理自定义逻辑
}
