/**
 * @file canprotocol.h
 * @brief 声明 CAN FD 协议相关的 ID 编码、定点数转换以及帧负载编解码接口。
 */

#ifndef PROTOCOL_CANPROTOCOL_H
#define PROTOCOL_CANPROTOCOL_H

#include <QByteArray>
#include <QString>
#include <QtGlobal>

#include "core/armtypes.h"

namespace CanProtocol {

struct ControlCommand
{
    double targetCurrentA = 0.0;
    double targetPositionDeg = 0.0;
    double targetVelocityDegS = 0.0;
    double targetAccelerationDegS2 = 0.0;
    double targetJerkDegS3 = 0.0;
    double stiffness = 0.0;
    double damping = 0.0;
    int controlMode = AppConfig::kDefaultControlMode;
};

/**
 * @brief 按项目协议生成 11 位标准 CAN ID
 * @param src 源节点 ID
 * @param dst 目标节点 ID
 * @param msgType 消息类型
 * @return 编码后的 CAN ID
 */
quint16 makeCanId(int src, int dst, int msgType);

/**
 * @brief 从 CAN ID 中提取源节点 ID
 * @param canId 11 位标准 CAN ID
 * @return 源节点 ID
 */
int canIdSource(quint16 canId);

/**
 * @brief 从 CAN ID 中提取目标节点 ID
 * @param canId 11 位标准 CAN ID
 * @return 目标节点 ID
 */
int canIdDestination(quint16 canId);

/**
 * @brief 从 CAN ID 中提取消息类型
 * @param canId 11 位标准 CAN ID
 * @return 消息类型
 */
int canIdType(quint16 canId);

/**
 * @brief 将浮点数编码为 32 位定点数
 * @param value 待编码的浮点值
 * @return 定点数结果
 */
qint32 floatToFix32(double value);

/**
 * @brief 将 32 位定点数解码为浮点数
 * @param value 定点数原始值
 * @return 解码后的浮点值
 */
double fix32ToFloat(qint32 value);

/**
 * @brief 将控制模式枚举值转换为可读字符串
 * @param mode 控制模式枚举
 * @return 控制模式名称
 */
QString controlModeName(int mode);

/**
 * @brief 将控制命令打包为 CONTROL 帧负载
 * @param command 控制命令结构体
 * @return 按小端序编码后的负载数据
 */
QByteArray encodeControlPayload(const ControlCommand &command);

/**
 * @brief 将 STATUS 帧负载解析为电机状态
 * @param payload 原始帧负载
 * @param status 输出状态结构体指针
 * @return 解析成功返回 true，否则返回 false
 */
bool decodeStatusPayload(const QByteArray &payload, MotorStatus *status);

} // namespace CanProtocol

#endif // PROTOCOL_CANPROTOCOL_H
