/**
 * @file canprotocol.cpp
 * @brief 实现 CAN FD 协议字段打包、状态帧解析和控制模式辅助函数。
 */

#include "protocol/canprotocol.h"

#include <QDataStream>
#include <QIODevice>
#include <QtEndian>

#include <algorithm>
#include <limits>

namespace CanProtocol {

quint16 makeCanId(int src, int dst, int msgType)
{
    return static_cast<quint16>(((src & 0x0F) << 8) | ((dst & 0x0F) << 4) | (msgType & 0x0F));
}

int canIdSource(quint16 canId)
{
    return (canId >> 8) & 0x0F;
}

int canIdDestination(quint16 canId)
{
    return (canId >> 4) & 0x0F;
}

int canIdType(quint16 canId)
{
    return canId & 0x0F;
}

qint32 floatToFix32(double value)
{
    const double scaled = value / AppConfig::kScale32;
    const double bounded = std::clamp(
        scaled,
        static_cast<double>(std::numeric_limits<qint32>::min()),
        static_cast<double>(std::numeric_limits<qint32>::max()));
    return static_cast<qint32>(bounded);
}

double fix32ToFloat(qint32 value)
{
    return static_cast<double>(value) * AppConfig::kScale32;
}

QString controlModeName(int mode)
{
    switch (mode) {
    case AppConfig::kControlModeNormal:
        return QStringLiteral("normal");
    case AppConfig::kControlModeTrapezoidal:
        return QStringLiteral("trapezoidal");
    case AppConfig::kControlModeSCurve:
        return QStringLiteral("s_curve");
    default:
        return QStringLiteral("unknown(%1)").arg(mode);
    }
}

QByteArray encodeControlPayload(const ControlCommand &command)
{
    QByteArray payload;
    payload.reserve(29);

    // 字段顺序必须和下位机 CONTROL 帧定义完全一致：7 个 int32 + 1 个 uint8。
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << floatToFix32(command.targetCurrentA);
    stream << floatToFix32(command.targetPositionDeg);
    stream << floatToFix32(command.targetVelocityDegS);
    stream << floatToFix32(command.targetAccelerationDegS2);
    stream << floatToFix32(command.targetJerkDegS3);
    stream << floatToFix32(command.stiffness);
    stream << floatToFix32(command.damping);
    stream << static_cast<quint8>(command.controlMode & 0xFF);
    return payload;
}

bool decodeStatusPayload(const QByteArray &payload, MotorStatus *status)
{
    if (status == nullptr || payload.size() < 20) {
        return false;
    }

    // STATUS 帧固定为 5 个 little-endian int32，分别对应位置/速度/电流/温度/加速度。
    const auto readInt32 = [&](int offset) -> qint32 {
        return qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(payload.constData() + offset));
    };

    status->th = fix32ToFloat(readInt32(0));
    status->w = fix32ToFloat(readInt32(4));
    status->cur = fix32ToFloat(readInt32(8));
    status->temp = fix32ToFloat(readInt32(12));
    status->acc = fix32ToFloat(readInt32(16));
    status->status = 0;
    status->error = 0;
    return true;
}

} // namespace CanProtocol
