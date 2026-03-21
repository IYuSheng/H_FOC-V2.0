/**
 * @file slcancanfddevice.cpp
 * @brief 实现 slcan CAN FD 设备的打开、命令发送、文本帧解析和状态缓存。
 */

#include "canfd/slcancanfddevice.h"

#include <QThread>

SlcanCanFdDevice::SlcanCanFdDevice(QObject *parent)
    : QObject(parent)
{
    m_clock.start();
    connect(&m_port, &QSerialPort::readyRead, this, &SlcanCanFdDevice::onReadyRead);
}

bool SlcanCanFdDevice::open(const OpenOptions &options, QString *errorMessage)
{
    if (m_port.isOpen()) {
        close();
    }

    const QByteArray nominalCommand = bitrateCommand(options.nominalBitrate);
    if (nominalCommand.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("不支持的仲裁段波特率: %1").arg(options.nominalBitrate);
        }
        return false;
    }

    const QByteArray dataCommand = dataBitrateCommand(options.dataBitrate);
    if (options.dataBitrate != 0 && dataCommand.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("不支持的数据段波特率: %1").arg(options.dataBitrate);
        }
        return false;
    }

    m_options = options;
    m_pendingBuffer.clear();
    m_motorStates.clear();
    m_rxFrames = 0;
    m_txFrames = 0;
    m_errorFrames = 0;

    m_port.setPortName(options.channel);
    m_port.setBaudRate(options.ttyBaudrate);
    m_port.setDataBits(QSerialPort::Data8);
    m_port.setParity(QSerialPort::NoParity);
    m_port.setStopBits(QSerialPort::OneStop);
    m_port.setFlowControl(QSerialPort::NoFlowControl);

    if (!m_port.open(QIODevice::ReadWrite)) {
        if (errorMessage) {
            *errorMessage = m_port.errorString();
        }
        return false;
    }

    // 大多数 slcan 设备在串口刚打开后需要一小段稳定时间，否则首个命令容易丢。
    QThread::msleep(AppConfig::kSerialOpenDelayMs);
    m_port.clear(QSerialPort::AllDirections);

    QString localError;
    if (!writeCommandAndWait("C", &localError)
        || !writeCommandAndWait(nominalCommand, &localError)
        || (options.dataBitrate != 0 && !writeCommandAndWait(dataCommand, &localError))
        || !writeCommandAndWait(options.listenOnly ? "L" : "O", &localError)) {
        close();
        if (errorMessage) {
            *errorMessage = localError;
        }
        return false;
    }

    emit portStateChanged(true);
    return true;
}

void SlcanCanFdDevice::close()
{
    if (!m_port.isOpen()) {
        return;
    }

    m_port.write("C\r");
    m_port.waitForBytesWritten(AppConfig::kCommandWriteTimeoutMs);
    m_port.close();
    m_pendingBuffer.clear();
    emit portStateChanged(false);
}

bool SlcanCanFdDevice::isOpen() const
{
    return m_port.isOpen();
}

bool SlcanCanFdDevice::sendControlCommand(int destinationId, const CanProtocol::ControlCommand &command)
{
    if (!m_port.isOpen()) {
        ++m_errorFrames;
        return false;
    }

    const QByteArray payload = CanProtocol::encodeControlPayload(command);
    // 11 位标准帧 ID 仍然沿用原 Python 版本的 (src << 8) | (dst << 4) | type 映射。
    const quint16 canId = CanProtocol::makeCanId(
        AppConfig::kMasterId,
        destinationId,
        AppConfig::kMsgTypeControl);
    return sendFdFrame(canId, payload, false, true);
}

std::optional<MotorStatus> SlcanCanFdDevice::motorStatus(int motorId) const
{
    const auto it = m_motorStates.constFind(motorId);
    if (it == m_motorStates.constEnd()) {
        return std::nullopt;
    }
    return it.value();
}

std::optional<JointState> SlcanCanFdDevice::readJoint() const
{
    const auto status1 = motorStatus(AppConfig::kMotor1Id);
    const auto status2 = motorStatus(AppConfig::kMotor2Id);
    if (!status1 || !status2) {
        return std::nullopt;
    }

    const qint64 nowMs = m_clock.elapsed();
    if ((nowMs - status1->rxTimestampMs) > AppConfig::kRxStaleMs
        || (nowMs - status2->rxTimestampMs) > AppConfig::kRxStaleMs) {
        return std::nullopt;
    }

    return JointState { status1->th, status1->w, status2->th, status2->w };
}

int SlcanCanFdDevice::rxFrames() const
{
    return m_rxFrames;
}

int SlcanCanFdDevice::txFrames() const
{
    return m_txFrames;
}

int SlcanCanFdDevice::errorFrames() const
{
    return m_errorFrames;
}

QString SlcanCanFdDevice::channel() const
{
    return m_options.channel;
}

void SlcanCanFdDevice::onReadyRead()
{
    m_pendingBuffer.append(m_port.readAll());

    while (true) {
        int delimiterIndex = -1;
        char delimiter = '\0';

        for (int i = 0; i < m_pendingBuffer.size(); ++i) {
            const char current = m_pendingBuffer.at(i);
            if (current == '\r' || current == '\a') {
                delimiterIndex = i;
                delimiter = current;
                break;
            }
        }

        if (delimiterIndex < 0) {
            break;
        }

        const QByteArray line = m_pendingBuffer.left(delimiterIndex);
        m_pendingBuffer.remove(0, delimiterIndex + 1);

        if (delimiter == '\a') {
            ++m_errorFrames;
            continue;
        }

        processLine(line);
    }
}

bool SlcanCanFdDevice::writeCommandAndWait(const QByteArray &command, QString *errorMessage)
{
    if (!m_port.isOpen()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("串口未打开");
        }
        return false;
    }

    m_port.readAll();

    if (m_port.write(command + '\r') == -1) {
        if (errorMessage) {
            *errorMessage = m_port.errorString();
        }
        return false;
    }
    if (!m_port.waitForBytesWritten(AppConfig::kCommandWriteTimeoutMs)) {
        if (errorMessage) {
            *errorMessage = m_port.errorString();
        }
        return false;
    }

    QByteArray response;
    QElapsedTimer timer;
    timer.start();

    while (timer.elapsed() < AppConfig::kCommandAckTimeoutMs) {
        if (!m_port.waitForReadyRead(100)) {
            continue;
        }

        response.append(m_port.readAll());
        const int okIndex = response.indexOf('\r');
        const int errorIndex = response.indexOf('\a');

        if (errorIndex >= 0 && (okIndex < 0 || errorIndex < okIndex)) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("设备拒绝命令: %1").arg(QString::fromLatin1(command));
            }
            return false;
        }
        if (okIndex >= 0) {
            return true;
        }
    }

    if (errorMessage) {
        *errorMessage = QStringLiteral("等待设备应答超时: %1").arg(QString::fromLatin1(command));
    }
    return false;
}

bool SlcanCanFdDevice::sendFdFrame(quint32 arbitrationId, const QByteArray &payload, bool extendedId, bool bitrateSwitch)
{
    if (!m_port.isOpen() || payload.size() > 64) {
        ++m_errorFrames;
        return false;
    }

    const int dlc = dlcFromPayloadLength(payload.size());
    // slcan FD 命令中，b/B 表示 FD + BRS，d/D 表示 FD 不切换速率。
    const char prefix = extendedId ? (bitrateSwitch ? 'B' : 'D') : (bitrateSwitch ? 'b' : 'd');

    QByteArray command;
    command.reserve(16 + payload.size() * 2);
    command.append(prefix);
    command.append(QByteArray::number(arbitrationId, 16).toUpper().rightJustified(extendedId ? 8 : 3, '0'));
    command.append(QByteArray::number(dlc, 16).toUpper());
    command.append(payload.toHex().toUpper());
    command.append('\r');

    if (m_port.write(command) == -1) {
        ++m_errorFrames;
        return false;
    }

    m_port.flush();
    ++m_txFrames;
    return true;
}

void SlcanCanFdDevice::processLine(const QByteArray &line)
{
    if (line.isEmpty()) {
        return;
    }

    // 根据首字符区分标准帧/扩展帧/FD 帧，解析规则与 python-can 的 slcan backend 对齐。
    const char frameType = line.at(0);
    int idDigits = 0;
    int dlcOffset = 0;

    switch (frameType) {
    case 't':
    case 'r':
    case 'd':
    case 'b':
        idDigits = 3;
        dlcOffset = 1 + idDigits;
        break;
    case 'T':
    case 'R':
    case 'D':
    case 'B':
        idDigits = 8;
        dlcOffset = 1 + idDigits;
        break;
    default:
        return;
    }

    bool ok = false;
    const quint32 arbitrationId = QString::fromLatin1(line.mid(1, idDigits)).toUInt(&ok, 16);
    if (!ok || line.size() <= dlcOffset) {
        return;
    }

    const int dlc = QString::fromLatin1(line.mid(dlcOffset, 1)).toInt(&ok, 16);
    if (!ok) {
        return;
    }

    if (frameType == 'r' || frameType == 'R') {
        return;
    }

    const int headerSize = dlcOffset + 1;
    const int expectedBytes = (frameType == 'd' || frameType == 'D' || frameType == 'b' || frameType == 'B')
        ? dataLengthFromDlc(dlc)
        : dlc;
    // 某些适配器上可能收到不完整行，这里按“实际可解析长度”做一次保护。
    const int availableHexChars = qMax(0, line.size() - headerSize);
    const int actualBytes = qMin(expectedBytes, availableHexChars / 2);
    const QByteArray payload = QByteArray::fromHex(line.mid(headerSize, actualBytes * 2));

    if (CanProtocol::canIdType(arbitrationId) != AppConfig::kMsgTypeStatus) {
        return;
    }

    const int source = CanProtocol::canIdSource(arbitrationId);
    const int destination = CanProtocol::canIdDestination(arbitrationId);
    if ((source != AppConfig::kMotor1Id && source != AppConfig::kMotor2Id)
        || (destination != AppConfig::kMasterId && destination != AppConfig::kBroadcastId)) {
        return;
    }

    MotorStatus status;
    if (!CanProtocol::decodeStatusPayload(payload, &status)) {
        return;
    }

    // 上层只关心每个电机的最新一帧，所以这里直接覆盖缓存即可。
    status.rxTimestampMs = m_clock.elapsed();
    m_motorStates.insert(source, status);
    ++m_rxFrames;
}

int SlcanCanFdDevice::dataLengthFromDlc(int dlc)
{
    static const int table[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    return table[qBound(0, dlc, 15)];
}

int SlcanCanFdDevice::dlcFromPayloadLength(int length)
{
    if (length <= 8) {
        return qBound(0, length, 8);
    }
    if (length <= 12) {
        return 9;
    }
    if (length <= 16) {
        return 10;
    }
    if (length <= 20) {
        return 11;
    }
    if (length <= 24) {
        return 12;
    }
    if (length <= 32) {
        return 13;
    }
    if (length <= 48) {
        return 14;
    }
    return 15;
}

QByteArray SlcanCanFdDevice::bitrateCommand(int bitrate)
{
    switch (bitrate) {
    case 10000:
        return "S0";
    case 20000:
        return "S1";
    case 50000:
        return "S2";
    case 100000:
        return "S3";
    case 125000:
        return "S4";
    case 250000:
        return "S5";
    case 500000:
        return "S6";
    case 750000:
        return "S7";
    case 1000000:
        return "S8";
    case 83300:
        return "S9";
    default:
        return {};
    }
}

QByteArray SlcanCanFdDevice::dataBitrateCommand(int bitrate)
{
    switch (bitrate) {
    case 0:
        return {};
    case 2000000:
        return "Y2";
    case 5000000:
        return "Y5";
    default:
        return {};
    }
}
