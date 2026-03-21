/**
 * @file slcancanfddevice.h
 * @brief 声明基于串口 slcan 文本协议的 CAN FD 收发设备封装类。
 */

#ifndef CANFD_SLCANCANFDDEVICE_H
#define CANFD_SLCANCANFDDEVICE_H

#include <QObject>

#include <QElapsedTimer>
#include <QHash>
#include <QSerialPort>

#include <optional>

#include "core/appconfig.h"
#include "core/armtypes.h"
#include "protocol/canprotocol.h"

class SlcanCanFdDevice : public QObject
{
    Q_OBJECT

public:
    struct OpenOptions
    {
        QString channel = QString::fromLatin1(AppConfig::kDefaultChannel);
        qint32 ttyBaudrate = AppConfig::kDefaultTtyBaudrate;
        int nominalBitrate = AppConfig::kCanNominalBitrate;
        int dataBitrate = AppConfig::kCanDataBitrate;
        bool listenOnly = false;
    };

    /**
     * @brief 构造 slcan CAN FD 设备对象
     * @param parent Qt 父对象指针
     */
    explicit SlcanCanFdDevice(QObject *parent = nullptr);

    /**
     * @brief 打开串口并初始化 slcan CAN FD 通道
     * @param options 打开参数，包含串口号和波特率配置
     * @param errorMessage 打开失败时返回错误信息
     * @return 打开成功返回 true，否则返回 false
     */
    bool open(const OpenOptions &options, QString *errorMessage = nullptr);

    /**
     * @brief 关闭当前 slcan 设备
     */
    void close();

    /**
     * @brief 查询设备当前是否处于打开状态
     * @return 已打开返回 true，否则返回 false
     */
    bool isOpen() const;

    /**
     * @brief 向指定电机节点发送一帧控制命令
     * @param destinationId 目标电机节点 ID
     * @param command 控制命令内容
     * @return 发送成功返回 true，否则返回 false
     */
    bool sendControlCommand(int destinationId, const CanProtocol::ControlCommand &command);

    /**
     * @brief 获取指定电机最近一次缓存的状态帧
     * @param motorId 电机节点 ID
     * @return 若有缓存则返回状态，否则返回空
     */
    std::optional<MotorStatus> motorStatus(int motorId) const;

    /**
     * @brief 读取双关节的最新有效反馈
     * @return 若两个电机反馈都存在且未超时则返回关节状态，否则返回空
     */
    std::optional<JointState> readJoint() const;

    /**
     * @brief 获取累计接收帧数
     * @return 接收帧计数
     */
    int rxFrames() const;

    /**
     * @brief 获取累计发送帧数
     * @return 发送帧计数
     */
    int txFrames() const;

    /**
     * @brief 获取累计错误帧数
     * @return 错误计数
     */
    int errorFrames() const;

    /**
     * @brief 获取当前设备绑定的串口号
     * @return 串口名称
     */
    QString channel() const;

signals:
    void portStateChanged(bool open);

private slots:
    /**
     * @brief 串口有新数据到达时解析 slcan 文本帧
     */
    void onReadyRead();

private:
    /**
     * @brief 发送一条 slcan 管理命令并等待设备应答
     * @param command 待发送命令，不含行结束符
     * @param errorMessage 失败时返回错误信息
     * @return 成功返回 true，否则返回 false
     */
    bool writeCommandAndWait(const QByteArray &command, QString *errorMessage);

    /**
     * @brief 发送一帧 slcan FD 数据帧
     * @param arbitrationId 仲裁 ID
     * @param payload 原始数据负载
     * @param extendedId 是否为扩展帧
     * @param bitrateSwitch 是否启用 BRS
     * @return 发送成功返回 true，否则返回 false
     */
    bool sendFdFrame(quint32 arbitrationId, const QByteArray &payload, bool extendedId, bool bitrateSwitch);

    /**
     * @brief 解析一整行 slcan 文本帧并更新内部状态缓存
     * @param line 单行原始数据，不含结束符
     */
    void processLine(const QByteArray &line);

    /**
     * @brief 根据 DLC 计算 FD 帧的实际数据长度
     * @param dlc CAN FD DLC 编码值
     * @return 对应的数据字节数
     */
    static int dataLengthFromDlc(int dlc);

    /**
     * @brief 根据数据长度反推适配的 DLC 编码
     * @param length 原始数据长度
     * @return DLC 编码值
     */
    static int dlcFromPayloadLength(int length);

    /**
     * @brief 将仲裁段波特率转换为 slcan 命令
     * @param bitrate 仲裁段波特率
     * @return 对应的 slcan 命令字符串
     */
    static QByteArray bitrateCommand(int bitrate);

    /**
     * @brief 将数据段波特率转换为 slcan FD 命令
     * @param bitrate 数据段波特率
     * @return 对应的 slcan 命令字符串
     */
    static QByteArray dataBitrateCommand(int bitrate);

    QSerialPort m_port;
    QByteArray m_pendingBuffer;
    QHash<int, MotorStatus> m_motorStates;
    OpenOptions m_options;
    QElapsedTimer m_clock;
    int m_rxFrames = 0;
    int m_txFrames = 0;
    int m_errorFrames = 0;
};

#endif // CANFD_SLCANCANFDDEVICE_H
