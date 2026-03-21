/**
 * @file armcontroller.h
 * @brief 声明机械臂上位机控制器，负责自动连接 CAN FD、执行控制循环并对界面输出状态快照。
 */

#ifndef CONTROLLER_ARMCONTROLLER_H
#define CONTROLLER_ARMCONTROLLER_H

#include <QObject>

#include <QElapsedTimer>
#include <QTimer>

#include "canfd/slcancanfddevice.h"
#include "core/armtypes.h"

class ArmController : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 构造机械臂控制器对象
     * @param parent Qt 父对象指针
     */
    explicit ArmController(QObject *parent = nullptr);

    /**
     * @brief 启动控制循环并尝试自动连接默认 CAN FD 通道
     */
    void start();

    /**
     * @brief 停止控制循环并关闭当前 CAN FD 设备
     */
    void stop();

    /**
     * @brief 获取当前控制器状态快照
     * @return 供界面直接显示的状态结构体
     */
    ControllerStatus statusSnapshot() const;

    /**
     * @brief 设置末端目标位置
     * @param x 目标 X 坐标，单位为米
     * @param y 目标 Y 坐标，单位为米
     */
    void setTarget(double x, double y);

    /**
     * @brief 设置下发给驱动器的控制模式
     * @param mode 控制模式枚举值
     */
    void setControlMode(int mode);

    /**
     * @brief 设置指定关节的轨迹速度参数
     * @param jointIndex 关节编号，1 表示 J1，2 表示 J2
     * @param value 速度值，单位为度每秒
     */
    void setProfileVelocity(int jointIndex, double value);

    /**
     * @brief 设置指定关节的轨迹加速度参数
     * @param jointIndex 关节编号，1 表示 J1，2 表示 J2
     * @param value 加速度值，单位为度每二次方秒
     */
    void setProfileAcceleration(int jointIndex, double value);

    /**
     * @brief 设置指定关节的轨迹加加速度参数
     * @param jointIndex 关节编号，1 表示 J1，2 表示 J2
     * @param value 加加速度值，单位为度每三次方秒
     */
    void setProfileJerk(int jointIndex, double value);

    /**
     * @brief 设置指定关节的刚度参数
     * @param jointIndex 关节编号，1 表示 J1，2 表示 J2
     * @param value 刚度值
     */
    void setStiffness(int jointIndex, double value);

    /**
     * @brief 设置指定关节的阻尼参数
     * @param jointIndex 关节编号，1 表示 J1，2 表示 J2
     * @param value 阻尼值
     */
    void setDamping(int jointIndex, double value);

signals:
    /**
     * @brief 向界面发送状态提示消息
     * @param message 提示内容
     */
    void statusMessage(const QString &message);

private slots:
    /**
     * @brief 控制循环主函数，每 1 ms 执行一次
     */
    void runControlCycle();

private:
    struct EffectiveProfile
    {
        int controlMode = AppConfig::kDefaultControlMode;
        double velocity1 = 0.0;
        double velocity2 = 0.0;
        double acceleration1 = 0.0;
        double acceleration2 = 0.0;
        double jerk1 = 0.0;
        double jerk2 = 0.0;
        double stiffness1 = AppConfig::kCmdStiffnessDefaultJ1;
        double stiffness2 = AppConfig::kCmdStiffnessDefaultJ2;
        double damping1 = AppConfig::kCmdDampingDefaultJ1;
        double damping2 = AppConfig::kCmdDampingDefaultJ2;
    };

    /**
     * @brief 生成当前实际生效的轨迹参数
     * @return 已根据控制模式处理后的轨迹配置
     */
    EffectiveProfile effectiveProfile() const;

    /**
     * @brief 按默认配置自动连接 COM21 上的 slcan CAN FD 设备
     * @param errorMessage 连接失败时返回错误信息
     * @return 连接成功返回 true，否则返回 false
     */
    bool tryAutoConnect(QString *errorMessage = nullptr);

    /**
     * @brief 将关节显示姿态复位到最低点
     */
    void resetPoseToRest();

    /**
     * @brief 向两个驱动器发送零电流保持命令
     * @param profile 当前生效的轨迹参数
     */
    void sendZeroCurrentHold(const EffectiveProfile &profile);

    /**
     * @brief 将底层 CAN 设备缓存同步到外部状态快照
     */
    void syncFeedbackSnapshot();

    QTimer m_controlTimer;
    QElapsedTimer m_clock;
    qint64 m_lastTickNs = 0;
    bool m_running = false;
    bool m_hasFeedback = false;
    bool m_warnedFeedbackStale = false;
    const bool m_enableControlTx = AppConfig::kEnableControlTx;

    SlcanCanFdDevice m_canDevice;

    JointState m_joint = { AppConfig::kZero1, 0.0, AppConfig::kZero2, 0.0 };
    JointState m_cmdJoint = { AppConfig::kZero1, 0.0, AppConfig::kZero2, 0.0 };
    CartesianState m_cart = { 0.0, -(AppConfig::kL1 + AppConfig::kL2), 0.0, 0.0 };
    ControlState m_ctrl;
    CartesianState m_target = { 0.0, -(AppConfig::kL1 + AppConfig::kL2), 0.0, 0.0 };
    ProfileSettings m_profile;

    double m_hz = 0.0;
    qint64 m_lastFeedbackMs = 0;
    int m_rxFrames = 0;
    int m_txFrames = 0;
    int m_errTxFrames = 0;
    bool m_hasFb1 = false;
    bool m_hasFb2 = false;
    MotorStatus m_fb1;
    MotorStatus m_fb2;
    QString m_mode = QStringLiteral("CAN FD 未连接");
};

#endif // CONTROLLER_ARMCONTROLLER_H
