/**
 * @file armcontroller.cpp
 * @brief 实现机械臂控制主流程，包括自动连接 CAN FD、反馈处理、逆解和控制指令下发。
 */

#include "controller/armcontroller.h"

#include "kinematics/armkinematics.h"
#include "protocol/canprotocol.h"

#include <QPointF>

ArmController::ArmController(QObject *parent)
    : QObject(parent)
{
    m_clock.start();
    m_controlTimer.setInterval(AppConfig::kLoopIntervalMs);
    m_controlTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_controlTimer, &QTimer::timeout, this, &ArmController::runControlCycle);
    connect(&m_canDevice, &SlcanCanFdDevice::portStateChanged, this, [this](bool open) {
        if (!open) {
            m_hasFeedback = false;
            m_hasFb1 = false;
            m_hasFb2 = false;
            resetPoseToRest();
            m_mode = QStringLiteral("CAN FD 未连接");
        }
    });
}

void ArmController::start()
{
    if (m_running) {
        return;
    }

    m_running = true;
    m_lastTickNs = 0;
    m_warnedFeedbackStale = false;
    resetPoseToRest();

    QString errorMessage;
    if (tryAutoConnect(&errorMessage)) {
        emit statusMessage(QStringLiteral("已自动连接 %1").arg(QString::fromLatin1(AppConfig::kDefaultChannel)));
    } else if (!errorMessage.isEmpty()) {
        emit statusMessage(QStringLiteral("自动连接失败：%1").arg(errorMessage));
    }

    m_controlTimer.start();
}

void ArmController::stop()
{
    if (!m_running) {
        return;
    }

    m_controlTimer.stop();

    if (m_canDevice.isOpen() && m_enableControlTx) {
        sendZeroCurrentHold(effectiveProfile());
    }

    m_canDevice.close();
    resetPoseToRest();
    m_running = false;
}

ControllerStatus ArmController::statusSnapshot() const
{
    ControllerStatus status;
    status.joint = m_joint;
    status.cmdJoint = m_cmdJoint;
    status.cart = m_cart;
    status.target = m_target;
    status.ctrl = m_ctrl;
    status.profile = m_profile;
    status.hz = m_hz;
    status.rxFrames = m_rxFrames;
    status.txFrames = m_txFrames;
    status.errTxFrames = m_errTxFrames;
    status.mode = m_mode;
    status.controlModeName = CanProtocol::controlModeName(m_profile.controlMode);
    status.canConnected = m_canDevice.isOpen();
    status.hasFeedback1 = m_hasFb1;
    status.hasFeedback2 = m_hasFb2;
    status.channel = QString::fromLatin1(AppConfig::kDefaultChannel);
    status.fb1 = m_fb1;
    status.fb2 = m_fb2;
    return status;
}

void ArmController::setTarget(double x, double y)
{
    m_target.x = x;
    m_target.y = y;
}

void ArmController::setControlMode(int mode)
{
    if (mode == AppConfig::kControlModeNormal
        || mode == AppConfig::kControlModeTrapezoidal
        || mode == AppConfig::kControlModeSCurve) {
        m_profile.controlMode = mode;
    }
}

void ArmController::setProfileVelocity(int jointIndex, double value)
{
    const double bounded = ArmKinematics::clamp(
        value,
        AppConfig::kProfileVelocityMinDegS,
        AppConfig::kProfileVelocityMaxDegS);

    if (jointIndex == 1) {
        m_profile.velocity1 = bounded;
    } else if (jointIndex == 2) {
        m_profile.velocity2 = bounded;
    }
}

void ArmController::setProfileAcceleration(int jointIndex, double value)
{
    const double bounded = ArmKinematics::clamp(
        value,
        AppConfig::kProfileAccelMinDegS2,
        AppConfig::kProfileAccelMaxDegS2);

    if (jointIndex == 1) {
        m_profile.acceleration1 = bounded;
    } else if (jointIndex == 2) {
        m_profile.acceleration2 = bounded;
    }
}

void ArmController::setProfileJerk(int jointIndex, double value)
{
    const double bounded = ArmKinematics::clamp(
        value,
        AppConfig::kProfileJerkMinDegS3,
        AppConfig::kProfileJerkMaxDegS3);

    if (jointIndex == 1) {
        m_profile.jerk1 = bounded;
    } else if (jointIndex == 2) {
        m_profile.jerk2 = bounded;
    }
}

void ArmController::setStiffness(int jointIndex, double value)
{
    const double bounded = ArmKinematics::clamp(
        value,
        AppConfig::kCmdStiffnessMin,
        AppConfig::kCmdStiffnessMax);

    if (jointIndex == 1) {
        m_profile.stiffness1 = bounded;
    } else if (jointIndex == 2) {
        m_profile.stiffness2 = bounded;
    }
}

void ArmController::setDamping(int jointIndex, double value)
{
    if (jointIndex == 1) {
        m_profile.damping1 = ArmKinematics::clamp(
            value,
            AppConfig::kCmdDampingMin,
            AppConfig::kCmdDampingMaxJ1);
    } else if (jointIndex == 2) {
        m_profile.damping2 = ArmKinematics::clamp(
            value,
            AppConfig::kCmdDampingMin,
            AppConfig::kCmdDampingMaxJ2);
    }
}

void ArmController::runControlCycle()
{
    const qint64 nowNs = m_clock.nsecsElapsed();
    if (m_lastTickNs != 0) {
        const double dtSec = static_cast<double>(nowNs - m_lastTickNs) / 1.0e9;
        if (dtSec > 0.0) {
            m_hz = (m_hz <= 0.0) ? (1.0 / dtSec) : (0.9 * m_hz + 0.1 * (1.0 / dtSec));
        }
    }
    m_lastTickNs = nowNs;

    if (!m_canDevice.isOpen()) {
        resetPoseToRest();
        m_mode = QStringLiteral("CAN FD 未连接");
        syncFeedbackSnapshot();
        return;
    }

    const qint64 nowMs = m_clock.elapsed();
    if (const auto feedbackJoint = m_canDevice.readJoint()) {
        m_joint = *feedbackJoint;
        m_hasFeedback = true;
        m_lastFeedbackMs = nowMs;
    }

    if (!m_hasFeedback) {
        resetPoseToRest();
        m_mode = QStringLiteral("CAN FD 等待反馈");
        syncFeedbackSnapshot();
        return;
    }

    const auto forwardResult = ArmKinematics::forward(m_joint);
    m_cart = forwardResult.cart;
    m_ctrl = ArmKinematics::pdControl(m_cart, m_target, forwardResult.jacobian);

    const auto grav = ArmKinematics::gravityCompensationCurrents(m_joint);
    m_ctrl.grav1 = grav.first;
    m_ctrl.grav2 = grav.second;

    if (const auto ikJoint = ArmKinematics::inverse(QPointF(m_target.x, m_target.y), m_joint)) {
        m_cmdJoint = *ikJoint;
    } else {
        m_cmdJoint = JointState { m_joint.th1, 0.0, m_joint.th2, 0.0 };
    }

    const EffectiveProfile profile = effectiveProfile();
    if ((nowMs - m_lastFeedbackMs) > AppConfig::kRxStaleMs) {
        sendZeroCurrentHold(profile);
        m_hasFeedback = false;
        resetPoseToRest();
        m_mode = QStringLiteral("CAN FD 反馈超时");
        if (!m_warnedFeedbackStale) {
            emit statusMessage(QStringLiteral("CAN FD 反馈超时，已自动将输出回零"));
            m_warnedFeedbackStale = true;
        }
        syncFeedbackSnapshot();
        return;
    }

    if (m_enableControlTx) {
        const double totalIq1 = ArmKinematics::clamp(
            m_ctrl.iq1 + m_ctrl.grav1,
            -AppConfig::kIqMax,
            AppConfig::kIqMax);
        const double totalIq2 = ArmKinematics::clamp(
            m_ctrl.iq2 + m_ctrl.grav2,
            -AppConfig::kIqMax,
            AppConfig::kIqMax);

        CanProtocol::ControlCommand j1Command;
        j1Command.targetCurrentA = totalIq1;
        j1Command.targetPositionDeg = m_cmdJoint.th1;
        j1Command.targetVelocityDegS = profile.velocity1;
        j1Command.targetAccelerationDegS2 = profile.acceleration1;
        j1Command.targetJerkDegS3 = profile.jerk1;
        j1Command.stiffness = profile.stiffness1;
        j1Command.damping = profile.damping1;
        j1Command.controlMode = profile.controlMode;

        CanProtocol::ControlCommand j2Command = j1Command;
        j2Command.targetCurrentA = totalIq2;
        j2Command.targetPositionDeg = m_cmdJoint.th2;
        j2Command.targetVelocityDegS = profile.velocity2;
        j2Command.targetAccelerationDegS2 = profile.acceleration2;
        j2Command.targetJerkDegS3 = profile.jerk2;
        j2Command.stiffness = profile.stiffness2;
        j2Command.damping = profile.damping2;

        m_canDevice.sendControlCommand(AppConfig::kMotor1Id, j1Command);
        m_canDevice.sendControlCommand(AppConfig::kMotor2Id, j2Command);
    }

    m_warnedFeedbackStale = false;
    m_mode = QStringLiteral("CAN FD 在线");
    syncFeedbackSnapshot();
}

ArmController::EffectiveProfile ArmController::effectiveProfile() const
{
    EffectiveProfile profile;
    profile.controlMode = m_profile.controlMode;
    profile.stiffness1 = m_profile.stiffness1;
    profile.stiffness2 = m_profile.stiffness2;
    profile.damping1 = m_profile.damping1;
    profile.damping2 = m_profile.damping2;

    if (m_profile.controlMode == AppConfig::kControlModeNormal) {
        return profile;
    }

    profile.velocity1 = m_profile.velocity1;
    profile.velocity2 = m_profile.velocity2;
    profile.acceleration1 = m_profile.acceleration1;
    profile.acceleration2 = m_profile.acceleration2;
    profile.jerk1 = m_profile.jerk1;
    profile.jerk2 = m_profile.jerk2;
    return profile;
}

bool ArmController::tryAutoConnect(QString *errorMessage)
{
    if (m_canDevice.isOpen()) {
        return true;
    }

    SlcanCanFdDevice::OpenOptions options;
    options.channel = QString::fromLatin1(AppConfig::kDefaultChannel);
    options.ttyBaudrate = AppConfig::kDefaultTtyBaudrate;
    options.nominalBitrate = AppConfig::kCanNominalBitrate;
    options.dataBitrate = AppConfig::kCanDataBitrate;

    if (!m_canDevice.open(options, errorMessage)) {
        m_mode = QStringLiteral("CAN FD 未连接");
        return false;
    }

    m_mode = QStringLiteral("CAN FD 等待反馈");
    return true;
}

void ArmController::resetPoseToRest()
{
    m_joint = JointState { AppConfig::kZero1, 0.0, AppConfig::kZero2, 0.0 };
    m_cmdJoint = m_joint;
    m_cart = CartesianState { 0.0, -(AppConfig::kL1 + AppConfig::kL2), 0.0, 0.0 };
    m_ctrl = ControlState {};
}

void ArmController::sendZeroCurrentHold(const EffectiveProfile &profile)
{
    if (!m_canDevice.isOpen() || !m_enableControlTx) {
        return;
    }

    CanProtocol::ControlCommand j1Command;
    j1Command.targetCurrentA = 0.0;
    j1Command.targetPositionDeg = m_joint.th1;
    j1Command.targetVelocityDegS = profile.velocity1;
    j1Command.targetAccelerationDegS2 = profile.acceleration1;
    j1Command.targetJerkDegS3 = profile.jerk1;
    j1Command.stiffness = profile.stiffness1;
    j1Command.damping = profile.damping1;
    j1Command.controlMode = profile.controlMode;

    CanProtocol::ControlCommand j2Command = j1Command;
    j2Command.targetPositionDeg = m_joint.th2;
    j2Command.targetVelocityDegS = profile.velocity2;
    j2Command.targetAccelerationDegS2 = profile.acceleration2;
    j2Command.targetJerkDegS3 = profile.jerk2;
    j2Command.stiffness = profile.stiffness2;
    j2Command.damping = profile.damping2;

    m_canDevice.sendControlCommand(AppConfig::kMotor1Id, j1Command);
    m_canDevice.sendControlCommand(AppConfig::kMotor2Id, j2Command);
}

void ArmController::syncFeedbackSnapshot()
{
    m_rxFrames = m_canDevice.rxFrames();
    m_txFrames = m_canDevice.txFrames();
    m_errTxFrames = m_canDevice.errorFrames();

    if (const auto status = m_canDevice.motorStatus(AppConfig::kMotor1Id)) {
        m_fb1 = *status;
        m_hasFb1 = true;
    } else {
        m_hasFb1 = false;
    }

    if (const auto status = m_canDevice.motorStatus(AppConfig::kMotor2Id)) {
        m_fb2 = *status;
        m_hasFb2 = true;
    } else {
        m_hasFb2 = false;
    }
}
