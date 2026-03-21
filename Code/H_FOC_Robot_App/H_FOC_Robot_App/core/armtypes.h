/**
 * @file armtypes.h
 * @brief 定义机械臂控制、反馈、轨迹参数和界面状态共用的数据结构。
 */

#ifndef CORE_ARMTYPES_H
#define CORE_ARMTYPES_H

#include <QString>

#include "core/appconfig.h"

struct JointState
{
    double th1 = 0.0;
    double w1 = 0.0;
    double th2 = 0.0;
    double w2 = 0.0;
};

struct CartesianState
{
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
};

struct ControlState
{
    double fx = 0.0;
    double fy = 0.0;
    double iq1 = 0.0;
    double iq2 = 0.0;
    double grav1 = 0.0;
    double grav2 = 0.0;
};

struct MotorStatus
{
    double th = 0.0;
    double w = 0.0;
    double cur = 0.0;
    double temp = 0.0;
    double acc = 0.0;
    int status = 0;
    int error = 0;
    qint64 rxTimestampMs = 0;
};

struct ProfileSettings
{
    int controlMode = AppConfig::kDefaultControlMode;
    double velocity1 = AppConfig::kProfileVelocityDefaultDegS;
    double velocity2 = AppConfig::kProfileVelocityDefaultDegS;
    double acceleration1 = AppConfig::kProfileAccelDefaultDegS2;
    double acceleration2 = AppConfig::kProfileAccelDefaultDegS2;
    double jerk1 = AppConfig::kProfileJerkDefaultDegS3;
    double jerk2 = AppConfig::kProfileJerkDefaultDegS3;
    double stiffness1 = AppConfig::kCmdStiffnessDefaultJ1;
    double stiffness2 = AppConfig::kCmdStiffnessDefaultJ2;
    double damping1 = AppConfig::kCmdDampingDefaultJ1;
    double damping2 = AppConfig::kCmdDampingDefaultJ2;
};

struct ControllerStatus
{
    JointState joint = { AppConfig::kZero1, 0.0, AppConfig::kZero2, 0.0 };
    JointState cmdJoint = { AppConfig::kZero1, 0.0, AppConfig::kZero2, 0.0 };
    CartesianState cart = { 0.0, -(AppConfig::kL1 + AppConfig::kL2), 0.0, 0.0 };
    CartesianState target = { 0.0, -(AppConfig::kL1 + AppConfig::kL2), 0.0, 0.0 };
    ControlState ctrl;
    ProfileSettings profile;
    double hz = 0.0;
    int rxFrames = 0;
    int txFrames = 0;
    int errTxFrames = 0;
    QString mode = QStringLiteral("CAN FD 未连接");
    QString controlModeName = QStringLiteral("s_curve");
    bool canConnected = false;
    bool hasFeedback1 = false;
    bool hasFeedback2 = false;
    QString channel = QString::fromLatin1(AppConfig::kDefaultChannel);
    MotorStatus fb1;
    MotorStatus fb2;
};

#endif // CORE_ARMTYPES_H
