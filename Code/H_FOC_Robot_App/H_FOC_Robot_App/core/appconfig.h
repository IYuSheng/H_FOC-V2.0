/**
 * @file appconfig.h
 * @brief 集中定义上位机项目所使用的默认参数、控制常量和机械臂配置。
 */

#ifndef CORE_APPCONFIG_H
#define CORE_APPCONFIG_H

#include <QtGlobal>

namespace AppConfig {

inline constexpr char kDefaultChannel[] = "COM21";
inline constexpr qint32 kDefaultTtyBaudrate = 4000000;
inline constexpr int kCanNominalBitrate = 1000000;
inline constexpr int kCanDataBitrate = 2000000;
inline constexpr bool kEnableControlTx = true;

inline constexpr int kMasterId = 0;
inline constexpr int kMotor1Id = 1;
inline constexpr int kMotor2Id = 2;
inline constexpr int kBroadcastId = 0x0F;

inline constexpr int kMsgTypeStatus = 0x0;
inline constexpr int kMsgTypeControl = 0x1;

inline constexpr int kControlModeNormal = 0;
inline constexpr int kControlModeTrapezoidal = 1;
inline constexpr int kControlModeSCurve = 2;
inline constexpr int kDefaultControlMode = kControlModeSCurve;

inline constexpr double kScale32 = 1e-5;

inline constexpr double kCmdStiffnessDefaultJ1 = 0.6;
inline constexpr double kCmdStiffnessDefaultJ2 = 0.1;
inline constexpr double kCmdStiffnessMin = 0.0;
inline constexpr double kCmdStiffnessMax = 1.0;

inline constexpr double kCmdDampingDefaultJ1 = 0.015;
inline constexpr double kCmdDampingDefaultJ2 = 0.001;
inline constexpr double kCmdDampingMin = 0.0;
inline constexpr double kCmdDampingMaxJ1 = 0.02;
inline constexpr double kCmdDampingMaxJ2 = 0.005;

inline constexpr double kProfileVelocityMinDegS = 5.0;
inline constexpr double kProfileVelocityMaxDegS = 300.0;
inline constexpr double kProfileVelocityDefaultDegS = 90.0;
inline constexpr double kProfileAccelMinDegS2 = 10.0;
inline constexpr double kProfileAccelMaxDegS2 = 20000.0;
inline constexpr double kProfileAccelDefaultDegS2 = 2000.0;
inline constexpr double kProfileJerkMinDegS3 = 10.0;
inline constexpr double kProfileJerkMaxDegS3 = 50000.0;
inline constexpr double kProfileJerkDefaultDegS3 = 5000.0;

inline constexpr double kGravityFfJ1 = 3.1;
inline constexpr double kGravityFfJ2 = 0.38;
inline constexpr double kGravityOffsetJ1Deg = -53.7;
inline constexpr double kGravityOffsetJ2Deg = 172.5;

inline constexpr int kLoopIntervalMs = 1;
inline constexpr int kUiUpdateMs = 50;
inline constexpr int kRxStaleMs = 50;
inline constexpr int kSerialOpenDelayMs = 1500;
inline constexpr int kCommandAckTimeoutMs = 800;
inline constexpr int kCommandWriteTimeoutMs = 250;

inline constexpr double kL1 = 0.07748;
inline constexpr double kL2 = 0.0625;
inline constexpr double kZero1 = -53.7;
inline constexpr double kZero2 = 172.5;
inline constexpr double kKpx = 10.0;
inline constexpr double kKdx = 1.0;
inline constexpr double kKpy = 10.0;
inline constexpr double kKdy = 1.0;
inline constexpr double kFMax = 20.0;
inline constexpr double kIqMax = 8.0;
inline constexpr double kKt = 0.095;

inline constexpr double kXMin = -0.15;
inline constexpr double kXMax = 0.15;
inline constexpr double kYMin = -0.18;
inline constexpr double kYMax = 0.05;

inline constexpr double kJ1MinDeg = -170.0;
inline constexpr double kJ1MaxDeg = 70.0;

} // namespace AppConfig

#endif // CORE_APPCONFIG_H
