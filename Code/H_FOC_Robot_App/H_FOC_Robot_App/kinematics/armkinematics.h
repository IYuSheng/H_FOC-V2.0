/**
 * @file armkinematics.h
 * @brief 声明机械臂正逆运动学、雅可比、PD 控制和重力补偿相关接口。
 */

#ifndef KINEMATICS_ARMKINEMATICS_H
#define KINEMATICS_ARMKINEMATICS_H

#include <QPair>
#include <QPointF>

#include <array>
#include <optional>

#include "core/armtypes.h"

namespace ArmKinematics {

struct Jacobian
{
    double j11 = 0.0;
    double j12 = 0.0;
    double j21 = 0.0;
    double j22 = 0.0;
};

struct ForwardResult
{
    CartesianState cart;
    Jacobian jacobian;
    std::array<QPointF, 3> points;
};

/**
 * @brief 对输入值做区间限幅
 * @param value 待限幅的数值
 * @param minimum 下限
 * @param maximum 上限
 * @return 限幅后的结果
 */
double clamp(double value, double minimum, double maximum);

/**
 * @brief 根据当前双关节角度计算末端位姿、速度、雅可比和绘图点位
 * @param joint 当前双关节状态，角度单位为度，角速度单位为度每秒
 * @return 正运动学计算结果
 */
ForwardResult forward(const JointState &joint);

/**
 * @brief 在笛卡尔空间执行 PD 控制，并通过雅可比映射到关节电流
 * @param current 当前末端状态
 * @param target 目标末端状态
 * @param jacobian 当前姿态下的雅可比矩阵
 * @return 控制输出结果，包含 Fx、Fy、Iq1、Iq2
 */
ControlState pdControl(const CartesianState &current, const CartesianState &target, const Jacobian &jacobian);

/**
 * @brief 根据当前姿态计算两个关节的重力补偿前馈电流
 * @param joint 当前双关节状态
 * @return 返回 J1 和 J2 的重力补偿电流
 */
QPair<double, double> gravityCompensationCurrents(const JointState &joint);

/**
 * @brief 根据末端目标位置求解最接近当前姿态的逆运动学分支
 * @param target 目标末端位置，单位为米
 * @param referenceJoint 当前参考关节角，用于选择最平滑的逆解分支
 * @return 若目标可达则返回目标关节角，否则返回空
 */
std::optional<JointState> inverse(const QPointF &target, const JointState &referenceJoint);

/**
 * @brief 判断目标点在当前约束条件下是否可达
 * @param target 目标末端位置，单位为米
 * @param referenceJoint 当前参考关节角
 * @return 可达返回 true，否则返回 false
 */
bool isTargetReachable(const QPointF &target, const JointState &referenceJoint);

} // namespace ArmKinematics

#endif // KINEMATICS_ARMKINEMATICS_H
