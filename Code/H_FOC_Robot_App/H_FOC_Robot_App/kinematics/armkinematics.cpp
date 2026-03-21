/**
 * @file armkinematics.cpp
 * @brief 实现双关节机械臂的运动学解算、控制映射和可达性判断。
 */

#include "kinematics/armkinematics.h"

#include <QtMath>

#include <algorithm>

namespace {

double wrapDegError(double errorDeg)
{
    while (errorDeg > 180.0) {
        errorDeg -= 360.0;
    }
    while (errorDeg < -180.0) {
        errorDeg += 360.0;
    }
    return errorDeg;
}

double unwrapToReference(double angleDeg, double referenceDeg)
{
    return referenceDeg + wrapDegError(angleDeg - referenceDeg);
}

} // namespace

namespace ArmKinematics {

double clamp(double value, double minimum, double maximum)
{
    return std::clamp(value, minimum, maximum);
}

ForwardResult forward(const JointState &joint)
{
    // 先把机械角转换到 2R 平面模型所使用的相对关节角坐标系。
    const double t1 = qDegreesToRadians(joint.th1 - AppConfig::kZero1);
    const double t2 = qDegreesToRadians(joint.th2 - AppConfig::kZero2);
    const double t12 = t1 + t2;

    const double s1 = qSin(t1);
    const double c1 = qCos(t1);
    const double s12 = qSin(t12);
    const double c12 = qCos(t12);

    const double x = -(AppConfig::kL1 * s1 + AppConfig::kL2 * s12);
    const double y = -(AppConfig::kL1 * c1 + AppConfig::kL2 * c12);

    Jacobian jacobian;
    jacobian.j11 = -(AppConfig::kL1 * c1 + AppConfig::kL2 * c12);
    jacobian.j12 = -(AppConfig::kL2 * c12);
    jacobian.j21 = AppConfig::kL1 * s1 + AppConfig::kL2 * s12;
    jacobian.j22 = AppConfig::kL2 * s12;

    const double d1 = qDegreesToRadians(joint.w1);
    const double d2 = qDegreesToRadians(joint.w2);

    CartesianState cart;
    cart.x = x;
    cart.y = y;
    cart.vx = jacobian.j11 * d1 + jacobian.j12 * d2;
    cart.vy = jacobian.j21 * d1 + jacobian.j22 * d2;

    ForwardResult result;
    result.cart = cart;
    result.jacobian = jacobian;
    result.points = {
        // 三个点分别是基座、肘部、末端，供 UI 直接绘制连杆。
        QPointF(0.0, 0.0),
        QPointF(-AppConfig::kL1 * s1, -AppConfig::kL1 * c1),
        QPointF(x, y)
    };
    return result;
}

ControlState pdControl(const CartesianState &current, const CartesianState &target, const Jacobian &jacobian)
{
    ControlState control;
    // 先在笛卡尔空间做 PD，再通过雅可比转成两个关节的力矩/电流。
    control.fx = clamp(
        AppConfig::kKpx * (target.x - current.x) + AppConfig::kKdx * (target.vx - current.vx),
        -AppConfig::kFMax,
        AppConfig::kFMax);
    control.fy = clamp(
        AppConfig::kKpy * (target.y - current.y) + AppConfig::kKdy * (target.vy - current.vy),
        -AppConfig::kFMax,
        AppConfig::kFMax);

    const double tau1 = jacobian.j11 * control.fx + jacobian.j21 * control.fy;
    const double tau2 = jacobian.j12 * control.fx + jacobian.j22 * control.fy;

    control.iq1 = clamp(tau1 / AppConfig::kKt, -AppConfig::kIqMax, AppConfig::kIqMax);
    control.iq2 = clamp(tau2 / AppConfig::kKt, -AppConfig::kIqMax, AppConfig::kIqMax);
    return control;
}

QPair<double, double> gravityCompensationCurrents(const JointState &joint)
{
    const double theta1 = qDegreesToRadians(joint.th1 - AppConfig::kGravityOffsetJ1Deg);
    const double theta2 = qDegreesToRadians(joint.th2 - AppConfig::kGravityOffsetJ2Deg);
    const double grav1 = AppConfig::kGravityFfJ1 * qSin(theta1);
    const double grav2 = AppConfig::kGravityFfJ2 * qSin(theta1 + theta2);
    return qMakePair(grav1, grav2);
}

std::optional<JointState> inverse(const QPointF &target, const JointState &referenceJoint)
{
    // 本项目的工作空间坐标与标准 2R 平面定义不同，这里先映射到标准形式再求逆解。
    const double xStd = -target.y();
    const double yStd = -target.x();
    const double r2 = xStd * xStd + yStd * yStd;
    const double r = qSqrt(r2);

    if (r > (AppConfig::kL1 + AppConfig::kL2) || r < qAbs(AppConfig::kL1 - AppConfig::kL2)) {
        return std::nullopt;
    }

    double c2 = (r2 - AppConfig::kL1 * AppConfig::kL1 - AppConfig::kL2 * AppConfig::kL2)
        / (2.0 * AppConfig::kL1 * AppConfig::kL2);
    c2 = clamp(c2, -1.0, 1.0);
    const double s2Abs = qSqrt(qMax(0.0, 1.0 - c2 * c2));

    std::optional<JointState> best;
    double bestScore = 1.0e18;

    for (const double s2 : { s2Abs, -s2Abs }) {
        // 两组 s2 对应肘上/肘下两种分支，这里都算出来再择优。
        const double t2 = qAtan2(s2, c2);
        const double t1 = qAtan2(yStd, xStd) - qAtan2(AppConfig::kL2 * s2, AppConfig::kL1 + AppConfig::kL2 * c2);

        double th1Deg = qRadiansToDegrees(t1) + AppConfig::kZero1;
        double th2Deg = qRadiansToDegrees(t2) + AppConfig::kZero2;

        th1Deg = unwrapToReference(th1Deg, referenceJoint.th1);
        th2Deg = unwrapToReference(th2Deg, referenceJoint.th2);

        // J1 先做硬限位过滤，避免把不可执行的目标继续往下传。
        if (th1Deg < AppConfig::kJ1MinDeg || th1Deg > AppConfig::kJ1MaxDeg) {
            continue;
        }

        // 选和当前姿态最接近的解，减少突然翻肘或跨 ±180 度跳变。
        const double score = qAbs(wrapDegError(th1Deg - referenceJoint.th1))
            + qAbs(wrapDegError(th2Deg - referenceJoint.th2));

        if (!best || score < bestScore) {
            best = JointState { th1Deg, 0.0, th2Deg, 0.0 };
            bestScore = score;
        }
    }

    return best;
}

bool isTargetReachable(const QPointF &target, const JointState &referenceJoint)
{
    return inverse(target, referenceJoint).has_value();
}

} // namespace ArmKinematics
