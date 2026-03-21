/**
 * @file armcanvaswidget.cpp
 * @brief 实现机械臂工作空间画布的坐标变换、绘制和点击取点逻辑。
 */

#include "ui/widgets/armcanvaswidget.h"

#include <QMouseEvent>
#include <QPainter>

#include "core/appconfig.h"

ArmCanvasWidget::ArmCanvasWidget(QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(700, 700);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void ArmCanvasWidget::setScene(const std::array<QPointF, 3> &points, const QPointF &target)
{
    m_points = points;
    m_target = target;
    update();
}

void ArmCanvasWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }

    const QPointF worldPoint = screenToWorld(event->position().x(), event->position().y());
    if (worldPoint.x() < AppConfig::kXMin || worldPoint.x() > AppConfig::kXMax
        || worldPoint.y() < AppConfig::kYMin || worldPoint.y() > AppConfig::kYMax) {
        return;
    }

    emit targetClicked(worldPoint.x(), worldPoint.y());
}

void ArmCanvasWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor("#f8f5f0"));

    const Transform tf = computeTransform();
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#fffdfb"));
    painter.drawRoundedRect(tf.area, 18.0, 18.0);

    painter.setPen(QPen(QColor("#e5d9cd"), 1.5));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(tf.area, 18.0, 18.0);

    painter.setPen(QPen(QColor("#efe6dd"), 1.0));
    constexpr int gridCount = 6;
    for (int i = 0; i <= gridCount; ++i) {
        const double t = static_cast<double>(i) / gridCount;
        const double xr = AppConfig::kXMin + (AppConfig::kXMax - AppConfig::kXMin) * t;
        const double yr = AppConfig::kYMin + (AppConfig::kYMax - AppConfig::kYMin) * t;
        painter.drawLine(worldToScreen(xr, AppConfig::kYMin), worldToScreen(xr, AppConfig::kYMax));
        painter.drawLine(worldToScreen(AppConfig::kXMin, yr), worldToScreen(AppConfig::kXMax, yr));
    }

    painter.setPen(QPen(QColor("#c96b2b"), 2.0));
    painter.drawLine(worldToScreen(0.0, AppConfig::kYMin), worldToScreen(0.0, AppConfig::kYMax));
    painter.drawLine(worldToScreen(AppConfig::kXMin, 0.0), worldToScreen(AppConfig::kXMax, 0.0));

    const QPointF center = worldToScreen(0.0, 0.0);
    const double outerRadius = (AppConfig::kL1 + AppConfig::kL2) * tf.scale;
    const double innerRadius = qAbs(AppConfig::kL1 - AppConfig::kL2) * tf.scale;
    painter.setPen(QPen(QColor("#e2a67c"), 1.4, Qt::DashLine));
    painter.drawEllipse(center, outerRadius, outerRadius);
    painter.drawEllipse(center, innerRadius, innerRadius);

    const QPointF base = worldToScreen(m_points[0].x(), m_points[0].y());
    const QPointF elbow = worldToScreen(m_points[1].x(), m_points[1].y());
    const QPointF tip = worldToScreen(m_points[2].x(), m_points[2].y());

    painter.setPen(QPen(QColor("#cf6a2c"), 10.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(base, elbow);
    painter.setPen(QPen(QColor("#e78a4d"), 10.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(elbow, tip);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#584a41"));
    painter.drawEllipse(base, 8.0, 8.0);
    painter.setBrush(QColor("#c96b2b"));
    painter.drawEllipse(elbow, 8.0, 8.0);
    painter.setBrush(QColor("#e78a4d"));
    painter.drawEllipse(tip, 10.0, 10.0);
    painter.setBrush(Qt::white);
    painter.drawEllipse(tip, 4.0, 4.0);

    const QPointF targetPoint = worldToScreen(m_target.x(), m_target.y());
    painter.setPen(QPen(QColor("#c96b2b"), 2.4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(targetPoint, 11.0, 11.0);
    painter.drawLine(QPointF(targetPoint.x() - 12.0, targetPoint.y()), QPointF(targetPoint.x() + 12.0, targetPoint.y()));
    painter.drawLine(QPointF(targetPoint.x(), targetPoint.y() - 12.0), QPointF(targetPoint.x(), targetPoint.y() + 12.0));
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#c96b2b"));
    painter.drawEllipse(targetPoint, 3.5, 3.5);
}

QRectF ArmCanvasWidget::plotRect() const
{
    constexpr double margin = 24.0;
    const double side = qBound(20.0, 640.0, qMin(width() - 2.0 * margin, height() - 2.0 * margin));
    return QRectF((width() - side) * 0.5, (height() - side) * 0.5, side, side);
}

ArmCanvasWidget::Transform ArmCanvasWidget::computeTransform() const
{
    const QRectF rect = plotRect();
    const double sx = rect.width() / (AppConfig::kXMax - AppConfig::kXMin);
    const double sy = rect.height() / (AppConfig::kYMax - AppConfig::kYMin);
    const double scale = qMin(sx, sy);
    const double drawWidth = (AppConfig::kXMax - AppConfig::kXMin) * scale;
    const double drawHeight = (AppConfig::kYMax - AppConfig::kYMin) * scale;
    const double left = rect.left() + (rect.width() - drawWidth) * 0.5;
    const double top = rect.top() + (rect.height() - drawHeight) * 0.5;

    Transform transform;
    transform.area = QRectF(left, top, drawWidth, drawHeight);
    transform.ox = left - AppConfig::kXMin * scale;
    transform.oy = top + AppConfig::kYMax * scale;
    transform.scale = scale;
    return transform;
}

QPointF ArmCanvasWidget::worldToScreen(double x, double y) const
{
    const Transform tf = computeTransform();
    return QPointF(tf.ox + x * tf.scale, tf.oy - y * tf.scale);
}

QPointF ArmCanvasWidget::screenToWorld(double px, double py) const
{
    const Transform tf = computeTransform();
    return QPointF((px - tf.ox) / tf.scale, (tf.oy - py) / tf.scale);
}
