/**
 * @file armcanvaswidget.h
 * @brief 声明机械臂二维工作空间绘图控件，负责显示连杆和目标点。
 */

#ifndef UI_WIDGETS_ARMCANVASWIDGET_H
#define UI_WIDGETS_ARMCANVASWIDGET_H

#include <QWidget>

#include <QPointF>

#include <array>

class ArmCanvasWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ArmCanvasWidget(QWidget *parent = nullptr);

    void setScene(const std::array<QPointF, 3> &points, const QPointF &target);

signals:
    void targetClicked(double x, double y);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

private:
    struct Transform
    {
        QRectF area;
        double ox = 0.0;
        double oy = 0.0;
        double scale = 1.0;
    };

    QRectF plotRect() const;
    Transform computeTransform() const;
    QPointF worldToScreen(double x, double y) const;
    QPointF screenToWorld(double px, double py) const;

    std::array<QPointF, 3> m_points {
        QPointF(0.0, 0.0),
        QPointF(0.0, -0.05),
        QPointF(-0.05, -0.12)
    };
    QPointF m_target = QPointF(-0.05, -0.12);
};

#endif // UI_WIDGETS_ARMCANVASWIDGET_H
