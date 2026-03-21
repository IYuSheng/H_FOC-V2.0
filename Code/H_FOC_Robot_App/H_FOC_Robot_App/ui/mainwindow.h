/**
 * @file mainwindow.h
 * @brief 声明主窗口类，负责组织界面、同步控制参数并展示机械臂实时状态。
 */

#ifndef APP_UI_MAINWINDOW_H
#define APP_UI_MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class ArmCanvasWidget;
class ArmController;
class QCloseEvent;
class QDoubleSpinBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void buildDynamicUi();
    void configureInputs();
    void connectSignals();
    void applyStyles();
    void applyProfileUiState(int mode);
    void refreshUi();
    void syncSpinBoxValue(QDoubleSpinBox *spinBox, double value);
    void handleCanvasClick(double x, double y);

    Ui::MainWindow *ui = nullptr;
    ArmController *m_controller = nullptr;
    ArmCanvasWidget *m_canvas = nullptr;
    bool m_syncingUi = false;
};

#endif // APP_UI_MAINWINDOW_H
