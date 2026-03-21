/**
 * @file mainwindow.cpp
 * @brief 实现主界面的初始化、样式应用、参数同步和状态刷新逻辑。
 */

#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include "controller/armcontroller.h"
#include "kinematics/armkinematics.h"
#include "ui/widgets/armcanvaswidget.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QStyle>
#include <QTimer>

namespace {

void configureSpinBox(
    QDoubleSpinBox *spinBox,
    double minimum,
    double maximum,
    double value,
    double step,
    int decimals,
    const QString &suffix)
{
    spinBox->setRange(minimum, maximum);
    spinBox->setValue(value);
    spinBox->setSingleStep(step);
    spinBox->setDecimals(decimals);
    spinBox->setSuffix(suffix);
    spinBox->setKeyboardTracking(false);
    spinBox->setAlignment(Qt::AlignRight);
    spinBox->setAccelerated(true);
}

}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_controller(new ArmController(this))
{
    ui->setupUi(this);
    buildDynamicUi();
    configureInputs();
    applyStyles();
    connectSignals();

    m_controller->start();
    refreshUi();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (m_controller) {
        m_controller->stop();
    }
    QMainWindow::closeEvent(event);
}

void MainWindow::buildDynamicUi()
{
    setWindowTitle(QStringLiteral("H FOC App"));
    statusBar()->setSizeGripEnabled(false);

    m_canvas = new ArmCanvasWidget(this);
    ui->canvasLayout->addWidget(m_canvas, 0, Qt::AlignCenter);

    ui->controlModeComboBox->addItem(QStringLiteral("正常控制"), AppConfig::kControlModeNormal);
    ui->controlModeComboBox->addItem(QStringLiteral("梯形加减速"), AppConfig::kControlModeTrapezoidal);
    ui->controlModeComboBox->addItem(QStringLiteral("S 曲线"), AppConfig::kControlModeSCurve);

    ui->connectionStateButton->setFocusPolicy(Qt::NoFocus);
    ui->connectionStateButton->setCursor(Qt::ArrowCursor);
}

void MainWindow::configureInputs()
{
    configureSpinBox(
        ui->stiffnessJ1SpinBox,
        AppConfig::kCmdStiffnessMin,
        AppConfig::kCmdStiffnessMax,
        AppConfig::kCmdStiffnessDefaultJ1,
        0.01,
        2,
        QString());
    configureSpinBox(
        ui->dampingJ1SpinBox,
        AppConfig::kCmdDampingMin,
        AppConfig::kCmdDampingMaxJ1,
        AppConfig::kCmdDampingDefaultJ1,
        0.0005,
        4,
        QString());
    configureSpinBox(
        ui->stiffnessJ2SpinBox,
        AppConfig::kCmdStiffnessMin,
        AppConfig::kCmdStiffnessMax,
        AppConfig::kCmdStiffnessDefaultJ2,
        0.01,
        2,
        QString());
    configureSpinBox(
        ui->dampingJ2SpinBox,
        AppConfig::kCmdDampingMin,
        AppConfig::kCmdDampingMaxJ2,
        AppConfig::kCmdDampingDefaultJ2,
        0.0001,
        4,
        QString());

    configureSpinBox(
        ui->profileVelJ1SpinBox,
        AppConfig::kProfileVelocityMinDegS,
        AppConfig::kProfileVelocityMaxDegS,
        AppConfig::kProfileVelocityDefaultDegS,
        5.0,
        0,
        QStringLiteral(" °/s"));
    configureSpinBox(
        ui->profileAccJ1SpinBox,
        AppConfig::kProfileAccelMinDegS2,
        AppConfig::kProfileAccelMaxDegS2,
        AppConfig::kProfileAccelDefaultDegS2,
        100.0,
        0,
        QStringLiteral(" °/s²"));
    configureSpinBox(
        ui->profileJerkJ1SpinBox,
        AppConfig::kProfileJerkMinDegS3,
        AppConfig::kProfileJerkMaxDegS3,
        AppConfig::kProfileJerkDefaultDegS3,
        100.0,
        0,
        QStringLiteral(" °/s³"));
    configureSpinBox(
        ui->profileVelJ2SpinBox,
        AppConfig::kProfileVelocityMinDegS,
        AppConfig::kProfileVelocityMaxDegS,
        AppConfig::kProfileVelocityDefaultDegS,
        5.0,
        0,
        QStringLiteral(" °/s"));
    configureSpinBox(
        ui->profileAccJ2SpinBox,
        AppConfig::kProfileAccelMinDegS2,
        AppConfig::kProfileAccelMaxDegS2,
        AppConfig::kProfileAccelDefaultDegS2,
        100.0,
        0,
        QStringLiteral(" °/s²"));
    configureSpinBox(
        ui->profileJerkJ2SpinBox,
        AppConfig::kProfileJerkMinDegS3,
        AppConfig::kProfileJerkMaxDegS3,
        AppConfig::kProfileJerkDefaultDegS3,
        100.0,
        0,
        QStringLiteral(" °/s³"));
}

void MainWindow::connectSignals()
{
    connect(m_controller, &ArmController::statusMessage, this, [this](const QString &message) {
        statusBar()->showMessage(message, 3500);
    });

    connect(m_canvas, &ArmCanvasWidget::targetClicked, this, &MainWindow::handleCanvasClick);

    connect(ui->controlModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        if (m_syncingUi) {
            return;
        }
        const int mode = ui->controlModeComboBox->currentData().toInt();
        m_controller->setControlMode(mode);
        applyProfileUiState(mode);
    });

    connect(ui->stiffnessJ1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setStiffness(1, value);
        }
    });
    connect(ui->dampingJ1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setDamping(1, value);
        }
    });
    connect(ui->stiffnessJ2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setStiffness(2, value);
        }
    });
    connect(ui->dampingJ2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setDamping(2, value);
        }
    });

    connect(ui->profileVelJ1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setProfileVelocity(1, value);
        }
    });
    connect(ui->profileAccJ1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setProfileAcceleration(1, value);
        }
    });
    connect(ui->profileJerkJ1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setProfileJerk(1, value);
        }
    });
    connect(ui->profileVelJ2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setProfileVelocity(2, value);
        }
    });
    connect(ui->profileAccJ2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setProfileAcceleration(2, value);
        }
    });
    connect(ui->profileJerkJ2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (!m_syncingUi) {
            m_controller->setProfileJerk(2, value);
        }
    });

    auto *refreshTimer = new QTimer(this);
    refreshTimer->setInterval(AppConfig::kUiUpdateMs);
    connect(refreshTimer, &QTimer::timeout, this, &MainWindow::refreshUi);
    refreshTimer->start();
}

void MainWindow::applyStyles()
{
    setStyleSheet(QStringLiteral(
        "QMainWindow { background: #f4eee7; }"
        "QFrame#topBarFrame, QFrame#workspaceFrame, QFrame#controlsFrame, QFrame#canvasFrame, QFrame#statusFrame {"
        "  background: #fffdfb; border: 1px solid #e6d9cc; border-radius: 18px; }"
        "QLabel#titleLabel { font-size: 26px; font-weight: 700; color: #56483f; }"
        "QLabel#subtitleLabel, QLabel#workspaceHintLabel, QLabel#profileHintLabel { color: #8d7868; font-size: 12px; }"
        "QLabel#stateBadgeLabel {"
        "  background: #f7ecdf; color: #c66c31; border-radius: 10px; padding: 6px 12px; font-weight: 700; }"
        "QPushButton#connectionStateButton {"
        "  min-width: 148px; min-height: 38px; border: none; border-radius: 11px; font-weight: 700; }"
        "QPushButton#connectionStateButton[connected=\"true\"] { background: #e27f42; color: white; }"
        "QPushButton#connectionStateButton[connected=\"false\"] { background: #d6c8bc; color: #fffaf4; }"
        "QGroupBox {"
        "  background: #fffdfb; border: 1px solid #e6d9cc; border-radius: 16px; margin-top: 16px;"
        "  padding: 14px 14px 12px 14px; font-weight: 700; color: #56483f; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }"
        "QLabel#panelTitleLabel, QLabel#statusTitleLabel { font-size: 17px; font-weight: 700; color: #56483f; }"
        "QLabel#targetCaptionLabel, QLabel#cartCaptionLabel, QLabel#cmdCaptionLabel, QLabel#jointCaptionLabel,"
        "QLabel#runtimeCaptionLabel, QLabel#currentCaptionLabel, QLabel#trafficCaptionLabel {"
        "  color: #8d7868; font-size: 12px; font-weight: 600; }"
        "QLabel#targetValueLabel, QLabel#cartValueLabel, QLabel#cmdValueLabel, QLabel#jointValueLabel,"
        "QLabel#runtimeValueLabel, QLabel#currentValueLabel {"
        "  color: #56483f; font-size: 13px; font-weight: 600; }"
        "QLabel#trafficValueLabel { color: #56483f; font-size: 13px; font-weight: 600; }"
        "QComboBox, QDoubleSpinBox {"
        "  min-height: 36px; border: 1px solid #ddcfc2; border-radius: 10px; padding: 4px 10px;"
        "  background: #fffaf6; color: #56483f; }"
        "QComboBox:disabled, QDoubleSpinBox:disabled { background: #f3ece6; color: #a39386; }"
        "QStatusBar { background: transparent; color: #7d695b; }"));
}

void MainWindow::applyProfileUiState(int mode)
{
    const bool enabled = mode != AppConfig::kControlModeNormal;
    ui->profileVelJ1SpinBox->setEnabled(enabled);
    ui->profileAccJ1SpinBox->setEnabled(enabled);
    ui->profileJerkJ1SpinBox->setEnabled(enabled);
    ui->profileVelJ2SpinBox->setEnabled(enabled);
    ui->profileAccJ2SpinBox->setEnabled(enabled);
    ui->profileJerkJ2SpinBox->setEnabled(enabled);
}

void MainWindow::refreshUi()
{
    const ControllerStatus status = m_controller->statusSnapshot();
    const auto forward = ArmKinematics::forward(status.joint);
    m_canvas->setScene(forward.points, QPointF(status.target.x, status.target.y));

    ui->targetValueLabel->setText(QStringLiteral("X %1 mm   Y %2 mm")
                                      .arg(status.target.x * 1000.0, 0, 'f', 1)
                                      .arg(status.target.y * 1000.0, 0, 'f', 1));
    ui->cartValueLabel->setText(QStringLiteral("X %1 mm   Y %2 mm")
                                    .arg(status.cart.x * 1000.0, 0, 'f', 1)
                                    .arg(status.cart.y * 1000.0, 0, 'f', 1));
    ui->cmdValueLabel->setText(QStringLiteral("J1 %1°   J2 %2°")
                                   .arg(status.cmdJoint.th1, 0, 'f', 1)
                                   .arg(status.cmdJoint.th2, 0, 'f', 1));
    ui->jointValueLabel->setText(QStringLiteral("J1 %1° / %2°/s   J2 %3° / %4°/s")
                                     .arg(status.joint.th1, 0, 'f', 1)
                                     .arg(status.joint.w1, 0, 'f', 0)
                                     .arg(status.joint.th2, 0, 'f', 1)
                                     .arg(status.joint.w2, 0, 'f', 0));
    ui->runtimeValueLabel->setText(QStringLiteral("%1 | %2 | %3 Hz")
                                       .arg(status.mode)
                                       .arg(status.controlModeName)
                                       .arg(status.hz, 0, 'f', 0));
    ui->currentValueLabel->setText(QStringLiteral("IQ1 %1 A   IQ2 %2 A   G1 %3 A   G2 %4 A")
                                       .arg(status.ctrl.iq1 + status.ctrl.grav1, 0, 'f', 3)
                                       .arg(status.ctrl.iq2 + status.ctrl.grav2, 0, 'f', 3)
                                       .arg(status.ctrl.grav1, 0, 'f', 3)
                                       .arg(status.ctrl.grav2, 0, 'f', 3));
    ui->trafficValueLabel->setText(QStringLiteral("%1 | RX %2   TX %3   ERR %4 | %5")
                                       .arg(status.canConnected ? QStringLiteral("在线") : QStringLiteral("离线"))
                                       .arg(status.rxFrames)
                                       .arg(status.txFrames)
                                       .arg(status.errTxFrames)
                                       .arg(status.channel));

    ui->connectionStateButton->setText(status.canConnected ? QStringLiteral("CAN FD 已连接")
                                                           : QStringLiteral("CAN FD 未连接"));
    ui->connectionStateButton->setProperty("connected", status.canConnected);
    ui->connectionStateButton->style()->unpolish(ui->connectionStateButton);
    ui->connectionStateButton->style()->polish(ui->connectionStateButton);

    if (status.profile.controlMode == AppConfig::kControlModeNormal) {
        ui->stateBadgeLabel->setText(QStringLiteral("NORMAL"));
    } else if (status.profile.controlMode == AppConfig::kControlModeTrapezoidal) {
        ui->stateBadgeLabel->setText(QStringLiteral("TRAPEZOID"));
    } else {
        ui->stateBadgeLabel->setText(QStringLiteral("S CURVE"));
    }

    m_syncingUi = true;

    {
        const QSignalBlocker blocker(ui->controlModeComboBox);
        const int index = ui->controlModeComboBox->findData(status.profile.controlMode);
        if (index >= 0 && ui->controlModeComboBox->currentIndex() != index) {
            ui->controlModeComboBox->setCurrentIndex(index);
        }
    }

    syncSpinBoxValue(ui->stiffnessJ1SpinBox, status.profile.stiffness1);
    syncSpinBoxValue(ui->dampingJ1SpinBox, status.profile.damping1);
    syncSpinBoxValue(ui->stiffnessJ2SpinBox, status.profile.stiffness2);
    syncSpinBoxValue(ui->dampingJ2SpinBox, status.profile.damping2);
    syncSpinBoxValue(ui->profileVelJ1SpinBox, status.profile.velocity1);
    syncSpinBoxValue(ui->profileAccJ1SpinBox, status.profile.acceleration1);
    syncSpinBoxValue(ui->profileJerkJ1SpinBox, status.profile.jerk1);
    syncSpinBoxValue(ui->profileVelJ2SpinBox, status.profile.velocity2);
    syncSpinBoxValue(ui->profileAccJ2SpinBox, status.profile.acceleration2);
    syncSpinBoxValue(ui->profileJerkJ2SpinBox, status.profile.jerk2);

    applyProfileUiState(status.profile.controlMode);
    m_syncingUi = false;
}

void MainWindow::syncSpinBoxValue(QDoubleSpinBox *spinBox, double value)
{
    if (!spinBox || spinBox->hasFocus()) {
        return;
    }

    const QSignalBlocker blocker(spinBox);
    spinBox->setValue(value);
}

void MainWindow::handleCanvasClick(double x, double y)
{
    const ControllerStatus status = m_controller->statusSnapshot();
    if (!ArmKinematics::isTargetReachable(QPointF(x, y), status.joint)) {
        statusBar()->showMessage(QStringLiteral("目标点不可达，或 J1 超出允许范围"), 3500);
        return;
    }

    m_controller->setTarget(x, y);
    statusBar()->showMessage(
        QStringLiteral("目标已更新：X %1 mm，Y %2 mm")
            .arg(x * 1000.0, 0, 'f', 1)
            .arg(y * 1000.0, 0, 'f', 1),
        2500);
}
