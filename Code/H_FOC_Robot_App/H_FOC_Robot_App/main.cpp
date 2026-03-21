/**
 * @file main.cpp
 * @brief Qt 应用程序入口，负责创建 QApplication 并启动主窗口。
 */

#include "ui/mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
