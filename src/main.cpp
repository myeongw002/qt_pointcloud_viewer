#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[]) {
    // ROS2 및 Qt 초기화
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    return app.exec();
}
