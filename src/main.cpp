#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "mainwindow.h"
#include <QDebug>

int main(int argc, char *argv[]) {
    try {
        // ROS2 및 Qt 초기화
    
    rclcpp::init(argc, argv);    
    QApplication app(argc, argv);
    MainWindow window;
    
    window.show();
    
    return app.exec();
    }
    catch (const std::exception &e) {
        std::cout << "Unhandled Exception:" << e.what() << std::endl;
    } catch (...) {
        std::cout << "Unknown Exception occurred!" << std::endl;
    }
    
    return -1;
}
