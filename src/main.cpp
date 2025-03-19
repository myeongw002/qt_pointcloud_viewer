#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "mainwindow.h"
#include <QDebug>

int main(int argc, char *argv[]) {
    try {
        // ROS2 및 Qt 초기화
    std::cout << "Initializing ROS2 and Qt..." << std::endl;    
    
    rclcpp::init(argc, argv);
    std::cout << "1" << std::endl;   
    
    QApplication app(argc, argv);
    std::cout << "2" << std::endl;    
    
    MainWindow window;
    std::cout << "3" << std::endl;
    
    window.show();
    std::cout << "4" << std::endl;
    
    return app.exec();
    }
    catch (const std::exception &e) {
        std::cout << "Unhandled Exception:" << e.what() << std::endl;
    } catch (...) {
        std::cout << "Unknown Exception occurred!" << std::endl;
    }
    
    return -1;
}
