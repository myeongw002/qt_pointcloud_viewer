#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "mainwindow.hpp"
#include <QDebug>
#include <QLoggingCategory>
#include <QDateTime>

QCoreApplication* app_ptr = nullptr;

void handle_sigint(int) {
    std::cout << "SIGINT received. Exiting Qt application..." << std::endl;
    if (app_ptr) {
        app_ptr->quit();  // Exit Qt event loop
    }
}

int main(int argc, char *argv[]) {
    
    rclcpp::init(argc, argv);    
    QApplication app(argc, argv);

    // Set global log message pattern
    qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss.zzz}][%{type}]: %{message}");
    
    // Or more detailed pattern
    // qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss.zzz}] %{type} %{file}:%{line} - %{message}");
    
    qRegisterMetaType<Widget::CloudConstPtr>("CloudConstPtr");
    qRegisterMetaType<Widget::PathConstPtr>("PathConstPtr");
    
    MainWindow window;
    window.show();
    
    app_ptr = &app;  // Store QApplication object in global pointer
    std::signal(SIGINT, handle_sigint);    

    return app.exec();
    
}
