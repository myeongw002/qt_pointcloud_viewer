#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "mainwindow.hpp"
#include <QDebug>

QCoreApplication* app_ptr = nullptr;

void handle_sigint(int) {
    std::cout << "SIGINT received. Exiting Qt application..." << std::endl;
    if (app_ptr) {
        app_ptr->quit();  // Qt 이벤트 루프 종료
    }
}

int main(int argc, char *argv[]) {
    
    rclcpp::init(argc, argv);    
    QApplication app(argc, argv);

    qRegisterMetaType<Widget::CloudConstPtr>("CloudConstPtr");
    qRegisterMetaType<Widget::PathConstPtr>("PathConstPtr");
    
    MainWindow window;
    window.show();
    
    app_ptr = &app;  // 전역 포인터에 QApplication 객체 저장
    std::signal(SIGINT, handle_sigint);    

    return app.exec();
    
}
