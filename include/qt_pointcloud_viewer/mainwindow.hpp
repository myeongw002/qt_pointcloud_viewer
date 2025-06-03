#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QVector3D>
#include <QDockWidget>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include "float_widget.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    

private slots:
    // void handleDockRequest(); // 도킹 요청 처리 슬롯

    
private:
    Ui::MainWindow *ui_;
    rclcpp::Node::SharedPtr node_;
    std::thread ros_thread_;
    Widget::PointCloudWidget *viewer_;
    Widget::FloatWidget *floatWidget_;
    std::vector<Widget::ViewerPanel*> panels_;
    // QOpenGLWidget *openGLWidget;
    int panelCount_ = 6; // Number of panels
    bool getWidgetGridPosition(QWidget* widget, int& row, int& col); // 위치 조회 함수

};

#endif // MAINWINDOW_H
