#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QVector3D>
#include <QDockWidget>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui_;
    rclcpp::Node::SharedPtr node_;
    std::thread ros_thread_;
    Widget::PointCloudWidget *viewer_;
    std::vector<Widget::ViewerPanel*> panels_;
    // QOpenGLWidget *openGLWidget;
    int panelCount_ = 6; // Number of panels

};

#endif // MAINWINDOW_H
