#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "pointcloud_widget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startStreaming();
    void updateStatus(const QString &status);

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;
    std::thread ros_thread;
    PointCloudWidget *viewer;
};

#endif // MAINWINDOW_H
