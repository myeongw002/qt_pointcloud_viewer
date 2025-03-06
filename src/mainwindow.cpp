#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    node = rclcpp::Node::make_shared("qt_pointcloud_viewer");
    
    viewer = dynamic_cast<PointCloudWidget*>(ui->openGLWidget);
    if (!viewer) {
        qFatal("Failed to cast to PointCloudWidget");
    }
    //viewer->setRosNode(node);
    connect(ui->start_button, &QPushButton::clicked, this, &MainWindow::startStreaming);

    // ✅ Start ROS spinning in a separate thread
    ros_thread = std::thread([this]() { rclcpp::spin(node); });
}

MainWindow::~MainWindow() {
    if (ros_thread.joinable()) {
        rclcpp::shutdown();
        ros_thread.join();
    }
    delete ui;
}

void MainWindow::startStreaming() {
    updateStatus("Listening for PointCloud2 data...");
}

void MainWindow::updateStatus(const QString &status) {
    ui->status_label->setText(status);
}
