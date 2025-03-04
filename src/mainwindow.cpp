#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    node = rclcpp::Node::make_shared("qt_pointcloud_viewer");

    viewer = new PointCloudWidget(this, node);  // ✅ Pass ROS node to PointCloudWidget

    if (!ui->pointcloud_view->layout()) {
        std::cout << "Creating new layout for pointcloud_view" << std::endl;
        ui->pointcloud_view->setLayout(new QVBoxLayout());
    }
    ui->pointcloud_view->layout()->addWidget(viewer);
    
    viewer -> setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addWidget(ui->pointcloud_view);  // OpenGL widget
    mainLayout->addWidget(ui->status_label);     // Status label
    mainLayout->addWidget(ui->start_button);     // Button

    QWidget *centralWidget = new QWidget();
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

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
