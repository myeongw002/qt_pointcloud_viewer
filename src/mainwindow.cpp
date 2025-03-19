#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {

    try {
        std::cout << "Setting up UI..." << std::endl;
        ui->setupUi(this);
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception in setupUi(): " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ Unknown Exception in setupUi()" << std::endl;
    }

    node = rclcpp::Node::make_shared("qt_pointcloud_viewer");
    

    // viewer = new PointCloudWidget(this, node);
    // std::cout << "Viewer before cast: " << ui->openGLWidget << std::endl;

    // if (ui->openGLWidget) {
    //     std::cout << "openGLWidget Type (RTTI): " << typeid(*ui->openGLWidget).name() << std::endl;

    //     if (ui->openGLWidget->metaObject()) {
    //         std::cout << "openGLWidget Qt Type: " << ui->openGLWidget->metaObject()->className() << std::endl;
    //     } else {
    //         std::cerr << "Warning: openGLWidget has no metaObject()" << std::endl;
    //     }
    // } else {
    //     std::cerr << "Error: ui->openGLWidget is nullptr" << std::endl;
    //     return;
    // }

    // Attempt qobject_cast
    PointCloudWidget *viewer = qobject_cast<PointCloudWidget *>(ui->openGLWidget);
    // std::cout << "Viewer after cast: " << viewer << std::endl;

    viewer->setSubscription(node);

    // if (!ui->centralwidget->layout()) {
    //     std::cout << "Creating new layout for pointcloud_view" << std::endl;
    //     ui->centralwidget->setLayout(new QVBoxLayout());
    // }
    

    // viewer -> setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);


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
