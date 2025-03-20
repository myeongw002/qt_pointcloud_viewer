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

    // Attempt qobject_cast
    PointCloudWidget *viewer = qobject_cast<PointCloudWidget *>(ui->openGLWidget);
    this->viewer = viewer;
    // std::cout << "Viewer after cast: " << viewer << std::endl;

    viewer->setNode(node);

    connect(ui->start_button, &QPushButton::clicked, this, &MainWindow::startStreaming);
    connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &MainWindow::onComboBoxIndexChanged);
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
    if (this->current_index == 0) {
        std::cout << "❌ No topic selected. Please select a topic." << std::endl;
        return ;
    } 

    this->viewer->setStartFlag(true);
}

void MainWindow::updateStatus(const QString &status) {
    ui->status_label->setText(status);
    ui->status_label->adjustSize(); 
}

void MainWindow::onComboBoxIndexChanged(int index) {
    QString status = "";

    this->current_index = index;
    std::cout << "Index changed to: " << this->current_index << std::endl;
    this->viewer->setTopicName(index);

    std::string topic_name = this->viewer->getTopicName();

    if (topic_name == "/") {
        status = QString::fromStdString("No topic selected. Please select a topic.");
    }
    else {
        status = QString::fromStdString("Listening for PointCloud2 data..." + topic_name);
        this->viewer->setStartFlag(true);
    }
    updateStatus(status);
}